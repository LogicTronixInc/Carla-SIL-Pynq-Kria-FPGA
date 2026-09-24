"""
Generates interleaved UDP test vector dataset (Camera + LiDAR) and golden references.

Outputs:
  - golden_outputs/udp_input_interleaved.dat : Interleaved network beat stream
  - golden_outputs/golden_camera_<frame_id>.h264 : Ground-truth H.264 bitstream
  - golden_outputs/golden_lidar.dat : Ground-truth 32-bit decompressed polar words
"""

import os
import glob
import random
import struct
import numpy as np
import cv2
from scapy.layers.inet import IP, UDP

from layer1_lidar import build_lidar_payload, _cartesian_to_quantized_polar
from layer1_camera import build_camera_h264_frame_payload, CAMERA_H264_META_SIZE
from layer2_transport import fragment_payload, SENSOR_TYPE_LIDAR, SENSOR_TYPE_CAMERA

# PATHS & CONFIGURATION
INPUT_DIR = "testbench_input"
CAM_DIR = os.path.join(INPUT_DIR, "received_camera")
LIDAR_DIR = os.path.join(INPUT_DIR, "received_lidar")

OUTPUT_DIR = "golden_outputs"
NUM_TARGET_FRAMES = 15

SRC_IP = "192.168.1.1"
DST_IP = "192.168.1.2"
SRC_PORT = 5000
DST_PORT = 6000

# HELPER FUNCTIONS
def build_ip_udp_packet(src_ip: str, dst_ip: str, src_port: int, dst_port: int, payload: bytes) -> bytes:
    pkt = IP(src=src_ip, dst=dst_ip, id=0, ttl=255) / UDP(sport=src_port, dport=dst_port) / payload
    return bytes(pkt)


def packet_to_beats(packet: bytes):
    beats = []
    n = len(packet)
    for offset in range(0, n, 4):
        chunk = packet[offset:offset + 4]
        valid_bytes = len(chunk)
        word = 0
        for lane, b in enumerate(chunk):
            word |= b << (8 * lane)
        keep = (1 << valid_bytes) - 1
        last = 1 if (offset + 4 >= n) else 0
        beats.append((word, keep, last))
    return beats


def make_word(b0, b1, b2, b3):
    """Matches HLS MakeWord(b0, b1, b2, b3): b0 at [7:0], b3 at [31:24]"""
    return ((b3 & 0xFF) << 24) | ((b2 & 0xFF) << 16) | ((b1 & 0xFF) << 8) | (b0 & 0xFF)


def generate_decompressed_golden_words(frame_id, sensor_id, compressed, points_xyzi, timestamp):
    """Generates ground truth 32-bit polar words matching HLS layer1_point_decoder output."""
    distance_raw, azimuth_raw, elevation_raw, reflectivity_raw = _cartesian_to_quantized_polar(points_xyzi)
    n_points = points_xyzi.shape[0]
    words = []

    # Header Word 0: frame_id (4B, big-endian byte order)
    fid_bytes = frame_id.to_bytes(4, 'big')
    words.append(make_word(fid_bytes[0], fid_bytes[1], fid_bytes[2], fid_bytes[3]))

    # Header Word 1: sensor_id(B0), compressed(B1), pad(B2, B3)
    words.append(make_word(sensor_id, 1 if compressed else 0, 0, 0))

    # Header Word 2: num_points (4B, big-endian byte order)
    np_bytes = int(n_points).to_bytes(4, 'big')
    words.append(make_word(np_bytes[0], np_bytes[1], np_bytes[2], np_bytes[3]))

    # Header Words 3 & 4: timestamp double (8B, big-endian)
    ts_bytes = struct.pack('!d', float(timestamp))
    words.append(make_word(ts_bytes[0], ts_bytes[1], ts_bytes[2], ts_bytes[3]))
    words.append(make_word(ts_bytes[4], ts_bytes[5], ts_bytes[6], ts_bytes[7]))

    # Point Payload: 2 words (wA, wB) per point
    for i in range(n_points):
        dist = int(distance_raw[i]) & 0xFFFF
        az   = int(azimuth_raw[i]) & 0xFFFF
        el   = int(elevation_raw[i]) & 0xFFFF
        refl = int(reflectivity_raw[i]) & 0xFF

        # wA: dist_raw [15:8],[7:0] and az_raw [15:8],[7:0]
        wA = make_word((dist >> 8) & 0xFF, dist & 0xFF, (az >> 8) & 0xFF, az & 0xFF)
        words.append(wA)

        # wB: el_raw [15:8],[7:0], reflectivity [7:0], pad [7:0]
        wB = make_word((el >> 8) & 0xFF, el & 0xFF, refl, 0)
        words.append(wB)

    return words


# MAIN GENERATOR
def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print("=" * 70)
    print(" UNIFIED INTERLEAVED DATASET GENERATOR")
    print("=" * 70)

    # ------------------------------------------------------------------------
    # 1. GENERATE CAMERA FRAMES (Grouped per Frame)
    # ------------------------------------------------------------------------
    cam_files = sorted(glob.glob(os.path.join(CAM_DIR, "*.png")))
    if not cam_files:
        raise FileNotFoundError(f"No PNG camera images found in {CAM_DIR}")
    
    raw_cam_images = [cv2.imread(f, cv2.IMREAD_COLOR) for f in cam_files]
    print(f"[CAM] Loaded {len(raw_cam_images)} source images. Generating {NUM_TARGET_FRAMES} frames...")

    camera_frames_list = []
    total_cam_packets = 0
    for i in range(NUM_TARGET_FRAMES):
        frame_id = 2000 + i
        img = raw_cam_images[i % len(raw_cam_images)]
        
        payload = build_camera_h264_frame_payload(img, timestamp=float(i) * 0.1)
        bitstream = payload[CAMERA_H264_META_SIZE:]

        golden_cam_path = os.path.join(OUTPUT_DIR, f"golden_camera_{frame_id}.h264")
        with open(golden_cam_path, "wb") as f:
            f.write(bitstream)

        fragments = fragment_payload(payload, SENSOR_TYPE_CAMERA, sensor_id=1,
                                     frame_id=frame_id, compressed=True)
        
        frame_packets = []
        for pkt in fragments:
            full_pkt = build_ip_udp_packet(SRC_IP, DST_IP, SRC_PORT, DST_PORT, pkt)
            frame_packets.append(packet_to_beats(full_pkt))
        
        camera_frames_list.append(frame_packets)
        total_cam_packets += len(frame_packets)

    print(f"  └─ Generated {total_cam_packets:,} total Camera UDP packets across {len(camera_frames_list)} frames.")

    # ------------------------------------------------------------------------
    # 2. GENERATE LIDAR SCANS (Grouped per Frame)
    # ------------------------------------------------------------------------
    lidar_files = sorted(glob.glob(os.path.join(LIDAR_DIR, "*.npy")))
    if not lidar_files:
        raise FileNotFoundError(f"No .npy LiDAR files found in {LIDAR_DIR}")
    
    raw_lidar_scans = [np.load(f).astype(np.float32) for f in lidar_files]
    print(f"[LIDAR] Loaded {len(raw_lidar_scans)} source scans. Generating {NUM_TARGET_FRAMES} frames...")

    lidar_frames_list = []
    golden_lidar_lines = []
    total_lidar_packets = 0

    for i in range(NUM_TARGET_FRAMES):
        frame_id = 1000 + i
        points_xyzi = raw_lidar_scans[i % len(raw_lidar_scans)]
        ts = float(i) * 0.1
        compressed = True
        sensor_id = 0

        # Build compressed network payload
        compressed_payload = build_lidar_payload(points_xyzi, timestamp=ts, compress=compressed)

        # Generate Ground Truth 32-bit decompressed words matching
        gt_words = generate_decompressed_golden_words(
            frame_id=frame_id, sensor_id=sensor_id, compressed=compressed,
            points_xyzi=points_xyzi, timestamp=ts
        )

        golden_lidar_lines.append(f"FRAME {frame_id} {len(gt_words)}\n")
        for w in gt_words:
            golden_lidar_lines.append(f"{w:08X}\n")

        fragments = fragment_payload(compressed_payload, SENSOR_TYPE_LIDAR, sensor_id=sensor_id,
                                     frame_id=frame_id, compressed=compressed)
        
        frame_packets = []
        for pkt in fragments:
            full_pkt = build_ip_udp_packet(SRC_IP, DST_IP, SRC_PORT, DST_PORT, pkt)
            frame_packets.append(packet_to_beats(full_pkt))
        
        lidar_frames_list.append(frame_packets)
        total_lidar_packets += len(frame_packets)

    golden_lidar_path = os.path.join(OUTPUT_DIR, "golden_lidar.dat")
    with open(golden_lidar_path, "w") as f:
        f.writelines(golden_lidar_lines)

    print(f"  └─ Generated {total_lidar_packets:,} total LiDAR UDP packets across {len(lidar_frames_list)} frames.")
    print(f"  └─ Wrote ground-truth LiDAR reference to '{golden_lidar_path}'")

    # ------------------------------------------------------------------------
    # 3. INTERLEAVE AT FRAME LEVEL & SAVE
    # ------------------------------------------------------------------------
    all_frames = []
    max_frames = max(len(camera_frames_list), len(lidar_frames_list))
    
    # Interleave frames sequentially: Cam 2000 -> LiDAR 1000 -> Cam 2001 -> LiDAR 1001...
    for idx in range(max_frames):
        if idx < len(camera_frames_list):
            all_frames.append(camera_frames_list[idx])
        if idx < len(lidar_frames_list):
            all_frames.append(lidar_frames_list[idx])

    # Flatten frames into ordered packet sequence
    all_packets = []
    for frame_pkts in all_frames:
        all_packets.extend(frame_pkts)

    interleaved_beats = []
    for pkt_beats in all_packets:
        interleaved_beats.extend(pkt_beats)

    input_path = os.path.join(OUTPUT_DIR, "udp_input_interleaved.dat")
    with open(input_path, "w") as f:
        for word, keep, last in interleaved_beats:
            f.write(f"{word:08X} {keep:01X} {last}\n")

    print(f"[COMPLETE] Output saved to '{input_path}' ({len(interleaved_beats):,} total beats)")
    print("=" * 70)

if __name__ == "__main__":
    main()