import argparse
import glob
import os
import socket
import struct
import sys
import time
import cv2
import numpy as np
from scapy.layers.inet import IP, UDP

from layer1_camera import (
    PIXEL_FORMAT_BGR,
    build_camera_h264_frame_payload,
    build_camera_payload,
    CAMERA_H264_META_FORMAT,
    H264_ENCODER_CONFIG,
    PROFILE_IDS
)
from layer1_lidar import build_lidar_payload
from layer2_transport import (
    SENSOR_TYPE_CAMERA,
    SENSOR_TYPE_LIDAR,
    fragment_payload,
)

# Static Network Protocol Headers
SRC_IP = "192.168.1.1"
DST_IP = "192.168.1.2"
SRC_PORT = 5000
DST_PORT = 6000

# Input Directories
INPUT_DIR = "testbench_input"
CAM_DIR = os.path.join(INPUT_DIR, "received_camera")
LIDAR_DIR = os.path.join(INPUT_DIR, "received_lidar")


def build_ip_udp_packet(
    src_ip: str, dst_ip: str, src_port: int, dst_port: int, payload: bytes
) -> bytes:
    pkt = IP(src=src_ip, dst=dst_ip, id=0, ttl=255) / UDP(sport=src_port, dport=dst_port) / payload
    return bytes(pkt)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Stream benchmark camera and LiDAR UDP packets to an FPGA hardware target.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    # Required Arguments (No default values)
    parser.add_argument(
        "--fpga-ip",
        type=str,
        required=True,
        help="Target IP address of the FPGA board.",
    )
    parser.add_argument(
        "--fpga-port",
        type=int,
        required=True,
        help="Target UDP port on the FPGA board.",
    )

    # Optional Config Arguments (With Defaults)
    parser.add_argument(
        "--total-frames",
        type=int,
        default=200,
        help="Total number of combined frames to stream during the test.",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=25.0,
        help="Target streaming FPS limit (set to 0 for uncapped throughput test).",
    )
    parser.add_argument(
        "--unique-frames",
        type=int,
        default=15,
        help="Number of unique sensor frames to pre-build and cycle through.",
    )
    parser.add_argument(
        "--packet-delay",
        type=float,
        default=0.00003,
        help="Artificial delay (in seconds) between consecutive packet transmissions.",
    )

    # Pre-compressed H264 overrides
    parser.add_argument(
        "--use-precompressed",
        action="store_true",
        help="Skip live PyAV H.264 encoding and use pre-compressed .h264 bitstreams instead.",
    )
    parser.add_argument(
        "--h264-dir",
        type=str,
        default="golden_outputs",
        help="Directory containing pre-compressed .h264 files (used if --use-precompressed is set).",
    )

    # Sensor selection
    parser.add_argument(
        "--sensor",
        type=str,
        default="both",
        choices=["both", "lidar", "camera"],
        help="Which sensor(s) to stream: 'both', 'lidar', or 'camera'.",
    )

    # Convenience aliases (mutually exclusive)
    alias_group = parser.add_mutually_exclusive_group()
    alias_group.add_argument(
        "--lidar-only",
        action="store_true",
        help="Alias for --sensor lidar.",
    )
    alias_group.add_argument(
        "--camera-only",
        action="store_true",
        help="Alias for --sensor camera.",
    )

    return parser.parse_args()


def main():
    args = parse_args()

    fpga_ip = args.fpga_ip
    fpga_port = args.fpga_port
    total_frames_to_send = args.total_frames
    target_fps = args.fps
    num_unique_frames = args.unique_frames
    packet_delay_sec = args.packet_delay
    use_precomp = args.use_precompressed

    # Resolve final sensor mode
    if args.lidar_only:
        sensor_mode = "lidar"
    elif args.camera_only:
        sensor_mode = "camera"
    else:
        sensor_mode = args.sensor

    send_camera = sensor_mode in ("both", "camera")
    send_lidar = sensor_mode in ("both", "lidar")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print("=" * 70)
    print(f" STARTING BENCHMARK UDP SENDER -> Target: {fpga_ip}:{fpga_port}")
    print(f" Sensor Mode: {sensor_mode.upper()}")
    print(f" Target Frames: {total_frames_to_send} | Target FPS: {target_fps if target_fps > 0 else 'UNCAPPED'}")
    print(f" Pre-built Unique Sets: {num_unique_frames} | Inter-packet Delay: {packet_delay_sec}s")
    print(f" Live Encoding: {'DISABLED (Using .h264 files)' if use_precomp else 'ENABLED'}")
    print("=" * 70)

    # 1. Load LiDAR Files (only if needed)
    raw_lidar_scans = []
    if send_lidar:
        lidar_files = sorted(glob.glob(os.path.join(LIDAR_DIR, "*.npy")))
        if not lidar_files:
            raise FileNotFoundError(f"Missing LiDAR input files in {LIDAR_DIR}/")
        raw_lidar_scans = [np.load(f).astype(np.float32) for f in lidar_files]
        print(f"[INFO] Loaded {len(raw_lidar_scans)} LiDAR scans.")
    else:
        print("[INFO] LiDAR disabled — skipping LiDAR input loading.")

    # 2. Load Camera Files (only if needed)
    raw_cam_images = []
    raw_h264_streams = []
    cam_h = cam_w = gop = crf = profile_id = None

    if send_camera:
        cam_files = sorted(glob.glob(os.path.join(CAM_DIR, "*.png")))
        if not cam_files:
            raise FileNotFoundError(f"Missing Camera input files in {CAM_DIR}/")

        if use_precomp:
            h264_files = sorted(glob.glob(os.path.join(args.h264_dir, "*.h264")))
            if not h264_files:
                raise FileNotFoundError(
                    f"Missing .h264 files in {args.h264_dir}/. Generate them first."
                )
            raw_h264_streams = [open(f, "rb").read() for f in h264_files]

            sample_img = cv2.imread(cam_files[0], cv2.IMREAD_COLOR)
            cam_h, cam_w = sample_img.shape[:2]
            gop = int(H264_ENCODER_CONFIG["gop_size"])
            crf = H264_ENCODER_CONFIG["crf"]
            profile_id = PROFILE_IDS.get(
                H264_ENCODER_CONFIG["profile"], PROFILE_IDS["high422"]
            )
            print(f"[INFO] Loaded {len(raw_h264_streams)} pre-compressed H.264 streams.")
        else:
            raw_cam_images = [cv2.imread(f, cv2.IMREAD_COLOR) for f in cam_files]
            print(f"[INFO] Loaded {len(raw_cam_images)} raw Camera PNGs.")
    else:
        print("[INFO] Camera disabled — skipping Camera input loading.")

    print(f"[INFO] Pre-building {num_unique_frames} unique frames and network packets into RAM...")
    unique_frames = []

    for i in range(num_unique_frames):
        ts = float(i) * 0.1
        frame_entry = {}

        # --- CAMERA PAYLOAD ---
        if send_camera:
            cam_frame_id = 2000 + i
            if use_precomp:
                bitstream = raw_h264_streams[i % len(raw_h264_streams)]
                meta = struct.pack(
                    CAMERA_H264_META_FORMAT, cam_w, cam_h, gop, crf, profile_id, ts
                )
                cam_payload = meta + bitstream
            else:
                img = raw_cam_images[i % len(raw_cam_images)]
                cam_payload = build_camera_h264_frame_payload(img, timestamp=ts)

            cam_frags = fragment_payload(
                cam_payload,
                SENSOR_TYPE_CAMERA,
                sensor_id=1,
                frame_id=cam_frame_id,
                compressed=True,
            )
            cam_packets = [
                build_ip_udp_packet(SRC_IP, DST_IP, SRC_PORT, DST_PORT, frag)
                for frag in cam_frags
            ]
            frame_entry["cam_fid"] = cam_frame_id
            frame_entry["cam_packets"] = cam_packets

        # --- LIDAR PAYLOAD ---
        if send_lidar:
            lidar_frame_id = 1000 + i
            points = raw_lidar_scans[i % len(raw_lidar_scans)]
            lidar_payload = build_lidar_payload(points, timestamp=ts, compress=True)
            lidar_frags = fragment_payload(
                lidar_payload,
                SENSOR_TYPE_LIDAR,
                sensor_id=0,
                frame_id=lidar_frame_id,
                compressed=True,
            )
            lidar_packets = [
                build_ip_udp_packet(SRC_IP, DST_IP, SRC_PORT, DST_PORT, frag)
                for frag in lidar_frags
            ]
            frame_entry["lidar_fid"] = lidar_frame_id
            frame_entry["lidar_packets"] = lidar_packets

        unique_frames.append(frame_entry)

    total_sent_packets = 0
    total_sent_bytes = 0
    target_frame_time = (1.0 / target_fps) if target_fps > 0 else 0.0

    print("=" * 70)
    print(f"[INFO] Streaming {total_frames_to_send} frames (sensor={sensor_mode})...")
    print("=" * 70)

    t_start = time.time()

    for frame_idx in range(total_frames_to_send):
        frame_t_start = time.time()
        frame = unique_frames[frame_idx % num_unique_frames]

        if send_camera:
            for full_packet in frame["cam_packets"]:
                sock.sendto(full_packet, (fpga_ip, fpga_port))
                total_sent_packets += 1
                total_sent_bytes += len(full_packet)
                if packet_delay_sec > 0:
                    time.sleep(packet_delay_sec)

        if send_lidar:
            for full_packet in frame["lidar_packets"]:
                sock.sendto(full_packet, (fpga_ip, fpga_port))
                total_sent_packets += 1
                total_sent_bytes += len(full_packet)
                if packet_delay_sec > 0:
                    time.sleep(packet_delay_sec)

        if target_frame_time > 0:
            elapsed_frame = time.time() - frame_t_start
            sleep_time = target_frame_time - elapsed_frame
            if sleep_time > 0:
                time.sleep(sleep_time)

        if (frame_idx + 1) % 20 == 0 or (frame_idx + 1) == total_frames_to_send:
            print(f" Sent Frame {frame_idx + 1}/{total_frames_to_send}")

    t_end = time.time()

    print("[INFO] Transmission complete. Sending EOF_STREAM signal...")
    time.sleep(0.2)
    for _ in range(5):
        sock.sendto(b"EOF_STREAM", (fpga_ip, fpga_port))

    elapsed_total = t_end - t_start
    total_mb = total_sent_bytes / (1024 * 1024)
    mbps = (total_sent_bytes * 8) / (elapsed_total * 1e6) if elapsed_total > 0 else 0

    print("=" * 70)
    print(" SENDER BENCHMARK SUMMARY")
    print("=" * 70)
    print(f" Sensor Mode         : {sensor_mode.upper()}")
    print(f" Total Frames Sent   : {total_frames_to_send}")
    print(f" Total Packets Sent  : {total_sent_packets:,}")
    print(f" Total Data Sent     : {total_mb:.2f} MB")
    print(f" Total Active Time   : {elapsed_total:.3f} s")
    print(f" Sender FPS          : {total_frames_to_send / elapsed_total:.2f} FPS")
    print(f" Sender Throughput   : {mbps:.2f} Mbps ({total_mb / elapsed_total:.2f} MB/s)")
    print("=" * 70)

    sock.close()


if __name__ == "__main__":
    main()