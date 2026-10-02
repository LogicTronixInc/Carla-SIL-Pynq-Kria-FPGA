import socket
import queue
import threading
import time
import numpy as np
from scapy.layers.inet import IP, UDP

from layer1_camera import build_camera_payload, PIXEL_FORMAT_RGBA, PIXEL_FORMAT_BGR
from layer1_lidar import build_lidar_payload
from layer2_transport import fragment_payload, SENSOR_TYPE_CAMERA, SENSOR_TYPE_LIDAR

QUEUE_MAXSIZE = 800
PACING_BATCH_SIZE = 100
PACING_SLEEP_SEC = 0.0005
NUM_CAMERA_WORKERS = 2
NUM_LIDAR_WORKERS = 2 

LIDAR_SENSOR_ID = 0

SRC_IP = "192.168.1.1"
DST_IP = "192.168.1.2"
SRC_PORT = 5000
DST_PORT = 6000


def build_ip_udp_packet(payload: bytes) -> bytes:
    pkt = IP(src=SRC_IP, dst=DST_IP, id=0, ttl=255) / UDP(sport=SRC_PORT, dport=DST_PORT) / payload
    return bytes(pkt)


def carla_image_to_array(image) -> np.ndarray:
    return np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))


class NetworkSender:
    def __init__(self, dest_ip: str, dest_port: int,
                 num_camera_workers: int = NUM_CAMERA_WORKERS, num_lidar_workers: int = NUM_LIDAR_WORKERS):
        self.dest = (dest_ip, dest_port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 16 * 1024 * 1024)

        self._camera_queue = queue.Queue(maxsize=QUEUE_MAXSIZE)
        self._lidar_queue = queue.Queue(maxsize=QUEUE_MAXSIZE)
        self._running = True

        self._camera_workers = [threading.Thread(target=self._camera_loop, daemon=True)
                                for _ in range(num_camera_workers)]
        self._lidar_workers = [threading.Thread(target=self._lidar_loop, daemon=True)
                               for _ in range(num_lidar_workers)]
        for w in self._camera_workers + self._lidar_workers:
            w.start()

    def queue_frame(self, frame_bgra, timestamp, sensor_id, frame_id):
        try:
            self._camera_queue.put_nowait((frame_bgra, timestamp, sensor_id, frame_id))
        except queue.Full:
            print(f"WARNING: camera queue full, dropping frame sensor={sensor_id} frame={frame_id}")

    def queue_lidar(self, points_xyzi, timestamp, sensor_id, frame_id):
        try:
            self._lidar_queue.put_nowait((points_xyzi, timestamp, sensor_id, frame_id))
        except queue.Full:
            print(f"WARNING: lidar queue full, dropping scan sensor={sensor_id} frame={frame_id}")

    def _send_packets(self, packets):
        for i, p in enumerate(packets):
            self.sock.sendto(p, self.dest)
            if i % PACING_BATCH_SIZE == 0:
                time.sleep(PACING_SLEEP_SEC)

    def _camera_loop(self):
        while self._running:
            try:
                frame_bgra, timestamp, sensor_id, frame_id = self._camera_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                compress = True
                #for compress = true use frame_bgra[:,:,:3] -> drops alpha channel

                payload = build_camera_payload(frame_bgra[:,:,:3], timestamp=timestamp, pixel_format=PIXEL_FORMAT_BGR, compress=compress)
                # payload = build_camera_payload(frame_bgra, timestamp=timestamp, pixel_format=PIXEL_FORMAT_BGRA, compress=compress) #uncompressed
                raw_fragments = fragment_payload(payload, SENSOR_TYPE_CAMERA, sensor_id=sensor_id, frame_id=frame_id, compressed=compress)
                
                # Wrap each fragment in an IP/UDP header for hardware parsing
                packets = [build_ip_udp_packet(frag) for frag in raw_fragments]
                self._send_packets(packets)
            except Exception as e:
                print(f"CAMERA WORKER ERROR sensor={sensor_id} frame={frame_id}: {e}")
            finally:
                self._camera_queue.task_done()

    def _lidar_loop(self):
        while self._running:
            try:
                points_xyzi, timestamp, sensor_id, frame_id = self._lidar_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                compress = True
                payload = build_lidar_payload(points_xyzi, timestamp=timestamp, compress=compress)
                raw_fragments = fragment_payload(payload, SENSOR_TYPE_LIDAR, sensor_id=sensor_id, frame_id=frame_id, compressed=compress)
                
                # Wrap each fragment in an IP/UDP header for hardware parsing
                packets = [build_ip_udp_packet(frag) for frag in raw_fragments]
                self._send_packets(packets)
            except Exception as e:
                print(f"LIDAR WORKER ERROR sensor={sensor_id} frame={frame_id}: {e}")
            finally:
                self._lidar_queue.task_done()

    def close(self):
        self._camera_queue.join()
        self._lidar_queue.join()
        self._running = False
        for w in self._camera_workers + self._lidar_workers:
            w.join(timeout=2.0)
        self.sock.close()


def wrap_save_image_network(image, folder, sensor_id, save_image_fn, sender):
    try:
        frame_bgra = carla_image_to_array(image).copy()
        sender.queue_frame(frame_bgra, image.timestamp, sensor_id, image.frame)
    except Exception as e:
        print(f"CAMERA CALLBACK ERROR sensor={sensor_id} frame={image.frame}: {e}")


def wrap_save_lidar_network(point_cloud, save_lidar_fn, sender, sensor_id=LIDAR_SENSOR_ID):
    try:
        points_xyzi = np.frombuffer(point_cloud.raw_data, dtype=np.float32).reshape(-1, 4).copy()
        sender.queue_lidar(points_xyzi, point_cloud.timestamp, sensor_id, point_cloud.frame)
    except Exception as e:
        print(f"LIDAR CALLBACK ERROR sensor={sensor_id} frame={point_cloud.frame}: {e}")