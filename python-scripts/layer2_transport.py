import struct
import time

PACKET_HEADER_FORMAT = '!HBBBBIHHH'
PACKET_HEADER_SIZE = struct.calcsize(PACKET_HEADER_FORMAT)

SYNC = 0b1010101010101010
VERSION = 1

SENSOR_TYPE_LIDAR = 1
SENSOR_TYPE_CAMERA = 2

FLAG_LAST_FRAGMENT = 0x01
FLAG_COMPRESSED = 0x02

MAX_PAYLOAD = 1400
STALE_TIMEOUT_SEC = 5.0


def fragment_payload(payload: bytes, sensor_type: int, sensor_id: int, frame_id: int, compressed: bool = False):
    total_size = len(payload)
    total_frags = max(1, (total_size + MAX_PAYLOAD - 1) // MAX_PAYLOAD)
    base_flags = FLAG_COMPRESSED if compressed else 0

    packets = []
    for i in range(total_frags):
        start = i * MAX_PAYLOAD
        chunk = payload[start:start + MAX_PAYLOAD]
        flags = base_flags | (FLAG_LAST_FRAGMENT if i == total_frags - 1 else 0)

        header = struct.pack(
            PACKET_HEADER_FORMAT,
            SYNC, VERSION, sensor_type, sensor_id, flags,
            frame_id, total_frags, i, len(chunk),
        )
        packets.append(header + chunk)

    return packets


def reassemble_feed(buffers: dict, packet: bytes):
    if len(packet) < PACKET_HEADER_SIZE:
        return None

    header = packet[:PACKET_HEADER_SIZE]
    (sync, version, sensor_type, sensor_id, flags,
     frame_id, total_frags, frag_index, payload_len) = struct.unpack(PACKET_HEADER_FORMAT, header)

    if sync != SYNC or version != VERSION:
        return None

    payload = packet[PACKET_HEADER_SIZE: PACKET_HEADER_SIZE + payload_len]
    key = (sensor_type, sensor_id, frame_id)
    now = time.monotonic()

    compressed = bool(flags & FLAG_COMPRESSED)
    entry = buffers.setdefault(key, {"total": total_frags, "parts": {}, "last_seen": now, "compressed": compressed})

    if total_frags != entry["total"] or not (0 <= frag_index < entry["total"]):
        return None

    entry["parts"][frag_index] = payload
    entry["last_seen"] = now

    if len(entry["parts"]) < entry["total"]:
        return None

    ordered = [entry["parts"].get(i) for i in range(entry["total"])]
    if any(chunk is None for chunk in ordered):
        return None

    complete_blob = b"".join(ordered)
    was_compressed = entry["compressed"]
    del buffers[key]

    return {
        "sensor_type": sensor_type,
        "sensor_id": sensor_id,
        "frame_id": frame_id,
        "payload": complete_blob,
        "compressed": was_compressed,
    }


def purge_stale_buffers(buffers: dict, timeout_sec: float = STALE_TIMEOUT_SEC):
    now = time.monotonic()
    stale_keys = [k for k, v in buffers.items() if now - v["last_seen"] > timeout_sec]
    for k in stale_keys:
        del buffers[k]
    return len(stale_keys)