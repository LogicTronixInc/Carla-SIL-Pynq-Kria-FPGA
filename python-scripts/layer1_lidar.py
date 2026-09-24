import struct
import numpy as np

DISTANCE_RESOLUTION_M = 0.05
ANGLE_RESOLUTION_DEG = 0.1
BLOCK_SIZE = 256
MAX_WIDTH = 15

META_FORMAT = '!Id' + 'II' * 3
META_SIZE = struct.calcsize(META_FORMAT)


def _cartesian_to_quantized_polar(points_xyzi: np.ndarray):
    x, y, z, intensity = points_xyzi[:, 0], points_xyzi[:, 1], points_xyzi[:, 2], points_xyzi[:, 3]
    r = np.sqrt(x**2 + y**2 + z**2)
    azimuth_deg = np.degrees(np.arctan2(x, y)) % 360.0
    elevation_deg = np.degrees(np.arcsin(np.clip(z / np.maximum(r, 1e-6), -1.0, 1.0)))

    distance_raw = np.round(r / DISTANCE_RESOLUTION_M).astype(np.int64)
    azimuth_raw = np.round(azimuth_deg / ANGLE_RESOLUTION_DEG).astype(np.int64)
    elevation_raw = np.round(elevation_deg / ANGLE_RESOLUTION_DEG).astype(np.int64)
    reflectivity_raw = np.clip(intensity * 255.0, 0, 255).astype(np.uint8)

    return distance_raw, azimuth_raw, elevation_raw, reflectivity_raw


def _quantized_polar_to_cartesian(distance_raw, azimuth_raw, elevation_raw, reflectivity_raw):
    r = distance_raw.astype(np.float64) * DISTANCE_RESOLUTION_M
    az = np.deg2rad(azimuth_raw.astype(np.float64) * ANGLE_RESOLUTION_DEG)
    el = np.deg2rad(elevation_raw.astype(np.float64) * ANGLE_RESOLUTION_DEG)

    x = r * np.cos(el) * np.sin(az)
    y = r * np.cos(el) * np.cos(az)
    z = r * np.sin(el)
    intensity = reflectivity_raw.astype(np.float32) / 255.0

    return np.stack([x, y, z, intensity], axis=1).astype(np.float32)


def _zigzag_encode(deltas: np.ndarray) -> np.ndarray:
    return np.where(deltas >= 0, 2 * deltas, -2 * deltas - 1).astype(np.uint64)


def _zigzag_decode(zz: np.ndarray) -> np.ndarray:
    return (zz >> 1) ^ -(zz & 1).astype(np.int64)


def _bits_needed(max_val: int) -> int:
    return 1 if max_val <= 0 else max(1, int(max_val).bit_length())


def _pack_bits(values: np.ndarray, width: int) -> bytes:
    if len(values) == 0:
        return b""
    bit_positions = np.arange(width - 1, -1, -1, dtype=np.uint64)
    bits = ((values[:, None] >> bit_positions[None, :]) & np.uint64(1)).astype(np.uint8)
    flat = bits.reshape(-1)
    pad = (-len(flat)) % 8
    if pad:
        flat = np.concatenate([flat, np.zeros(pad, dtype=np.uint8)])
    return np.packbits(flat).tobytes()


def _unpack_bits(data: bytes, width: int, count: int) -> np.ndarray:
    if count == 0:
        return np.zeros(0, dtype=np.uint64)
    bits = np.unpackbits(np.frombuffer(data, dtype=np.uint8))[:count * width].reshape(count, width)
    weights = (1 << np.arange(width - 1, -1, -1, dtype=np.uint64))
    return bits @ weights


def _encode_field_blocked(raw_values: np.ndarray, block_size: int = BLOCK_SIZE):
    deltas = np.diff(raw_values, prepend=0)
    zz = _zigzag_encode(deltas)
    n = len(zz)

    num_blocks = (n + block_size - 1) // block_size
    widths = np.empty(num_blocks, dtype=np.uint8)
    block_bodies = []

    for b in range(num_blocks):
        start = b * block_size
        end = min(start + block_size, n)
        block = zz[start:end]
        width = min(_bits_needed(int(block.max())), MAX_WIDTH)
        widths[b] = width
        block_bodies.append(_pack_bits(block, width))

    return b"".join(block_bodies), widths.tobytes()


def _decode_field_blocked(body: bytes, widths_bytes: bytes, count: int, block_size: int = BLOCK_SIZE) -> np.ndarray:
    widths = np.frombuffer(widths_bytes, dtype=np.uint8)
    num_blocks = len(widths)
    out = np.empty(count, dtype=np.uint64)
    offset_bytes = 0

    for b in range(num_blocks):
        start = b * block_size
        end = min(start + block_size, count)
        block_count = end - start
        width = int(widths[b])

        body_len = (block_count * width + 7) // 8
        block_bytes = body[offset_bytes:offset_bytes + body_len]
        offset_bytes += body_len

        out[start:end] = _unpack_bits(block_bytes, width, block_count)

    deltas = _zigzag_decode(out.astype(np.int64))
    return np.cumsum(deltas)


def build_lidar_payload(points_xyzi: np.ndarray, timestamp: float, compress: bool = False) -> bytes:
    n = points_xyzi.shape[0]
    distance_raw, azimuth_raw, elevation_raw, reflectivity_raw = _cartesian_to_quantized_polar(points_xyzi)

    body_parts = []
    header_parts = [n, timestamp]

    for raw, dtype in ((distance_raw, np.uint16), (azimuth_raw, np.uint16), (elevation_raw, np.int16)):
        if compress:
            body, widths = _encode_field_blocked(raw)
        else:
            body, widths = raw.astype(dtype).tobytes(), b""
        header_parts += [len(body), len(widths)]
        body_parts += [body, widths]

    meta = struct.pack(META_FORMAT, *header_parts)
    return meta + b"".join(body_parts) + reflectivity_raw.tobytes()


def parse_lidar_payload(blob: bytes, compressed: bool = False):
    header = struct.unpack(META_FORMAT, blob[:META_SIZE])
    n, timestamp = header[0], header[1]
    field_headers = [header[2 + i * 2: 4 + i * 2] for i in range(3)]

    offset = META_SIZE
    raws = []
    for (body_len, widths_len), dtype in zip(field_headers, (np.uint16, np.uint16, np.int16)):
        body = blob[offset:offset + body_len]
        offset += body_len
        widths = blob[offset:offset + widths_len]
        offset += widths_len

        if compressed:
            raws.append(_decode_field_blocked(body, widths, n))
        else:
            raws.append(np.frombuffer(body, dtype=dtype).astype(np.int64))

    distance_raw, azimuth_raw, elevation_raw = raws
    reflectivity_raw = np.frombuffer(blob[offset:offset + n], dtype=np.uint8)

    points_xyzi = _quantized_polar_to_cartesian(distance_raw, azimuth_raw, elevation_raw, reflectivity_raw)
    return points_xyzi, timestamp