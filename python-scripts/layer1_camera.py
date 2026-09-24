import io
import struct
import av
import numpy as np

# H.264 ENCODER CONFIGURATION
H264_ENCODER_CONFIG = {
    "codec_name": "libx264",
    "pix_fmt": "yuv422p",       
    "profile": "high422",       
    "level": "4.0",             
    "crf": 18,                   
    "fps": 30,                   
    "gop_size": "1",            
    "x264_params": {
        "annexb": "1",        
    },
}

CAMERA_META_FORMAT = '!HHBd'  # width(H), height(H), pixel_format(B), timestamp(d)
CAMERA_META_SIZE = struct.calcsize(CAMERA_META_FORMAT)

CAMERA_H264_META_FORMAT = "!IIBBBd"  # width, height, gop_size, crf, profile_id, timestamp
CAMERA_H264_META_SIZE = struct.calcsize(CAMERA_H264_META_FORMAT)

DEFAULT_PROFILE = "high422"
PROFILE_IDS = {"baseline": 0, "main": 1, "high": 2, "high422": 3}
PROFILE_NAMES = {v: k for k, v in PROFILE_IDS.items()}

PIXEL_FORMAT_RGB = 0
PIXEL_FORMAT_BGR = 1
PIXEL_FORMAT_RGBA = 2
PIXEL_FORMAT_GRAY = 3

_CHANNELS_FOR_FORMAT = {
    PIXEL_FORMAT_RGB: 3,
    PIXEL_FORMAT_BGR: 3,
    PIXEL_FORMAT_RGBA: 4,
    PIXEL_FORMAT_GRAY: 1,
}


def build_camera_h264_frame_payload(
    frame: np.ndarray,
    timestamp: float,
    crf: int = H264_ENCODER_CONFIG["crf"],
    profile: str = H264_ENCODER_CONFIG["profile"],
) -> bytes:
    h, w = frame.shape[0], frame.shape[1]
    buf = io.BytesIO()
    container = av.open(buf, mode="w", format="h264")
    stream = container.add_stream(H264_ENCODER_CONFIG["codec_name"], rate=H264_ENCODER_CONFIG["fps"])
    
    stream.width = w
    stream.height = h
    stream.pix_fmt = H264_ENCODER_CONFIG["pix_fmt"]
    x264_options = {
        "crf": str(crf),
        "g": H264_ENCODER_CONFIG["gop_size"],
        "profile": profile,
        "level": H264_ENCODER_CONFIG["level"],
        "threads":"3"     # threads=1 makes libx264's output byte-reproducible across separate
                        # process invocations (multi-threaded encoding is not bit-exact run to run)
    }

    x264_options.update(H264_ENCODER_CONFIG["x264_params"])
    stream.options = x264_options

    av_frame = av.VideoFrame.from_ndarray(frame, format="bgr24")
    
    for packet in stream.encode(av_frame):
        container.mux(packet)
    for packet in stream.encode(None): 
        container.mux(packet)
    container.close()

    gop = int(H264_ENCODER_CONFIG["gop_size"])
    profile_id = PROFILE_IDS.get(profile, PROFILE_IDS["high422"])

    meta = struct.pack(CAMERA_H264_META_FORMAT, w, h, gop, crf, profile_id, timestamp)
    return meta + buf.getvalue()


def parse_camera_h264_frame_payload(blob: bytes):
    w, h, gop_size, crf, profile_id, timestamp = struct.unpack(
        CAMERA_H264_META_FORMAT, blob[:CAMERA_H264_META_SIZE]
    )
    bitstream = blob[CAMERA_H264_META_SIZE:]
    buf = io.BytesIO(bitstream)
    
    container = av.open(buf, mode="r", format="h264")
    frames = [f.to_ndarray(format="bgr24") for f in container.decode(video=0)]
    container.close()
    return frames[0], timestamp


def build_camera_payload(
    frame: np.ndarray,
    timestamp: float,
    pixel_format: int = PIXEL_FORMAT_BGR,
    compress: bool = False,
    crf: int = H264_ENCODER_CONFIG["crf"],
) -> bytes:
    if compress:
        if pixel_format not in (PIXEL_FORMAT_RGB, PIXEL_FORMAT_BGR):
            raise ValueError("H.264 compression requires RGB/BGR standard 3-channel frame input.")
        frame_bgr = frame[:, :, ::-1] if pixel_format == PIXEL_FORMAT_RGB else frame
        return build_camera_h264_frame_payload(frame_bgr, timestamp, crf=crf)

    height, width = frame.shape[0], frame.shape[1]
    meta = struct.pack(CAMERA_META_FORMAT, width, height, pixel_format, timestamp)
    return meta + frame.tobytes()


def parse_camera_payload(blob: bytes, compressed: bool = False):
    if compressed:
        frame_bgr, timestamp = parse_camera_h264_frame_payload(blob)
        return frame_bgr, PIXEL_FORMAT_BGR, timestamp

    width, height, pixel_format, timestamp = struct.unpack(CAMERA_META_FORMAT, blob[:CAMERA_META_SIZE])
    channels = _CHANNELS_FOR_FORMAT[pixel_format]
    pixels = np.frombuffer(blob[CAMERA_META_SIZE:], dtype=np.uint8)
    shape = (height, width) if channels == 1 else (height, width, channels)
    return pixels.reshape(shape), pixel_format, timestamp