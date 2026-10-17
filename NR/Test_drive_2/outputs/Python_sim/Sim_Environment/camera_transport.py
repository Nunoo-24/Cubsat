"""Lossless, bounded run-length codec for simulator camera frames."""

MAGIC = b"RLE1"
RUN_HEADER_BYTES = 3
MAX_RUN_LENGTH = (1 << 16) - 1
MAX_ENCODED_BYTES = 8192


def encode_rle_frame(frame):
    """Encode grayscale bytes as ``RLE1`` + little-endian u16/value runs."""
    if not isinstance(frame, (bytes, bytearray, memoryview)):
        raise TypeError("frame must be a bytes-like grayscale buffer")
    pixels = memoryview(frame).cast("B")
    output = bytearray(MAGIC)
    index = 0
    while index < len(pixels):
        value = pixels[index]
        end = index + 1
        while end < len(pixels) and pixels[end] == value and (
                end - index < MAX_RUN_LENGTH):
            end += 1
        run_length = end - index
        output.extend(run_length.to_bytes(2, "little"))
        output.append(value)
        index = end
    return bytes(output)


def decode_rle_frame(encoded, expected_pixels):
    """Decode a strict RLE1 frame, rejecting malformed or wrong-sized data."""
    if not isinstance(expected_pixels, int) or isinstance(expected_pixels, bool) or expected_pixels < 1:
        raise ValueError("expected_pixels must be a positive integer")
    if not isinstance(encoded, (bytes, bytearray, memoryview)):
        raise TypeError("encoded frame must be bytes-like")
    data = memoryview(encoded).cast("B")
    if len(data) > MAX_ENCODED_BYTES:
        raise ValueError("camera frame RLE1 payload exceeds transfer bound")
    if len(data) < len(MAGIC) or bytes(data[:len(MAGIC)]) != MAGIC:
        raise ValueError("camera frame has an invalid RLE1 header")
    if (len(data) - len(MAGIC)) % RUN_HEADER_BYTES:
        raise ValueError("camera frame has an incomplete RLE1 run")

    output = bytearray()
    for offset in range(len(MAGIC), len(data), RUN_HEADER_BYTES):
        run_length = data[offset] | (data[offset + 1] << 8)
        value = data[offset + 2]
        if run_length == 0 or run_length > expected_pixels - len(output):
            raise ValueError("camera frame RLE1 run exceeds expected image size")
        output.extend(bytes((value,)) * run_length)
    if len(output) != expected_pixels:
        raise ValueError("camera frame RLE1 data has the wrong decoded size")
    return bytes(output)
