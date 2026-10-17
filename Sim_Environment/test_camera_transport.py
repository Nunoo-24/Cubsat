import unittest

from .camera_transport import decode_rle_frame, encode_rle_frame


class CameraFrameCodecTests(unittest.TestCase):
    def test_round_trips_frame_and_splits_runs_above_u16_limit(self):
        frame = bytes([190]) * 76800
        encoded = encode_rle_frame(frame)
        self.assertLess(len(encoded), 16)
        self.assertEqual(decode_rle_frame(encoded, len(frame)), frame)

    def test_round_trips_alternating_and_mixed_image_runs(self):
        frame = bytes([190] * 8 + [12] * 5 + [190] * 9 + [0, 255])
        self.assertEqual(decode_rle_frame(encode_rle_frame(frame), len(frame)),
                         frame)

    def test_decoder_rejects_malformed_and_wrong_sized_frames(self):
        malformed = (
            b"bad!\x01\x00\xbe",       # invalid magic
            b"RLE1\x01",                # incomplete run record
            b"RLE1\x00\x00\xbe",       # zero-length run
            b"RLE1\x04\x00\xbe",       # run longer than the image
            b"RLE1\x02\x00\xbe",       # decoded image too short
        )
        for payload in malformed:
            with self.subTest(payload=payload), self.assertRaises(ValueError):
                decode_rle_frame(payload, 3)

    def test_decoder_requires_positive_integer_output_size(self):
        encoded = encode_rle_frame(b"\xbe")
        for expected in (0, -1, 1.5, True):
            with self.subTest(expected=expected), self.assertRaises(ValueError):
                decode_rle_frame(encoded, expected)


if __name__ == "__main__":
    unittest.main()
