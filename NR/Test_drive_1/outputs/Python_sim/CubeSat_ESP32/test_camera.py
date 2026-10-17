import random
import unittest
from unittest.mock import patch

from Sim_Environment.camera_transport import encode_rle_frame
from . import sil
from .camera import detect_black_target


class CameraDetectorTests(unittest.TestCase):
    @staticmethod
    def reference_detector(gray, width, height, stride, threshold=40,
                           min_dark_pixels=50, max_dark_pixels=200):
        xs, ys = [], []
        raw_count = 0
        for y in range(height):
            for x in range(width):
                if gray[y * stride + x] > threshold:
                    continue
                raw_count += 1
                neighbours = sum(
                    gray[ny * stride + nx] <= threshold
                    for ny in range(max(0, y - 1), min(height, y + 2))
                    for nx in range(max(0, x - 1), min(width, x + 2))
                    if nx != x or ny != y
                )
                if neighbours >= 2:
                    xs.append(x)
                    ys.append(y)
        count = len(xs)
        if not min_dark_pixels <= count <= max_dark_pixels:
            return {"valid": False, "dark_pixel_count": raw_count,
                    "coherent_pixel_count": count}
        cx, cy = sum(xs) / count, sum(ys) / count
        return {"valid": True, "dark_pixel_count": raw_count,
                "coherent_pixel_count": count, "centroid_x_px": cx,
                "centroid_y_px": cy,
                "error_x_px": cx - (width - 1) / 2,
                "error_y_px": cy - (height - 1) / 2}

    def test_camera_frame_request_requires_exact_sequence_and_qvga_length(self):
        class Response:
            headers = {
                "X-Camera-Frame-Sequence": "17",
                "X-Camera-Frame-Encoding": "RLE1",
                "X-Camera-Frame-Width": str(sil.CAMERA_WIDTH_PX),
                "X-Camera-Frame-Height": str(sil.CAMERA_HEIGHT_PX),
            }

            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            def read(self, _size):
                return encode_rle_frame(bytes([190]) * sil.CAMERA_FRAME_BYTES)

        with patch.object(sil, "urlopen", return_value=Response()) as open_url:
            frame = sil.request_camera_frame(17)
            self.assertEqual(len(frame), sil.CAMERA_FRAME_BYTES)
            self.assertEqual(frame[0], 190)
            self.assertEqual(frame[-1], 190)
        self.assertIn("/api/camera-frame.rle?sequence=17",
                      open_url.call_args.args[0].full_url)
        with patch.object(sil, "urlopen", return_value=Response()):
            with self.assertRaisesRegex(ValueError, "sequence"):
                sil.request_camera_frame(18)

    def test_camera_frame_request_rejects_unknown_encoding(self):
        class Response:
            headers = {
                "X-Camera-Frame-Sequence": "17",
                "X-Camera-Frame-Encoding": "RAW",
                "X-Camera-Frame-Width": str(sil.CAMERA_WIDTH_PX),
                "X-Camera-Frame-Height": str(sil.CAMERA_HEIGHT_PX),
            }

            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            def read(self, _size):
                return b""

        with patch.object(sil, "urlopen", return_value=Response()):
            with self.assertRaisesRegex(ValueError, "encoding"):
                sil.request_camera_frame(17)

    def test_camera_frame_request_enforces_compressed_payload_bound(self):
        class Response:
            headers = {
                "X-Camera-Frame-Sequence": "17",
                "X-Camera-Frame-Encoding": "RLE1",
                "X-Camera-Frame-Width": str(sil.CAMERA_WIDTH_PX),
                "X-Camera-Frame-Height": str(sil.CAMERA_HEIGHT_PX),
            }

            def __enter__(self):
                return self

            def __exit__(self, *_args):
                return False

            def read(self, size):
                self.size = size
                return bytes(size)

        response = Response()
        with patch.object(sil, "urlopen", return_value=response):
            with self.assertRaisesRegex(ValueError, "transfer bound"):
                sil.request_camera_frame(17)
        self.assertEqual(response.size, sil.CAMERA_RLE_MAX_BYTES + 1)

    def test_detects_target_centroid_with_padded_stride(self):
        width, height, stride = 320, 240, 324
        image = bytearray([255] * (stride * height))
        for y in range(95, 105):
            for x in range(170, 180):
                image[y * stride + x] = 0
        result = detect_black_target(image, width, height, stride=stride)
        self.assertTrue(result["valid"])
        self.assertEqual(result["dark_pixel_count"], 100)
        self.assertAlmostEqual(result["error_x_px"], 15.0)
        self.assertAlmostEqual(result["error_y_px"], -20.0)

    def test_rejects_empty_and_overlarge_dark_areas(self):
        image = bytes([255] * (320 * 240))
        self.assertFalse(detect_black_target(image, 320, 240)["valid"])
        dark = bytes([0] * (320 * 240))
        self.assertFalse(detect_black_target(dark, 320, 240)["valid"])

    def test_detector_rejects_isolated_dark_noise_and_keeps_coherent_blob(self):
        width = height = 32
        image = bytearray([190] * (width * height))
        for y in range(2, 32, 4):
            for x in range(2, 32, 4):
                image[y * width + x] = 0
        isolated = detect_black_target(image, width, height,
                                       min_dark_pixels=1,
                                       max_dark_pixels=200)
        self.assertFalse(isolated["valid"])
        self.assertEqual(isolated["dark_pixel_count"], 64)
        self.assertEqual(isolated["coherent_pixel_count"], 0)

        image[:] = bytes([190] * len(image))
        for y in range(11, 21):
            for x in range(11, 21):
                image[y * width + x] = 0
        blob = detect_black_target(image, width, height)
        self.assertTrue(blob["valid"])
        self.assertEqual(blob["dark_pixel_count"], 100)
        self.assertEqual(blob["coherent_pixel_count"], 100)
        self.assertAlmostEqual(blob["error_x_px"], 0.0)
        self.assertAlmostEqual(blob["error_y_px"], 0.0)

    def test_rejects_bad_buffer_or_dimensions(self):
        with self.assertRaises(ValueError):
            detect_black_target(bytes(10), 320, 240)
        with self.assertRaises(ValueError):
            detect_black_target(bytes(320 * 240), 700, 240)

    def test_detector_configuration_matches_c_field_ranges(self):
        image = bytes([255] * (16 * 16))
        with self.assertRaisesRegex(ValueError, "threshold"):
            detect_black_target(image, 16, 16, threshold=256)
        with self.assertRaisesRegex(ValueError, "area bounds"):
            detect_black_target(image, 16, 16, max_dark_pixels=257)
        with self.assertRaises(ValueError):
            detect_black_target(image, 16, 16, min_dark_pixels=1.0)

    def test_bitset_detector_matches_direct_neighbour_reference(self):
        generator = random.Random(9182)
        for _ in range(100):
            width = generator.randint(1, 25)
            height = generator.randint(1, 20)
            stride = width + generator.randint(0, 3)
            image = bytearray([190] * (stride * height))
            for index in range(len(image)):
                if generator.random() < 0.35:
                    image[index] = generator.choice((0, 12, 40, 41, 190, 255))
            kwargs = {"stride": stride, "threshold": 40,
                      "min_dark_pixels": 1,
                      "max_dark_pixels": width * height}
            expected = self.reference_detector(image, width, height, **kwargs)
            self.assertEqual(
                detect_black_target(image, width, height, **kwargs), expected
            )

    def test_dense_dark_scene_reports_exact_area_without_centroid(self):
        width, height = 320, 240
        image = bytes([0]) * (width * height)
        result = detect_black_target(image, width, height)
        self.assertFalse(result["valid"])
        self.assertEqual(result["dark_pixel_count"], width * height)
        self.assertEqual(result["coherent_pixel_count"], width * height)

if __name__ == "__main__":
    unittest.main()
