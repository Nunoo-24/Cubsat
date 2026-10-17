"""Host reference for the firmware's bounded dark-target detector."""

import math


def detect_black_target(gray, width, height, *, stride=None, threshold=40,
                        min_dark_pixels=50, max_dark_pixels=200):
    stride = width if stride is None else stride
    integer_args = (width, height, stride, threshold,
                    min_dark_pixels, max_dark_pixels)
    if (any(isinstance(value, bool) or not isinstance(value, int)
            for value in integer_args) or
            not 1 <= width <= 640 or not 1 <= height <= 480 or
            stride < width):
        raise ValueError("image dimensions or stride are outside supported bounds")
    if len(gray) < stride * height:
        raise ValueError("grayscale buffer is shorter than stride × height")
    if (not 0 <= threshold <= 255 or min_dark_pixels < 1 or
            max_dark_pixels < min_dark_pixels or
            max_dark_pixels > width * height):
        raise ValueError("invalid threshold or dark-pixel area bounds")
    # Reject isolated threshold crossings while retaining coherent dark
    # blobs. Encode each row as an integer bitset so the same 8-neighbour
    # rule can be evaluated with bitwise operations instead of scanning up to
    # eight Python pixel comparisons for every dark pixel. Portable C uses the
    # equivalent direct-neighbour loop.
    dark_rows = []
    raw_count = 0
    for y in range(height):
        row = y * stride
        dark_bits = 0
        for x in range(width):
            if gray[row + x] <= threshold:
                dark_bits |= 1 << x
                raw_count += 1
        dark_rows.append(dark_bits)

    width_mask = (1 << width) - 1
    coherent_count = 0
    sum_x = sum_y = 0
    sum_xx = sum_yy = sum_xy = 0
    sum_xxx = sum_yyy = sum_xxy = sum_xyy = 0
    for y, current in enumerate(dark_rows):
        above = dark_rows[y - 1] if y else 0
        below = dark_rows[y + 1] if y + 1 < height else 0
        neighbour_masks = (
            above >> 1, above, (above << 1) & width_mask,
            current >> 1, (current << 1) & width_mask,
            below >> 1, below, (below << 1) & width_mask,
        )
        seen_once = 0
        seen_twice = 0
        for neighbours in neighbour_masks:
            seen_twice |= seen_once & neighbours
            seen_once |= neighbours
        coherent = seen_twice & current
        row_count = coherent.bit_count()
        coherent_count += row_count
        # A rejected area has no centroid to calculate. Still count all
        # coherent pixels so the diagnostic result remains exact.
        if coherent_count <= max_dark_pixels:
            while coherent:
                lowest = coherent & -coherent
                x = lowest.bit_length() - 1
                sum_x += x
                sum_y += y
                sum_xx += x * x
                sum_yy += y * y
                sum_xy += x * y
                sum_xxx += x * x * x
                sum_yyy += y * y * y
                sum_xxy += x * x * y
                sum_xyy += x * y * y
                coherent ^= lowest

    count = coherent_count
    if not min_dark_pixels <= count <= max_dark_pixels:
        return {"valid": False, "dark_pixel_count": raw_count,
                "coherent_pixel_count": count}
    cx, cy = sum_x / count, sum_y / count
    cov_xx = sum_xx / count - cx * cx
    cov_yy = sum_yy / count - cy * cy
    cov_xy = sum_xy / count - cx * cy
    principal_angle = 0.5 * math.atan2(2.0 * cov_xy, cov_xx - cov_yy)
    trace = cov_xx + cov_yy
    anisotropy = math.hypot(cov_xx - cov_yy, 2.0 * cov_xy)
    roll_orientation_rad = None
    if trace > 0.0 and anisotropy / trace >= 0.18:
        # The asymmetric fiducial has a long axis and a distinguishable end.
        # Third central moment selects the direction along the principal axis;
        # a plain point/circle has no valid orientation and returns None.
        c, sn = math.cos(principal_angle), math.sin(principal_angle)
        m30 = (sum_xxx - 3 * cx * sum_xx + 2 * count * cx ** 3)
        m03 = (sum_yyy - 3 * cy * sum_yy + 2 * count * cy ** 3)
        m21 = (sum_xxy - cy * sum_xx - 2 * cx * sum_xy
               + 2 * count * cx * cx * cy + cx * cx * sum_y
               - count * cx * cx * cy)
        m12 = (sum_xyy - cx * sum_yy - 2 * cy * sum_xy
               + 2 * count * cx * cy * cy + cy * cy * sum_x
               - count * cx * cy * cy)
        skew = (c ** 3 * m30 + 3 * c * c * sn * m21
                + 3 * c * sn * sn * m12 + sn ** 3 * m03)
        if abs(skew) > 0.05:
            if skew < 0.0:
                principal_angle += math.pi
            # Renderer defines the marker's forward direction toward the
            # smaller satellite; the third moment points toward the larger one.
            roll_orientation_rad = math.atan2(
                math.sin(principal_angle + math.pi),
                math.cos(principal_angle + math.pi))
    return {
        "valid": True,
        "dark_pixel_count": raw_count,
        "coherent_pixel_count": count,
        "centroid_x_px": cx,
        "centroid_y_px": cy,
        "error_x_px": cx - (width - 1) / 2,
        "error_y_px": cy - (height - 1) / 2,
        "roll_orientation_rad": roll_orientation_rad,
    }
