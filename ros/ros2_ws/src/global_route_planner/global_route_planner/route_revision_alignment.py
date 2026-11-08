"""Conservative world-XY alignment of two directed route polylines.

Use only after identity logic confirms a new route revision in the
same mission and map. Route-relative coordinates are not assumed
to be interchangeable across revisions.

Geometric alignment is not lane identity or permission for motion.
"""

from dataclasses import dataclass
import math

from global_route_planner.route_geometry import (
    interpolate_polyline_at_s,
)


@dataclass(frozen=True)
class RouteAlignmentParameters:
    sample_spacing_m: float = 1.0
    max_check_length_m: float = 30.0
    min_overlap_m: float = 8.0
    max_position_error_m: float = 1.0
    max_heading_error_deg: float = 15.0
    min_progress_fraction: float = 0.5

    def __post_init__(self):
        values = (
            self.sample_spacing_m,
            self.max_check_length_m,
            self.min_overlap_m,
            self.max_position_error_m,
            self.max_heading_error_deg,
            self.min_progress_fraction,
        )

        if not all(math.isfinite(v) for v in values):
            raise ValueError("Alignment parameters must be finite")

        if not 0.0 < self.sample_spacing_m <= self.min_overlap_m / 2.0:
            raise ValueError("Invalid alignment sample spacing")

        if not 0.0 < self.min_overlap_m <= self.max_check_length_m:
            raise ValueError("Invalid alignment comparison lengths")

        if self.max_position_error_m <= 0.0:
            raise ValueError("Position tolerance must be positive")

        if not 0.0 < self.max_heading_error_deg <= 180.0:
            raise ValueError("Heading tolerance outside (0, 180]")

        if not 0.0 < self.min_progress_fraction <= 1.0:
            raise ValueError("Progress fraction outside (0, 1]")

        if self.max_check_length_m / self.sample_spacing_m > 1000:
            raise ValueError("Alignment sampling would be excessive")


@dataclass(frozen=True)
class RouteRevisionAlignment:
    status: str
    reason: str
    checked_length_m: float
    matched_prefix_m: float
    previous_start_s_m: float | None
    previous_end_s_m: float | None
    max_observed_position_error_m: float
    max_observed_heading_error_deg: float


def _segments(points):
    if len(points) < 2:
        raise ValueError("Route requires at least two XY points")

    if not all(
        math.isfinite(x) and math.isfinite(y)
        for x, y in points
    ):
        raise ValueError("Route contains non-finite XY coordinates")

    segments = []
    accumulated_s = 0.0

    for (ax, ay), (bx, by) in zip(points, points[1:]):
        dx, dy = bx - ax, by - ay
        length = math.hypot(dx, dy)

        if length <= 0.0:
            continue

        segments.append(
            (accumulated_s, ax, ay, dx, dy, length)
        )
        accumulated_s += length

    if not math.isfinite(accumulated_s) or accumulated_s <= 0.0:
        raise ValueError("Route has invalid or zero arc length")

    return segments, accumulated_s


def _directed_projection(segments, x, y, heading, heading_limit):
    """Find a direction-compatible projection on the old route."""

    best = None

    for start_s, ax, ay, dx, dy, length in segments:
        old_heading = math.atan2(dy, dx)

        error_deg = abs(math.degrees(math.atan2(
            math.sin(heading - old_heading),
            math.cos(heading - old_heading),
        )))

        if error_deg > heading_limit:
            continue

        t = max(
            0.0,
            min(
                1.0,
                ((x - ax) * dx + (y - ay) * dy)
                / (length * length),
            ),
        )

        distance = math.hypot(
            x - (ax + t * dx),
            y - (ay + t * dy),
        )

        old_s = start_s + t * length

        # Closest position, then closest heading.
        # Ties use earlier previous-route coordinates.
        candidate = (distance, error_deg, old_s)

        if best is None or candidate < best:
            best = candidate

    return best


def compare_route_revisions(
    previous_points: list[tuple[float, float]],
    current_points: list[tuple[float, float]],
    parameters: RouteAlignmentParameters | None = None,
) -> RouteRevisionAlignment:
    """Compare the new route prefix against the old world geometry.

    ALIGNED means every sampled point in the checked prefix matches.

    The projected previous-route s values are diagnostics, not a
    global transformation between route coordinate systems.
    """

    params = parameters or RouteAlignmentParameters()

    previous_segments, previous_length = _segments(previous_points)
    _, current_length = _segments(current_points)

    checked_length = min(
        current_length,
        params.max_check_length_m,
    )

    def result(
        status,
        reason,
        matched=0.0,
        start=None,
        end=None,
        max_xy=0.0,
        max_heading=0.0,
    ):
        return RouteRevisionAlignment(
            status=status,
            reason=reason,
            checked_length_m=checked_length,
            matched_prefix_m=matched,
            previous_start_s_m=start,
            previous_end_s_m=end,
            max_observed_position_error_m=max_xy,
            max_observed_heading_error_deg=max_heading,
        )

    if checked_length < params.min_overlap_m:
        return result(
            "INSUFFICIENT_OVERLAP",
            "SHORT_NEW_ROUTE",
        )

    # Include both endpoints and intermediate samples.
    sample_positions = [0.0]

    for i in range(
        1,
        int(checked_length / params.sample_spacing_m) + 1,
    ):
        s = min(
            checked_length,
            i * params.sample_spacing_m,
        )

        if s > sample_positions[-1] + 1e-8:
            sample_positions.append(s)

    if checked_length > sample_positions[-1] + 1e-8:
        sample_positions.append(checked_length)

    old_start = None
    last_old_s = None
    last_new_s = None

    matched = 0.0
    max_xy = 0.0
    max_heading = 0.0

    for new_s in sample_positions:
        sample = interpolate_polyline_at_s(
            current_points,
            new_s,
        )

        projection = _directed_projection(
            previous_segments,
            sample.x,
            sample.y,
            sample.heading_rad,
            params.max_heading_error_deg,
        )

        if (
            projection is None
            or projection[0] > params.max_position_error_m
        ):
            return result(
                "DIVERGED",
                "POSITION_OR_HEADING_MISMATCH",
                matched,
                old_start,
                last_old_s,
                max_xy,
                max_heading,
            )

        xy_error, heading_error, old_s = projection

        # The projected point must advance along the old route
        # as we advance along the new one.
        if last_old_s is not None:
            minimum_progress = (
                params.min_progress_fraction
                * (new_s - last_new_s)
            )

            if old_s - last_old_s < minimum_progress - 1e-6:
                return result(
                    "DIVERGED",
                    "NON_FORWARD_PROJECTION",
                    matched,
                    old_start,
                    last_old_s,
                    max_xy,
                    max_heading,
                )

        if old_start is None:
            old_start = old_s

            if (
                previous_length - old_start
                < params.min_overlap_m - 1e-6
            ):
                return result(
                    "INSUFFICIENT_OVERLAP",
                    "SHORT_PREVIOUS_REMAINDER",
                    0.0,
                    old_start,
                    old_start,
                )

        last_old_s = old_s
        last_new_s = new_s

        matched = new_s
        max_xy = max(max_xy, xy_error)
        max_heading = max(max_heading, heading_error)

    return result(
        "ALIGNED",
        "DIRECTED_PREFIX_ALIGNED",
        matched,
        old_start,
        last_old_s,
        max_xy,
        max_heading,
    )
