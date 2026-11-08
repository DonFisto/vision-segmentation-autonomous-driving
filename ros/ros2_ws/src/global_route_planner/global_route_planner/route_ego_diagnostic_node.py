"""Diagnose live RoutePlan and ego odometry integration."""

import copy
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)

from autonomy_interfaces.msg import (
    LaneMap, RoutePlan, RouteAssociationStatus,
)
from nav_msgs.msg import Odometry

from global_route_planner.lane_route_coverage import (
    estimate_lane_route_coverage,
)

from global_route_planner.lane_route_forward_coverage import (
    summarize_forward_coverage,
)

from global_route_planner.lane_route_decision import (
    evaluate_lane_route_association,
)

from global_route_planner.route_support_policy import (
    RouteSupportDecision,
    evaluate_route_support,
)
from global_route_planner.lane_map_provenance_gate import (
    LaneMapProvenance,
    classify_lane_map_provenance,
)
from global_route_planner.lane_map_evidence_time import (
    classify_evidence_time,
)

from global_route_planner.lane_map_support_selection import (
    SegmentSupportCandidate,
    select_lane_map_support,
)
from global_route_planner.route_identity import RouteIdentity
from global_route_planner.route_support_temporal import (
    TemporalRouteSupportTracker,
)

from global_route_planner.lane_route_geometry import (
    LanePoint,
    compare_lane_to_route,
)

from global_route_planner.route_geometry import (
    extract_route_window,
    project_point_to_polyline,
)


def quaternion_to_yaw(quaternion):
    """Extract planar yaw in radians from a normalized ROS quaternion."""

    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)

    components = (x, y, z, w)

    if not all(math.isfinite(value) for value in components):
        raise ValueError("Quaternion contains non-finite values")

    norm = math.sqrt(x*x + y*y + z*z + w*w)

    if not math.isfinite(norm) or norm < 1e-6:
        raise ValueError("Quaternion has invalid norm")

    x /= norm
    y /= norm
    z /= norm
    w /= norm

    sin_yaw = 2.0 * (w*z + x*y)
    cos_yaw = 1.0 - 2.0 * (y*y + z*z)

    return math.atan2(sin_yaw, cos_yaw)


def normalize_angle(angle_rad):
    """Normalize an angle to [-pi, pi)."""

    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


class RouteEgoDiagnosticNode(Node):
    """Project the live ego pose onto the active global route."""

    def __init__(self):
        super().__init__("route_ego_diagnostic")

        self.declare_parameter("frame_id", "carla_world")
        self.declare_parameter("max_projection_distance_m", 5.0)
        self.declare_parameter("max_heading_error_deg", 45.0)
        self.declare_parameter("max_odom_age_s", 2.0)
        self.declare_parameter("report_period_s", 1.0)
        self.declare_parameter("window_backward_m", 10.0)
        self.declare_parameter("window_forward_m", 50.0)
        self.declare_parameter("max_lane_map_age_s", 2.0)
        self.declare_parameter("max_lane_odom_delta_s", 0.5)

        self._frame_id = str(
            self.get_parameter("frame_id").value
        )

        self._max_projection_distance_m = float(
            self.get_parameter("max_projection_distance_m").value
        )

        self._max_heading_error_deg = float(
            self.get_parameter("max_heading_error_deg").value
        )

        self._max_odom_age_s = float(
            self.get_parameter("max_odom_age_s").value
        )

        self._report_period_s = float(
            self.get_parameter("report_period_s").value
        )

        self._window_backward_m = float(
            self.get_parameter("window_backward_m").value
        )

        self._window_forward_m = float(
            self.get_parameter("window_forward_m").value
        )

        self._max_lane_map_age_s = float(
            self.get_parameter("max_lane_map_age_s").value
        )

        self._max_lane_odom_delta_s = float(
            self.get_parameter("max_lane_odom_delta_s").value
        )

        for name, value in (
            ("max_projection_distance_m", self._max_projection_distance_m),
            ("max_heading_error_deg", self._max_heading_error_deg),
            ("max_odom_age_s", self._max_odom_age_s),
            ("report_period_s", self._report_period_s),
            ("max_lane_map_age_s", self._max_lane_map_age_s),
            ("max_lane_odom_delta_s", self._max_lane_odom_delta_s),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")

        for name, value in (
            ("window_backward_m", self._window_backward_m),
            ("window_forward_m", self._window_forward_m),
        ):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(
                    f"{name} must be finite and nonnegative"
                )

        if self._max_heading_error_deg > 180.0:
            raise ValueError(
                "max_heading_error_deg cannot exceed 180"
            )

        self._active_route = None
        self._route_points = None

        self._latest_odom = None
        self._latest_odom_time = None

        self._odom_received = False

        self._latest_lane_map = None
        self._latest_lane_map_time = None

        self._temporal_support = TemporalRouteSupportTracker()
        self._last_route_identity = None
        self._last_accepted_provenance = None
        self._last_reference_stamp_ns = None
        self._last_association_status = None
        self._active_map_session_id = None
        self._seen_map_session_ids = set()
        self._clock_reset_evidence_floor = None

        route_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._route_subscription = self.create_subscription(
            RoutePlan,
            "/planning/route_plan",
            self._route_callback,
            route_qos,
        )

        self._odom_subscription = self.create_subscription(
            Odometry,
            "/carla/hero_odom",
            self._odom_callback,
            qos_profile_sensor_data,
        )

        lane_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self._lane_subscription = self.create_subscription(
            LaneMap,
            "/perception/lane/local_map/vector",
            self._lane_map_callback,
            lane_qos,
        )

        self._association_publisher = self.create_publisher(
            RouteAssociationStatus,
            "/planning/route_association/status",
            10,
        )

        self._projection_timer = self.create_timer(
            self._report_period_s,
            self._report_projection,
        )

        self._temporal_timer = self.create_timer(
            self._report_period_s,
            self._report_temporal_snapshot,
        )

        self.get_logger().info(
            "Route ego diagnostic initialized. "
            "Waiting for RoutePlan and odometry."
        )

    def _route_callback(self, message: RoutePlan):
        """Validate and cache the active route geometry."""

        if message.header.frame_id != self._frame_id:
            self.get_logger().error(
                "RoutePlan frame mismatch: "
                f"received={message.header.frame_id}, "
                f"expected={self._frame_id}"
            )
            return

        path = message.coarse_reference_path

        if path.header.frame_id != self._frame_id:
            self.get_logger().error(
                "RoutePlan path frame mismatch: "
                f"received={path.header.frame_id}, "
                f"expected={self._frame_id}"
            )
            return

        try:
            incoming = RouteIdentity(
                mission_id=int(message.mission_id),
                route_id=int(message.route_id),
                map_revision=int(message.map_revision),
            )
        except (ValueError, TypeError, OverflowError) as exc:
            self.get_logger().warn(
                f"route_revision_consistency status=INVALID reason={exc}"
            )
            return

        previous = self._last_route_identity

        if (
            previous is not None
            and incoming.mission_id == previous.mission_id
            and incoming.map_revision == previous.map_revision
        ):
            if incoming.route_id < previous.route_id:
                self.get_logger().warn(
                    "route_revision_consistency "
                    "status=OUT_OF_ORDER_REVISION "
                    f"route_id={incoming.route_id} "
                    "action=IGNORE"
                )
                return

            if (
                incoming.route_id == previous.route_id
                and (
                    message.status != RoutePlan.STATUS_VALID
                    or self._active_route is None
                )
            ):
                self.get_logger().warn(
                    "route_revision_consistency "
                    "status=ROUTE_ID_STATUS_CONFLICT "
                    f"route_id={incoming.route_id} "
                    "action=IGNORE"
                )
                return

        if message.status != RoutePlan.STATUS_VALID:
            self._last_route_identity = incoming
            current = self._temporal_support.route_identity

            if (
                current is not None
                and int(message.mission_id) == current.mission_id
                and int(message.map_revision) == current.map_revision
                and int(message.route_id) < current.route_id
            ):
                self.get_logger().warn(
                    "route_revision_consistency "
                    "status=OUT_OF_ORDER_REVISION "
                    f"route_id={message.route_id} "
                    "action=IGNORE"
                )
                return

            self._active_route = None
            self._route_points = None
            self._temporal_support = TemporalRouteSupportTracker()
            self._last_association_status = None

            self.get_logger().warn(
                "RoutePlan not valid: "
                f"mission_id={message.mission_id}, "
                f"route_id={message.route_id}, "
                f"status={message.status} "
                "temporal_history=RESET"
            )
            return

        pose_count = len(path.poses)

        if pose_count < 2:
            self.get_logger().warn(
                "RoutePlan has insufficient geometry: "
                f"poses={pose_count}"
            )
            return

        for pose in path.poses:
            pose_frame = pose.header.frame_id

            if pose_frame and pose_frame != self._frame_id:
                self.get_logger().error(
                    "RoutePlan pose frame mismatch: "
                    f"received={pose_frame}, "
                    f"expected={self._frame_id}"
                )
                return

        points = [
            (
                float(pose.pose.position.x),
                float(pose.pose.position.y),
            )
            for pose in path.poses
        ]

        if not all(
            math.isfinite(x) and math.isfinite(y)
            for x, y in points
        ):
            self.get_logger().error(
                "Rejecting RoutePlan: non-finite coordinates"
            )
            return

        has_valid_segment = any(
            math.hypot(
                points[i + 1][0] - points[i][0],
                points[i + 1][1] - points[i][1],
            ) > 0.0
            for i in range(len(points) - 1)
        )

        if not has_valid_segment:
            self.get_logger().error(
                "Rejecting RoutePlan: degenerate geometry"
            )
            return

        try:
            identity = RouteIdentity(
                mission_id=int(message.mission_id),
                route_id=int(message.route_id),
                map_revision=int(message.map_revision),
            )

            consistency = self._temporal_support.register_route(
                identity,
                points,
            )
        except (ValueError, TypeError, OverflowError) as exc:
            self.get_logger().warn(
                f"route_revision_consistency status=INVALID reason={exc}"
            )
            return

        if not consistency.accept_current_route:
            self.get_logger().warn(
                "route_revision_consistency "
                f"status={consistency.status} "
                f"route_id={identity.route_id} "
                "action=IGNORE"
            )
            return

        self._route_points = points
        self._active_route = message
        if (
            self._last_association_status is not None
            and (
                self._last_association_status.mission_id != message.mission_id
                or self._last_association_status.route_id != message.route_id
                or self._last_association_status.route_map_revision != message.map_revision
            )
        ):
            self._last_association_status = None
        self._last_route_identity = identity

        self.get_logger().info(
            "route_revision_consistency "
            f"mission_id={identity.mission_id} "
            f"route_id={identity.route_id} "
            f"status={consistency.status} "
            f"history_action={consistency.history_action} "
            f"geometry_candidate="
            f"{str(consistency.geometry_correspondence_candidate).lower()}"
        )

        self.get_logger().info(
            "Accepted RoutePlan: "
            f"mission_id={message.mission_id}, "
            f"route_id={message.route_id}, "
            f"map_revision={message.map_revision}, "
            f"poses={pose_count}"
        )

    def _odom_callback(self, message: Odometry):
        """Cache valid ego odometry and its arrival time."""

        if message.header.frame_id != self._frame_id:
            self._latest_odom = None
            self._latest_odom_time = None

            self.get_logger().error(
                "Odometry frame mismatch: "
                f"received={message.header.frame_id}, "
                f"expected={self._frame_id}"
            )
            return

        position = message.pose.pose.position

        if not (
            math.isfinite(position.x)
            and math.isfinite(position.y)
        ):
            self._latest_odom = None
            self._latest_odom_time = None

            self.get_logger().error(
                "Rejecting odometry: non-finite ego coordinates"
            )
            return

        try:
            quaternion_to_yaw(message.pose.pose.orientation)
        except ValueError as exc:
            self._latest_odom = None
            self._latest_odom_time = None

            self.get_logger().error(
                f"Rejecting odometry: {exc}"
            )
            return

        self._latest_odom = message
        self._latest_odom_time = time.monotonic()

        if not self._odom_received:
            self.get_logger().info(
                "First valid odometry received: "
                f"x={position.x:.3f}, "
                f"y={position.y:.3f}"
            )

            self._odom_received = True


    def _lane_map_callback(self, message: LaneMap):
        """Cache the latest perception-derived LaneMap."""

        self._latest_lane_map = message
        self._latest_lane_map_time = time.monotonic()

    def _log_route_support(
        self,
        association,
        coverage,
        lane_map_revision,
        segment_id,
    ):
        try:
            result = evaluate_route_support(
                association,
                coverage,
            )
        except (ValueError, OverflowError) as exc:
            self.get_logger().warn(
                "route_support_decision "
                "status=INVALID "
                f"route_id={self._active_route.route_id} "
                f"segment_id={segment_id} "
                f"reason={exc}"
            )
            return None

        self.get_logger().info(
            "route_support_decision "
            f"route_id={self._active_route.route_id} "
            f"lane_map_revision={lane_map_revision} "
            f"segment_id={segment_id} "
            f"status={result.status} "
            f"reason={result.reason} "
            f"near_ego_interval_m="
            f"{result.near_ego_interval_m:.3f} "
            f"initial_gap_tolerated="
            f"{str(result.initial_gap_tolerated).lower()}"
        )
        return result


    def _publish_association_status(
        self, lane_map, provenance, clock, admission,
        selection, snapshot, metrics, effective_decision=None,
    ):
        """Publish the accepted association and temporal diagnostic."""

        identity = self._temporal_support.route_identity

        if identity is None:
            return

        message = RouteAssociationStatus()
        message.header = copy.deepcopy(lane_map.header)

        message.mission_id = int(identity.mission_id)
        message.route_id = int(identity.route_id)
        message.route_map_revision = int(identity.map_revision)

        message.map_session_id = provenance.session_id
        message.lane_map_revision = provenance.map_revision
        message.evidence_revision = provenance.evidence_revision

        message.selected_segment_valid = (
            selection.selected_segment_id is not None
        )

        if message.selected_segment_valid:
            message.selected_segment_id = int(
                selection.selected_segment_id
            )

        decision = effective_decision or selection.decision

        message.decision_status = decision.status
        message.decision_reason = decision.reason
        message.temporal_status = snapshot.status
        message.temporal_reason = snapshot.reason

        message.selection_reason = selection.selection_reason
        message.provenance_status = admission.status
        message.evidence_time_status = clock.status

        message.candidate_count = selection.candidate_count
        message.supported_candidate_count = (
            selection.supported_candidate_count
        )
        message.consecutive_supported = (
            snapshot.consecutive_supported
        )

        message.evidence_age_s = (
            float(clock.evidence_age_s)
            if clock.evidence_age_s is not None
            else float("nan")
        )
        message.temporal_observation_age_s = (
            float(snapshot.latest_observation_age_s)
            if snapshot.latest_observation_age_s is not None
            else float("nan")
        )

        message.median_abs_lateral_m = float("nan")
        message.median_heading_error_deg = float("nan")

        chosen = (metrics or {}).get(
            selection.selected_segment_id
        )

        if chosen is not None:
            comparison, forward, ego_s_m, window = chosen

            message.metrics_valid = True
            message.median_abs_lateral_m = float(
                comparison.median_abs_lateral_m
            )
            message.median_heading_error_deg = float(
                comparison.median_abs_heading_error_deg
            )

            message.coverage_ratio_ahead = float(
                forward.coverage_ratio_ahead
            )
            message.supported_ahead_m = float(
                forward.supported_ahead_m
            )
            message.continuous_from_ego_m = float(
                forward.continuous_from_ego_m
            )

            message.ego_s_m = float(ego_s_m)
            message.window_start_s_m = float(
                window.start_s_m
            )
            message.window_end_s_m = float(
                window.end_s_m
            )

            intervals = forward.supported_intervals_ahead

            message.supported_interval_start_s_m = [
                float(a) for a, _ in intervals
            ]
            message.supported_interval_end_s_m = [
                float(b) for _, b in intervals
            ]

        self._last_association_status = message
        self._association_publisher.publish(message)

    def _record_selected_support(
        self, lane_map, candidates, metrics=None
    ):
        """Admit one map update using evidence and clock provenance."""

        identity = self._temporal_support.route_identity

        if identity is None or self._latest_odom is None:
            return

        if int(lane_map.schema_version) != 3:
            self.get_logger().warn(
                "route_support_observation "
                "status=SKIPPED reason=UNSUPPORTED_LANE_MAP_SCHEMA"
            )
            return

        def stamp_ns(stamp):
            return (
                int(stamp.sec) * 1_000_000_000
                + int(stamp.nanosec)
            )

        try:
            provenance = LaneMapProvenance(
                session_id=int(lane_map.map_session_id),
                map_revision=int(lane_map.map_revision),
                evidence_revision=int(lane_map.evidence_revision),
                latest_evidence_stamp_ns=stamp_ns(
                    lane_map.latest_evidence_stamp
                ),
            )

            reference_ns = stamp_ns(
                self._latest_odom.header.stamp
            )

            if (
                self._active_map_session_id is not None
                and provenance.session_id != self._active_map_session_id
                and provenance.session_id in self._seen_map_session_ids
            ):
                self.get_logger().warn(
                    "route_support_observation "
                    "status=SKIPPED "
                    "reason=PREVIOUS_MAPPER_SESSION "
                    f"map_session_id={provenance.session_id}"
                )
                return

            clock = classify_evidence_time(
                evidence_stamp_ns=provenance.latest_evidence_stamp_ns,
                reference_stamp_ns=reference_ns,
                previous_reference_stamp_ns=(
                    self._last_reference_stamp_ns
                ),
            )

            if clock.reset_temporal_history:
                self._last_association_status = None
                previous_map = self._last_accepted_provenance

                if previous_map is not None:
                    self._clock_reset_evidence_floor = (
                        previous_map.session_id,
                        previous_map.evidence_revision,
                    )

                self._last_accepted_provenance = None
                self._temporal_support.reset_evidence_history()
                self._last_reference_stamp_ns = reference_ns

                self.get_logger().warn(
                    "route_support_observation "
                    "status=RESET "
                    f"reason={clock.status}"
                )
                return

            self._last_reference_stamp_ns = reference_ns

            if self._clock_reset_evidence_floor is not None:
                floor_session, floor_revision = (
                    self._clock_reset_evidence_floor
                )

                if (
                    provenance.session_id == floor_session
                    and (
                        provenance.evidence_revision <= floor_revision
                        or not clock.may_confirm_positive
                    )
                ):
                    self.get_logger().warn(
                        "route_support_observation "
                        "status=SKIPPED "
                        "reason=WAITING_FOR_POST_RESET_EVIDENCE"
                    )
                    return

            admission = classify_lane_map_provenance(
                self._last_accepted_provenance,
                provenance,
            )

            if not admission.accept_map:
                if admission.status != "DUPLICATE":
                    self.get_logger().warn(
                        "route_support_observation "
                        "status=SKIPPED "
                        f"reason={admission.status}"
                    )
                return

            if admission.reset_history:
                self._temporal_support.reset_evidence_history()

            self._last_accepted_provenance = provenance
            self._active_map_session_id = provenance.session_id
            self._seen_map_session_ids.add(provenance.session_id)
            self._clock_reset_evidence_floor = None

            selection = select_lane_map_support(candidates)
            decision = selection.decision

            if decision.status == "SUPPORTED":
                if (
                    admission.may_confirm_positive
                    and clock.may_confirm_positive
                ):
                    pass
                elif not clock.may_confirm_positive:
                    # Do not preserve positive confirmation when
                    # incoming evidence fails the sensor-clock gate.
                    decision = RouteSupportDecision(
                        status="UNAVAILABLE",
                        reason=clock.status,
                        near_ego_interval_m=0.0,
                        initial_gap_tolerated=False,
                    )
                else:
                    # Pruning-only or other map-only changes must
                    # not increase or refresh positive confirmation.
                    self.get_logger().info(
                        "route_support_observation "
                        "status=SKIPPED "
                        "reason=NO_NEW_POSITIVE_EVIDENCE "
                        f"map_revision={provenance.map_revision} "
                        f"evidence_revision={provenance.evidence_revision}"
                    )
                    self._publish_association_status(
                        lane_map, provenance, clock, admission,
                        selection,
                        self._temporal_support.snapshot(time.monotonic()),
                        metrics,
                    )
                    return

            elif not admission.may_apply_negative:
                return

            snapshot = self._temporal_support.observe(
                identity,
                decision,
                observed_at_s=time.monotonic(),
                lane_map_revision=provenance.map_revision,
            )
            self._publish_association_status(
                lane_map, provenance, clock, admission,
                selection, snapshot, metrics, decision,
            )

        except (ValueError, TypeError, OverflowError) as exc:
            self.get_logger().warn(
                "route_support_observation "
                f"status=INVALID reason={exc}"
            )
            return

        selected_id = (
            "none"
            if selection.selected_segment_id is None
            else str(selection.selected_segment_id)
        )

        self.get_logger().info(
            "lane_map_route_support "
            f"mission_id={identity.mission_id} "
            f"route_id={identity.route_id} "
            f"map_session_id={provenance.session_id} "
            f"lane_map_revision={provenance.map_revision} "
            f"evidence_revision={provenance.evidence_revision} "
            f"provenance_status={admission.status} "
            f"evidence_time_status={clock.status} "
            f"selected_segment_id={selected_id} "
            f"decision_status={decision.status} "
            f"temporal_status={snapshot.status} "
            f"consecutive_supported={snapshot.consecutive_supported}"
        )

    def _report_temporal_snapshot(self):
        now = time.monotonic()
        snapshot = self._temporal_support.snapshot(now)
        identity = snapshot.route_identity

        if identity is None:
            return
        cached = self._last_association_status

        if (
            cached is not None
            and cached.mission_id == identity.mission_id
            and cached.route_id == identity.route_id
            and cached.route_map_revision == identity.map_revision
        ):
            status = copy.deepcopy(cached)
        else:
            status = RouteAssociationStatus()
            status.mission_id = int(identity.mission_id)
            status.route_id = int(identity.route_id)
            status.route_map_revision = int(identity.map_revision)
            status.decision_status = "UNAVAILABLE"
            status.decision_reason = "NO_EVALUATED_LANE_MAP"
            status.evidence_age_s = float("nan")

        status.header.frame_id = self._frame_id

        if self._latest_odom is not None:
            status.header.stamp = copy.deepcopy(
                self._latest_odom.header.stamp
            )

        status.temporal_status = snapshot.status
        status.temporal_reason = snapshot.reason
        status.consecutive_supported = snapshot.consecutive_supported

        status.temporal_observation_age_s = (
            float(snapshot.latest_observation_age_s)
            if snapshot.latest_observation_age_s is not None
            else float("nan")
        )

        if snapshot.status in (
            "WAITING",
            "STALE",
            "UNAVAILABLE",
        ):
            status.metrics_valid = False
            status.selected_segment_valid = False
            status.supported_interval_start_s_m = []
            status.supported_interval_end_s_m = []

        self._association_publisher.publish(status)

        age = (
            "none"
            if snapshot.latest_observation_age_s is None
            else f"{snapshot.latest_observation_age_s:.3f}"
        )

        lane_revision = (
            "none"
            if snapshot.last_lane_map_revision is None
            else str(snapshot.last_lane_map_revision)
        )

        self.get_logger().info(
            "route_support_temporal "
            f"mission_id={identity.mission_id} "
            f"route_id={identity.route_id} "
            f"status={snapshot.status} "
            f"reason={snapshot.reason} "
            f"consecutive_supported={snapshot.consecutive_supported} "
            f"observations={snapshot.observations_in_revision} "
            f"lane_map_revision={lane_revision} "
            f"observation_age_s={age} "
            f"freshness_weight={snapshot.freshness_weight:.3f}"
        )

    def _report_lane_comparison(
        self,
        window,
        ego_s_m,
        distance_gate,
        heading_gate,
    ):
        """Compare fresh perceived lane geometry against the route."""

        if not distance_gate or not heading_gate:
            self.get_logger().warn(
                "lane_route_comparison "
                "status=SKIPPED "
                "reason=EGO_ROUTE_GATE_FAILED"
            )
            return

        if self._latest_lane_map is None:
            self.get_logger().warn(
                "lane_route_comparison "
                "status=UNAVAILABLE "
                "reason=NO_LANE_MAP"
            )
            return

        lane_map = self._latest_lane_map

        if lane_map.header.frame_id != self._frame_id:
            self.get_logger().warn(
                "lane_route_comparison "
                "status=UNAVAILABLE "
                "reason=FRAME_MISMATCH"
            )
            return

        if (
            self._latest_lane_map_time is None
            or time.monotonic() - self._latest_lane_map_time
            > self._max_lane_map_age_s
        ):
            self.get_logger().warn(
                "lane_route_comparison "
                "status=UNAVAILABLE "
                "reason=STALE_LANE_MAP"
            )
            return

        def stamp_seconds(stamp):
            return (
                float(stamp.sec)
                + float(stamp.nanosec) * 1e-9
            )

        lane_stamp = stamp_seconds(lane_map.header.stamp)
        odom_stamp = stamp_seconds(self._latest_odom.header.stamp)

        if lane_stamp <= 0.0 or odom_stamp <= 0.0:
            self.get_logger().warn(
                "lane_route_comparison "
                "status=UNAVAILABLE "
                "reason=INVALID_TIMESTAMP"
            )
            return

        timestamp_delta_s = lane_stamp - odom_stamp

        if abs(timestamp_delta_s) > self._max_lane_odom_delta_s:
            self.get_logger().warn(
                "lane_route_comparison "
                "status=UNAVAILABLE "
                "reason=TIMESTAMP_MISMATCH "
                f"delta_s={timestamp_delta_s:.3f}"
            )
            return

        if not lane_map.segments:
            self.get_logger().info(
                "lane_route_comparison "
                "status=UNAVAILABLE "
                "reason=EMPTY_LANE_MAP "
                f"lane_map_revision={lane_map.map_revision}"
            )

            decision = evaluate_lane_route_association(None)

            self.get_logger().info(
                "lane_route_association "
                f"route_id={self._active_route.route_id} "
                f"status={decision.status} "
                f"reason={decision.reason} "
                "source=EMPTY_LANE_MAP"
            )

            self._log_route_support(
                decision,
                None,
                lane_map.map_revision,
                "none",
            )
            self._record_selected_support(lane_map, [])
            return

        compared_segments = 0
        support_candidates = []
        support_metrics = {}

        for segment in lane_map.segments:
            if not segment.drivable or len(segment.centerline) < 2:
                continue

            try:
                lane_points = [
                    LanePoint(
                        x_m=float(sample.pose.position.x),
                        y_m=float(sample.pose.position.y),
                        heading_rad=quaternion_to_yaw(
                            sample.pose.orientation
                        ),
                        confidence=float(sample.confidence),
                    )
                    for sample in segment.centerline
                ]

                result = compare_lane_to_route(
                    window,
                    lane_points,
                )

            except (ValueError, OverflowError) as exc:
                self.get_logger().warn(
                    "lane_route_comparison "
                    "status=INVALID_SEGMENT "
                    f"segment_id={segment.id} "
                    f"reason={exc}"
                )
                continue

            if result is None:
                continue

            compared_segments += 1

            decision = evaluate_lane_route_association(result)

            self.get_logger().info(
                "lane_route_comparison "
                "status=MEASURED "
                f"route_id={self._active_route.route_id} "
                f"lane_map_revision={lane_map.map_revision} "
                f"segment_id={segment.id} "
                f"samples={result.sample_count} "
                f"median_distance_m="
                f"{result.median_distance_m:.3f} "
                f"median_abs_lateral_m="
                f"{result.median_abs_lateral_m:.3f} "
                f"median_heading_error_deg="
                f"{result.median_abs_heading_error_deg:.2f} "
                f"median_confidence="
                f"{result.median_confidence:.3f} "
                f"projected_s_start_m="
                f"{result.min_route_s_m:.3f} "
                f"projected_s_end_m="
                f"{result.max_route_s_m:.3f} "
                f"projected_s_span_m="
                f"{result.projected_s_span_m:.3f} "
                f"lane_odom_delta_s={timestamp_delta_s:.3f}"
            )

            self.get_logger().info(
                "lane_route_association "
                f"route_id={self._active_route.route_id} "
                f"lane_map_revision={lane_map.map_revision} "
                f"segment_id={segment.id} "
                f"status={decision.status} "
                f"reason={decision.reason}"
            )

            try:
                coverage = estimate_lane_route_coverage(
                    window,
                    lane_points,
                )

                forward_coverage = summarize_forward_coverage(
                    coverage,
                    window,
                    ego_s_m,
                )
            except (ValueError, OverflowError) as exc:
                self.get_logger().warn(
                    "lane_route_coverage "
                    "status=INVALID "
                    f"segment_id={segment.id} "
                    f"reason={exc}"
                )
                continue

            self.get_logger().info(
                "lane_route_coverage "
                "status=MEASURED "
                f"route_id={self._active_route.route_id} "
                f"lane_map_revision={lane_map.map_revision} "
                f"segment_id={segment.id} "
                f"window_length_m="
                f"{coverage.window_length_m:.3f} "
                f"supported_length_m="
                f"{coverage.supported_length_m:.3f} "
                f"coverage_ratio="
                f"{coverage.coverage_ratio:.3f} "
                f"longest_contiguous_m="
                f"{coverage.longest_contiguous_m:.3f} "
                f"valid_samples="
                f"{coverage.valid_sample_count} "
                f"connected_pairs="
                f"{coverage.connected_pair_count} "
                f"interval_count="
                f"{len(coverage.supported_intervals)}"
            )

            first_gap = (
                "none"
                if forward_coverage.first_support_gap_m is None
                else f"{forward_coverage.first_support_gap_m:.3f}"
            )

            self.get_logger().info(
                "lane_route_forward_coverage "
                "status=MEASURED "
                f"route_id={self._active_route.route_id} "
                f"lane_map_revision={lane_map.map_revision} "
                f"segment_id={segment.id} "
                f"ego_s_m={ego_s_m:.3f} "
                f"forward_window_length_m="
                f"{forward_coverage.forward_window_length_m:.3f} "
                f"supported_ahead_m="
                f"{forward_coverage.supported_ahead_m:.3f} "
                f"coverage_ratio_ahead="
                f"{forward_coverage.coverage_ratio_ahead:.3f} "
                f"longest_contiguous_ahead_m="
                f"{forward_coverage.longest_contiguous_ahead_m:.3f} "
                f"continuous_from_ego_m="
                f"{forward_coverage.continuous_from_ego_m:.3f} "
                f"first_support_gap_m={first_gap} "
                f"interval_count_ahead="
                f"{len(forward_coverage.supported_intervals_ahead)}"
            )

            support_decision = self._log_route_support(
                decision,
                forward_coverage,
                lane_map.map_revision,
                segment.id,
            )

            if support_decision is not None:
                support_metrics[int(segment.id)] = (
                    result, forward_coverage, ego_s_m, window,
                )
                support_candidates.append(
                    SegmentSupportCandidate(
                        segment_id=int(segment.id),
                        decision=support_decision,
                    )
                )

        self._record_selected_support(
            lane_map,
            support_candidates,
            support_metrics,
        )

        if compared_segments == 0:
            self.get_logger().warn(
                "lane_route_comparison "
                "status=UNAVAILABLE "
                "reason=NO_USABLE_SEGMENTS"
            )

    def _report_projection(self):
        """Report ego route coordinates and heading consistency."""

        if (
            self._active_route is None
            or self._route_points is None
            or self._latest_odom is None
        ):
            return

        if (
            self._latest_odom_time is None
            or time.monotonic() - self._latest_odom_time
            > self._max_odom_age_s
        ):
            self.get_logger().warn(
                "Ego projection unavailable: stale odometry"
            )
            return

        position = self._latest_odom.pose.pose.position
        orientation = self._latest_odom.pose.pose.orientation

        try:
            projection = project_point_to_polyline(
                self._route_points,
                float(position.x),
                float(position.y),
            )

            ego_yaw = quaternion_to_yaw(orientation)

        except (ValueError, OverflowError) as exc:
            self.get_logger().error(
                f"Route projection failed: {exc}"
            )
            return

        values = (
            projection.s_m,
            projection.d_m,
            projection.distance_m,
            projection.heading_rad,
            ego_yaw,
        )

        if not all(math.isfinite(v) for v in values):
            self.get_logger().error(
                "Route projection failed: non-finite result"
            )
            return

        heading_error_rad = normalize_angle(
            ego_yaw - projection.heading_rad
        )

        heading_error_deg = math.degrees(
            heading_error_rad
        )

        ego_yaw_deg = math.degrees(ego_yaw)
        route_heading_deg = math.degrees(
            projection.heading_rad
        )

        within_distance_gate = (
            projection.distance_m
            <= self._max_projection_distance_m
        )

        within_heading_gate = (
            abs(heading_error_deg)
            <= self._max_heading_error_deg
        )

        try:
            window = extract_route_window(
                self._route_points,
                center_s_m=projection.s_m,
                backward_m=self._window_backward_m,
                forward_m=self._window_forward_m,
            )
        except (ValueError, OverflowError) as exc:
            self.get_logger().error(
                f"Local route window failed: {exc}"
            )
            return

        window_usable = (
            window.end_s_m > window.start_s_m
            and len(window.samples) >= 2
        )

        self.get_logger().info(
            "ego_route_projection "
            f"route_id={self._active_route.route_id} "
            f"s_m={projection.s_m:.3f} "
            f"d_m={projection.d_m:.3f} "
            f"distance_m={projection.distance_m:.3f} "
            f"segment={projection.segment_index} "
            f"ego_yaw_deg={ego_yaw_deg:.2f} "
            f"route_heading_deg={route_heading_deg:.2f} "
            f"heading_error_deg={heading_error_deg:.2f} "
            f"distance_gate="
            f"{'PASS' if within_distance_gate else 'FAIL'} "
            f"heading_gate="
            f"{'PASS' if within_heading_gate else 'FAIL'} "
            f"window_start_s={window.start_s_m:.3f} "
            f"window_end_s={window.end_s_m:.3f} "
            f"window_samples={len(window.samples)} "
            f"window_geometry="
            f"{'USABLE' if window_usable else 'UNUSABLE'}"
        )

        if window_usable:
            self._report_lane_comparison(
                window,
                projection.s_m,
                within_distance_gate,
                within_heading_gate,
            )


def main(args=None):
    rclpy.init(args=args)

    node = RouteEgoDiagnosticNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
