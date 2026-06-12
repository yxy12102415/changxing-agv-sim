import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Quaternion
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def resample_polyline(points, count):
    if len(points) == count:
        return points
    if len(points) < 2:
        return points * count

    lengths = [0.0]
    for i in range(1, len(points)):
        lengths.append(lengths[-1] + distance(points[i - 1], points[i]))
    total = lengths[-1]
    if total <= 1e-6:
        return [points[0]] * count

    result = []
    for i in range(count):
        target = total * i / max(1, count - 1)
        seg = 1
        while seg < len(lengths) and lengths[seg] < target:
            seg += 1
        if seg >= len(lengths):
            result.append(points[-1])
            continue
        span = max(1e-6, lengths[seg] - lengths[seg - 1])
        ratio = (target - lengths[seg - 1]) / span
        x = points[seg - 1][0] + ratio * (points[seg][0] - points[seg - 1][0])
        y = points[seg - 1][1] + ratio * (points[seg][1] - points[seg - 1][1])
        z = points[seg - 1][2] + ratio * (points[seg][2] - points[seg - 1][2])
        result.append((x, y, z))
    return result


def resample_by_spacing(points, spacing):
    if len(points) < 2:
        return points

    result = [points[0]]
    carry = 0.0
    prev = points[0]
    for current in points[1:]:
        seg_len = distance(prev, current)
        if seg_len <= 1e-6:
            prev = current
            continue
        direction = (
            (current[0] - prev[0]) / seg_len,
            (current[1] - prev[1]) / seg_len,
            (current[2] - prev[2]) / seg_len,
        )
        traveled = spacing - carry
        while traveled <= seg_len:
            result.append(
                (
                    prev[0] + direction[0] * traveled,
                    prev[1] + direction[1] * traveled,
                    prev[2] + direction[2] * traveled,
                )
            )
            traveled += spacing
        carry = seg_len - (traveled - spacing)
        prev = current
    if distance(result[-1], points[-1]) > spacing * 0.5:
        result.append(points[-1])
    return result


def interpolate_gap(start, end, spacing):
    gap = distance(start, end)
    if gap <= spacing * 1.5:
        return []

    direction = (
        (end[0] - start[0]) / gap,
        (end[1] - start[1]) / gap,
        (end[2] - start[2]) / gap,
    )
    result = []
    traveled = spacing
    while traveled < gap - spacing * 0.5:
        result.append(
            (
                start[0] + direction[0] * traveled,
                start[1] + direction[1] * traveled,
                start[2] + direction[2] * traveled,
            )
        )
        traveled += spacing
    return result


def smooth_path_points(points, passes):
    smoothed = list(points)
    for _ in range(max(0, int(passes))):
        if len(smoothed) < 3:
            break
        next_points = smoothed[:]
        for i in range(1, len(smoothed) - 1):
            prev_point = smoothed[i - 1]
            point = smoothed[i]
            next_point = smoothed[i + 1]
            next_points[i] = (
                0.25 * prev_point[0] + 0.5 * point[0] + 0.25 * next_point[0],
                0.25 * prev_point[1] + 0.5 * point[1] + 0.25 * next_point[1],
                0.25 * prev_point[2] + 0.5 * point[2] + 0.25 * next_point[2],
            )
        smoothed = next_points
    return smoothed


def cumulative_lengths(points):
    lengths = [0.0]
    for i in range(1, len(points)):
        lengths.append(lengths[-1] + distance(points[i - 1], points[i]))
    return lengths


def sample_by_s(points, lengths, target_s, wrap=True):
    if not points:
        raise RuntimeError("Cannot sample an empty path")
    if len(points) == 1:
        return points[0]

    total = lengths[-1]
    if wrap and total > 1e-6:
        target_s = target_s % total
    else:
        target_s = max(0.0, min(total, target_s))

    seg = 1
    while seg < len(lengths) and lengths[seg] < target_s:
        seg += 1
    if seg >= len(lengths):
        return points[-1]

    span = max(1e-6, lengths[seg] - lengths[seg - 1])
    ratio = (target_s - lengths[seg - 1]) / span
    return (
        points[seg - 1][0] + ratio * (points[seg][0] - points[seg - 1][0]),
        points[seg - 1][1] + ratio * (points[seg][1] - points[seg - 1][1]),
        points[seg - 1][2] + ratio * (points[seg][2] - points[seg - 1][2]),
    )


def slice_ring(points, lengths, start_s, end_s, spacing):
    total = lengths[-1]
    if total <= 1e-6:
        return list(points)
    if end_s <= start_s:
        end_s += total

    result = []
    current_s = start_s
    while current_s < end_s:
        result.append(sample_by_s(points, lengths, current_s, wrap=True))
        current_s += spacing
    result.append(sample_by_s(points, lengths, end_s, wrap=True))
    return result


def lane_change_curve(start, end, spacing):
    gap = distance(start, end)
    steps = max(2, int(math.ceil(gap / max(0.1, spacing))))
    result = []
    for i in range(steps + 1):
        ratio = i / steps
        smooth = ratio * ratio * (3.0 - 2.0 * ratio)
        result.append(
            (
                start[0] + (end[0] - start[0]) * smooth,
                start[1] + (end[1] - start[1]) * smooth,
                start[2] + (end[2] - start[2]) * smooth,
            )
        )
    return result


class CenterlineTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__("centerline_trajectory_publisher")
        self.declare_parameter("map_path", "")
        self.declare_parameter("output_topic", "/planning/scenario_planning/trajectory")
        self.declare_parameter("marker_topic", "/visualization/centerline_trajectory_marker")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("target_speed", 4.0)
        self.declare_parameter("min_speed", 0.5)
        self.declare_parameter("max_lateral_accel", 0.35)
        self.declare_parameter("curvature_window", 6)
        self.declare_parameter("speed_smoothing_passes", 3)
        self.declare_parameter("path_smoothing_passes", 3)
        self.declare_parameter("point_spacing", 0.5)
        self.declare_parameter("route_mode", "dual_lane_change")
        self.declare_parameter("lane_change_length", 16.0)
        self.declare_parameter("inner_to_outer_inner_s", 84.0)
        self.declare_parameter("inner_to_outer_outer_s", 84.0)
        self.declare_parameter("outer_to_inner_outer_s", 218.0)
        self.declare_parameter("outer_to_inner_inner_s", 212.0)
        self.declare_parameter("initial_x", 0.0674)
        self.declare_parameter("initial_y", -57.6716)

        map_path = self.get_parameter("map_path").get_parameter_value().string_value
        if not map_path:
            map_path = os.path.join(
                get_package_share_directory("agv_maps"), "map", "changxing_v1.osm"
            )
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.target_speed = self.get_parameter("target_speed").get_parameter_value().double_value
        self.min_speed = self.get_parameter("min_speed").get_parameter_value().double_value
        self.max_lateral_accel = (
            self.get_parameter("max_lateral_accel").get_parameter_value().double_value
        )
        self.curvature_window = (
            self.get_parameter("curvature_window").get_parameter_value().integer_value
        )
        self.speed_smoothing_passes = (
            self.get_parameter("speed_smoothing_passes").get_parameter_value().integer_value
        )
        self.path_smoothing_passes = (
            self.get_parameter("path_smoothing_passes").get_parameter_value().integer_value
        )
        self.point_spacing = self.get_parameter("point_spacing").get_parameter_value().double_value
        self.route_mode = self.get_parameter("route_mode").get_parameter_value().string_value
        self.lane_change_length = (
            self.get_parameter("lane_change_length").get_parameter_value().double_value
        )
        self.inner_to_outer_inner_s = (
            self.get_parameter("inner_to_outer_inner_s").get_parameter_value().double_value
        )
        self.inner_to_outer_outer_s = (
            self.get_parameter("inner_to_outer_outer_s").get_parameter_value().double_value
        )
        self.outer_to_inner_outer_s = (
            self.get_parameter("outer_to_inner_outer_s").get_parameter_value().double_value
        )
        self.outer_to_inner_inner_s = (
            self.get_parameter("outer_to_inner_inner_s").get_parameter_value().double_value
        )
        self.initial_xy = (
            self.get_parameter("initial_x").get_parameter_value().double_value,
            self.get_parameter("initial_y").get_parameter_value().double_value,
        )

        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        marker_topic = self.get_parameter("marker_topic").get_parameter_value().string_value
        self.publisher = self.create_publisher(Trajectory, output_topic, 1)
        self.marker_publisher = self.create_publisher(MarkerArray, marker_topic, 1)
        self.trajectory_points = self.build_centerline(map_path)
        self.trajectory_speeds = self.build_speed_profile(self.trajectory_points)

        publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / max(0.1, publish_rate), self.on_timer)
        self.get_logger().info(
            f"Publishing {len(self.trajectory_points)} centerline trajectory points from {map_path}; "
            f"speed range={min(self.trajectory_speeds):.2f}-{max(self.trajectory_speeds):.2f} m/s"
        )

    def build_centerline(self, map_path):
        root = ET.parse(map_path).getroot()
        nodes = {}
        ways = {}
        for node in root.findall("node"):
            tags = {tag.attrib["k"]: tag.attrib["v"] for tag in node.findall("tag")}
            if "local_x" in tags and "local_y" in tags:
                nodes[node.attrib["id"]] = (
                    float(tags["local_x"]),
                    float(tags["local_y"]),
                    float(tags.get("ele", "0.0")),
                )
        for way in root.findall("way"):
            ways[way.attrib["id"]] = [nd.attrib["ref"] for nd in way.findall("nd")]

        lanelets = []
        for relation in root.findall("relation"):
            tags = {tag.attrib["k"]: tag.attrib["v"] for tag in relation.findall("tag")}
            if tags.get("type") != "lanelet" or tags.get("subtype") != "road":
                continue
            members = {
                member.attrib.get("role"): member.attrib.get("ref")
                for member in relation.findall("member")
                if member.attrib.get("type") == "way"
            }
            left_id = members.get("left")
            right_id = members.get("right")
            if left_id not in ways or right_id not in ways:
                continue
            left = [nodes[node_id] for node_id in ways[left_id] if node_id in nodes]
            right = [nodes[node_id] for node_id in ways[right_id] if node_id in nodes]
            count = max(len(left), len(right), 2)
            left = resample_polyline(left, count)
            right = resample_polyline(right, count)
            center = [
                (
                    (left[i][0] + right[i][0]) * 0.5,
                    (left[i][1] + right[i][1]) * 0.5,
                    (left[i][2] + right[i][2]) * 0.5,
                )
                for i in range(count)
            ]
            lanelets.append({"id": relation.attrib["id"], "points": center})

        if not lanelets:
            raise RuntimeError(f"No road lanelets found in {map_path}")

        inner_ordered = self.build_lanelet_loop(lanelets)
        inner_ids = {lanelet["id"] for lanelet in inner_ordered}
        if self.route_mode == "dual_lane_change":
            outer_lanelets = [lanelet for lanelet in lanelets if lanelet["id"] not in inner_ids]
            if outer_lanelets:
                outer_start_index = min(
                    range(len(outer_lanelets)),
                    key=lambda i: distance(inner_ordered[0]["points"][0], outer_lanelets[i]["points"][0]),
                )
                outer_ordered = self.build_lanelet_loop(outer_lanelets, outer_start_index)
                return self.build_dual_lane_change_route(inner_ordered, outer_ordered)

        return self.build_single_loop_route(inner_ordered)

    def build_lanelet_loop(self, lanelets, start_index=None):
        remaining = list(lanelets)
        if start_index is None:
            start_index = min(
                range(len(remaining)), key=lambda i: distance(remaining[i]["points"][0], self.initial_xy)
            )
        ordered = [remaining.pop(start_index)]
        route_start = ordered[0]["points"][0]
        while remaining:
            tail = ordered[-1]["points"][-1]
            if distance(tail, route_start) < 1.0:
                break
            next_index = min(
                range(len(remaining)), key=lambda i: distance(tail, remaining[i]["points"][0])
            )
            if distance(tail, remaining[next_index]["points"][0]) > 5.0:
                break
            ordered.append(remaining.pop(next_index))
        return ordered

    def lanelets_to_points(self, ordered):
        raw_points = []
        for lanelet in ordered:
            if raw_points and distance(raw_points[-1], lanelet["points"][0]) < 0.1:
                raw_points.extend(lanelet["points"][1:])
            else:
                raw_points.extend(lanelet["points"])
        return raw_points

    def build_single_loop_route(self, ordered):
        spacing = max(0.1, self.point_spacing)
        raw_points = self.lanelets_to_points(ordered)
        points = resample_by_spacing(raw_points, spacing)
        nearest = min(range(len(points)), key=lambda i: distance(points[i], self.initial_xy))
        rotated = list(points[nearest:])
        if nearest > 0:
            rotated.extend(interpolate_gap(points[-1], points[0], spacing))
            rotated.extend(points[:nearest])
        return smooth_path_points(rotated, self.path_smoothing_passes)

    def build_dual_lane_change_route(self, inner_ordered, outer_ordered):
        spacing = max(0.1, self.point_spacing)
        inner = resample_by_spacing(self.lanelets_to_points(inner_ordered), spacing)
        outer = resample_by_spacing(self.lanelets_to_points(outer_ordered), spacing)
        inner_lengths = cumulative_lengths(inner)
        outer_lengths = cumulative_lengths(outer)

        start_s = min(
            inner_lengths,
            key=lambda s: distance(sample_by_s(inner, inner_lengths, s), self.initial_xy),
        )
        change_length = max(4.0, self.lane_change_length)

        route = []
        route.extend(slice_ring(inner, inner_lengths, start_s, self.inner_to_outer_inner_s, spacing))
        route.extend(
            lane_change_curve(
                sample_by_s(inner, inner_lengths, self.inner_to_outer_inner_s),
                sample_by_s(
                    outer, outer_lengths, self.inner_to_outer_outer_s + change_length
                ),
                spacing,
            )[1:]
        )
        route.extend(
            slice_ring(
                outer,
                outer_lengths,
                self.inner_to_outer_outer_s + change_length,
                self.outer_to_inner_outer_s,
                spacing,
            )[1:]
        )
        route.extend(
            lane_change_curve(
                sample_by_s(outer, outer_lengths, self.outer_to_inner_outer_s),
                sample_by_s(
                    inner, inner_lengths, self.outer_to_inner_inner_s + change_length
                ),
                spacing,
            )[1:]
        )
        route.extend(
            slice_ring(
                inner,
                inner_lengths,
                self.outer_to_inner_inner_s + change_length,
                start_s + inner_lengths[-1],
                spacing,
            )[1:]
        )
        return smooth_path_points(route, self.path_smoothing_passes)

    def build_speed_profile(self, points):
        if len(points) < 3:
            return [self.target_speed for _ in points]

        min_speed = max(0.1, min(self.min_speed, self.target_speed))
        max_speed = max(min_speed, self.target_speed)
        window = max(2, int(self.curvature_window))
        speeds = []

        for i, point in enumerate(points):
            prev_point = points[max(0, i - window)]
            next_point = points[min(len(points) - 1, i + window)]
            if prev_point == point or next_point == point:
                speeds.append(max_speed)
                continue

            yaw_prev = math.atan2(point[1] - prev_point[1], point[0] - prev_point[0])
            yaw_next = math.atan2(next_point[1] - point[1], next_point[0] - point[0])
            heading_change = abs(math.atan2(math.sin(yaw_next - yaw_prev), math.cos(yaw_next - yaw_prev)))
            arc_length = max(1e-3, distance(prev_point, point) + distance(point, next_point))
            curvature = heading_change / arc_length

            if curvature < 1e-4:
                speed = max_speed
            else:
                speed = math.sqrt(max(0.1, self.max_lateral_accel) / curvature)
            speeds.append(max(min_speed, min(max_speed, speed)))

        for _ in range(max(0, int(self.speed_smoothing_passes))):
            smoothed = speeds[:]
            for i in range(1, len(speeds) - 1):
                smoothed[i] = 0.25 * speeds[i - 1] + 0.5 * speeds[i] + 0.25 * speeds[i + 1]
            speeds = [max(min_speed, min(max_speed, speed)) for speed in smoothed]

        return speeds

    def on_timer(self):
        msg = Trajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        elapsed = 0.0
        for i, point in enumerate(self.trajectory_points):
            target_speed = self.trajectory_speeds[i]
            next_point = self.trajectory_points[min(i + 1, len(self.trajectory_points) - 1)]
            prev_point = self.trajectory_points[max(i - 1, 0)]
            yaw = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0])
            if i > 0:
                elapsed += distance(self.trajectory_points[i - 1], point) / max(
                    0.1, self.trajectory_speeds[max(0, i - 1)]
                )

            traj_point = TrajectoryPoint()
            traj_point.time_from_start = Duration(
                sec=int(elapsed), nanosec=int((elapsed - int(elapsed)) * 1e9)
            )
            traj_point.pose.position.x = point[0]
            traj_point.pose.position.y = point[1]
            traj_point.pose.position.z = point[2]
            traj_point.pose.orientation = yaw_to_quaternion(yaw)
            traj_point.longitudinal_velocity_mps = float(target_speed)
            traj_point.acceleration_mps2 = 0.0
            traj_point.heading_rate_rps = 0.0
            msg.points.append(traj_point)

        self.publisher.publish(msg)
        self.publish_marker(msg.header.stamp)

    def publish_marker(self, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = "mpc_centerline"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.18
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.lifetime.sec = 0

        for point in self.trajectory_points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2] + 0.15
            marker.points.append(p)

        markers = MarkerArray()
        markers.markers.append(marker)
        self.marker_publisher.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = CenterlineTrajectoryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
