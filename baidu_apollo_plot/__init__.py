from pathlib import Path
from apollo_modules.modules.map.proto.map_pb2 import Map
from apollo_modules.modules.perception.proto.traffic_light_detection_pb2 import TrafficLightDetection
from matplotlib import pyplot as plt
from cyber_record.record import Record
from shapely.geometry import Polygon
import matplotlib.patches as patches
import numpy as np
import math
import matplotlib.animation as animation
from matplotlib.animation import PillowWriter
from apollo.map_service import MapService

APOLLO_VEHICLE_LENGTH = 4.933
APOLLO_VEHICLE_WIDTH = 2.11
APOLLO_VEHICLE_back_edge_to_center = 1.043

def generate_adc_polygon(position, theta):
    points = []
    half_w = APOLLO_VEHICLE_WIDTH / 2.0
    front_l = APOLLO_VEHICLE_LENGTH - APOLLO_VEHICLE_back_edge_to_center
    back_l = -1 * APOLLO_VEHICLE_back_edge_to_center
    sin_h = math.sin(theta)
    cos_h = math.cos(theta)
    vectors = [(front_l * cos_h - half_w * sin_h,
                front_l * sin_h + half_w * cos_h),
               (back_l * cos_h - half_w * sin_h,
                back_l * sin_h + half_w * cos_h),
               (back_l * cos_h + half_w * sin_h,
                back_l * sin_h - half_w * cos_h),
               (front_l * cos_h + half_w * sin_h,
                front_l * sin_h - half_w * cos_h)]

    points = np.array([[position.x + x, position.y + y] for x, y in vectors])

    return points


def plot_scenario(record_path: str, map_bin: str, out_dir: str):
    assert Path(record_path).exists(), "Record file does not exist!"
    assert Path(map_bin).exists(), "HD Map does not exist!"
    Path(out_dir).mkdir(parents=True, exist_ok=True)

    map_service = MapService()
    map_service.load_map_from_file(map_bin)

    map = Map()
    # print(dir(map))
    with open(map_bin, "rb") as fp:
        map.ParseFromString(fp.read())

    record = Record(record_path)

    fig, ax = plt.subplots(figsize=(8, 8))

    for lane in map.lane:
        left_boundary = [
            (p.x, p.y) for p in lane.left_boundary.curve.segment[0].line_segment.point
        ]
        right_boundary = [
            (p.x, p.y) for p in lane.right_boundary.curve.segment[0].line_segment.point
        ]
        ax.plot(
            [p[0] for p in left_boundary],
            [p[1] for p in left_boundary],
            color="black",
            alpha=0.5,
        )
        ax.plot(
            [p[0] for p in right_boundary],
            [p[1] for p in right_boundary],
            color="black",
            alpha=0.5,
        )

    ego_positions = []
    obstacle_positions = []

    for _, msg, _ in record.read_messages(["/apollo/localization/pose"]):
        ego_positions.append((msg.pose.position.x, msg.pose.position.y, msg.pose.heading))

    for _, msg, _ in record.read_messages(["/apollo/perception/obstacles"]):
        frame_obstacles = []
        for obstacle in msg.perception_obstacle:
            # print(obstacle)
            obs_type = obstacle.type

            polygon_points = [(p.x, p.y) for p in obstacle.polygon_point]
            if polygon_points:
                polygon_points.append(polygon_points[0])
                frame_obstacles.append((polygon_points, obs_type))

        obstacle_positions.append(frame_obstacles)

    crosswalks, stop_lines = [], []
    signal_map = {}

    for crosswalk in map.crosswalk:
        points = [(p.x, p.y) for p in crosswalk.polygon.point]
        crosswalks.append(points)

    for signal in map.signal:
        signal_map[signal.id.id] = {
            "stop_line": [],
            "color": "red" 
        }
        for segment in signal.stop_line:
            points = [(p.x, p.y) for p in segment.segment[0].line_segment.point]
            signal_map[signal.id.id]["stop_line"].append(points)

    traffic_lights = {}

    for _, msg, _ in record.read_messages(["/apollo/perception/traffic_light"]):
        for light in msg.traffic_light:
            traffic_lights[light.id] = light.color

    num_frames = min(len(ego_positions), len(obstacle_positions))
    interval = 1000 / 24

    def update(frame):
        ax.clear()

        for lane in map.lane:
            left_boundary = [
                (p.x, p.y) for p in lane.left_boundary.curve.segment[0].line_segment.point
            ]
            right_boundary = [
                (p.x, p.y) for p in lane.right_boundary.curve.segment[0].line_segment.point
            ]
            ax.plot(
                [p[0] for p in left_boundary],
                [p[1] for p in left_boundary],
                color="black",
                alpha=0.5,
            )
            ax.plot(
                [p[0] for p in right_boundary],
                [p[1] for p in right_boundary],
                color="black",
                alpha=0.5,
            )

        if frame >= len(ego_positions) or frame >= len(obstacle_positions):
            return

        ego_x, ego_y, ego_theta = ego_positions[frame]
        ego_pts = generate_adc_polygon(type('Point', (object,), {"x": ego_x, "y": ego_y}), ego_theta)
        ego_pts = np.vstack([ego_pts, ego_pts[0]])
        ax.fill(ego_pts[:, 0], ego_pts[:, 1], color="blue")

        obstacle_colors = {
            3: "orange", # Pedestrian
            4: "purple", # Bicycle
            5: "red" # Vehicle
        }

        for obstacle, obs_type in obstacle_positions[frame]:
            if obs_type in obstacle_colors:
                ax.fill([p[0] for p in obstacle], [p[1] for p in obstacle], color=obstacle_colors[obs_type])
            else:
                print(f"Warning: Unknown obstacle type {obs_type}, skipping")

        # Dynamically modify the Stop Line color
        for signal_id, signal_data in signal_map.items():
            color = "red"
            if signal_id in traffic_lights:
                if traffic_lights[signal_id] == 1:
                    color = "red"
                elif traffic_lights[signal_id] == 2:
                    color = "yellow"
                elif traffic_lights[signal_id] == 3:
                    color = "green"

            for stop_line in signal_data["stop_line"]:
                ax.plot([p[0] for p in stop_line], [p[1] for p in stop_line], color=color, linewidth=2)

        for crosswalk in crosswalks:
            ax.fill([p[0] for p in crosswalk], [p[1] for p in crosswalk], color="lightgray", alpha=0.5)
        ax.set_xlim(ego_x - 100, ego_x + 100)
        ax.set_ylim(ego_y - 100, ego_y + 100)
        ax.set_title(f"Frame {frame + 1}/{num_frames}")

    ani = animation.FuncAnimation(fig, update, frames=num_frames, interval=interval)

    output_gif = Path(out_dir) / "scenario_video.gif"
    ani.save(output_gif, writer=PillowWriter(fps=24))
    print(f"GIF saved at {output_gif}")
    plt.show()
