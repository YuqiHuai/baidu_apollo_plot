from pathlib import Path
from apollo_modules.modules.map.proto.map_pb2 import Map
from matplotlib import pyplot as plt
from cyber_record.record import Record


def plot_scenario(record_path: str, map_bin: str, out_dir: str):
    assert Path(record_path).exists(), "Record file does not exist!"
    assert Path(map_bin).exists(), "HD Map does not exist!"
    Path(out_dir).mkdir(parents=True, exist_ok=True)

    map = Map()
    with open(map_bin, "rb") as fp:
        map.ParseFromString(fp.read())

    record = Record(record_path)

    for lane in map.lane:
        left_boundary = [
            (p.x, p.y) for p in lane.left_boundary.curve.segment[0].line_segment.point
        ]
        right_boundary = [
            (p.x, p.y) for p in lane.right_boundary.curve.segment[0].line_segment.point
        ]
        plt.plot(
            [p[0] for p in left_boundary],
            [p[1] for p in left_boundary],
            color="black",
            alpha=0.5,
        )
        plt.plot(
            [p[0] for p in right_boundary],
            [p[1] for p in right_boundary],
            color="black",
            alpha=0.5,
        )

    for _, msg, _ in record.read_messages(["/apollo/localization/pose"]):
        ego_x = msg.pose.position.x
        ego_y = msg.pose.position.y
        plt.plot(ego_x, ego_y, marker="o", markersize=5, color="blue")
        break

    plt.xlim(ego_x - 100, ego_x + 100)
    plt.ylim(ego_y - 100, ego_y + 100)

    for _, msg, _ in record.read_messages(["/apollo/perception/obstacles"]):
        for obstacle in msg.perception_obstacle:
            print(obstacle)
            polygon_points = obstacle.polygon_point
            polygon_points.append(polygon_points[0])
            plt.fill(
                [p.x for p in polygon_points],
                [p.y for p in polygon_points],
                color="red",
            )
        break

    plt.show()
