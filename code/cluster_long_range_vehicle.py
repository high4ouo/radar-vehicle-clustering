#!/usr/bin/env python3
"""Apply the final 8/26 long-range vehicle profile."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

import clustering_common as common
from rosbag_reader import read_frames


DATA = common.BUNDLE / "data/long_range"
# track은 군집화 뒤 차량 군집을 고를 때만 쓰는 확인 궤적 이름이다.
RUNS = {
    "parking_175m": {
        "bag": "camera_200530",
        "track": "parking",
        "start_s": 98.0,
        "end_s": 134.9,
    },
    "road_261m": {
        "bag": "camera_201136",
        "track": "road",
        "start_s": 26.5,
        "end_s": 205.0,
    },
}
VEHICLE_TRACKS = {
    "parking": {
        "max_range_m": 190.0,  # 이 거리 밖의 점은 차량 후보에서 제외한다.
        # 각 기준점은 (bag 시간[s], 차량 x[m], 차량 y[m])이다.
        "anchors": (
            (98.0, 5.35, 174.40), (100.5, 5.25, 169.58),
            (105.0, 4.92, 158.45), (107.0, 4.76, 151.46),
            (109.0, 4.42, 140.35), (115.0, 3.41, 102.85),
            (120.5, 2.26, 63.92), (125.0, 1.65, 34.73),
            (130.5, 2.08, 20.75), (134.5, 1.56, 12.38),
        ),
    },
    "road": {
        "max_range_m": 325.0,  # 이 거리 밖의 점은 차량 후보에서 제외한다.
        # 각 기준점은 (bag 시간[s], 차량 x[m], 차량 y[m])이다.
        "anchors": (
            (26.49, 1.76, 50.02), (34.99, 1.50, 63.96),
            (44.99, 2.85, 83.39), (54.99, 6.48, 99.19),
            (65.49, 6.75, 115.21), (79.99, 6.31, 141.70),
            (97.99, 4.86, 175.15), (100.99, 8.70, 179.16),
            (115.98, 6.29, 182.06), (121.98, 6.91, 182.02),
            (124.98, 7.92, 177.82), (130.98, 5.39, 169.55),
            (136.98, 7.55, 169.48), (142.98, 8.11, 181.99),
            (149.98, 8.01, 197.29), (159.98, 9.69, 216.72),
            (169.98, 13.53, 240.15), (177.48, 14.23, 261.04),
            (182.98, 12.24, 273.64), (186.98, 13.92, 277.78),
            (204.48, 15.53, 309.72),
        ),
    },
}

PROFILE = common.Profile(
    fifo_frames=7,  # 현재 프레임과 직전 6프레임을 함께 군집화한다.
    density={
        "hdbscan_min_cluster_size": 3,  # 군집 하나를 인정할 최소 누적 점 수다.
        "hdbscan_min_samples": 2,  # 외부 hdbscan에는 자기 자신 제외 1로 변환된다.
        "hdbscan_cluster_selection_epsilon": 2.5,  # 2.5 m 이내의 가까운 분할을 줄인다.
    },
)

MOVING_DOPPLER_MIN_MPS = 0.15  # 정지 clutter를 버리는 최소 절대 방사속도다.
VALID_DOPPLER_MAX_MPS = 40.0  # 비정상 Doppler 값을 버리는 최대 절대값이다.
TARGET_ASSOCIATION_GATE_M = 8.0  # 확인 궤적과 차량 군집 중심의 최대 허용 거리다.


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", choices=tuple(RUNS))
    parser.add_argument("--bag", type=Path)
    parser.add_argument("--start-s", type=float)
    parser.add_argument("--end-s", type=float)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--self-check", action="store_true")
    return parser.parse_args()


def keep(max_range_m: float):
    def select(frame: dict) -> np.ndarray:
        horizontal_range = np.hypot(frame["x"], frame["y"])
        return (
            np.isfinite(frame["x"] + frame["y"] + frame["z"] + frame["power"] + frame["doppler"])
            & (frame["y"] >= 0.0)
            & (horizontal_range <= max_range_m)
            & (np.abs(frame["doppler"]) >= MOVING_DOPPLER_MIN_MPS)
            & (np.abs(frame["doppler"]) < VALID_DOPPLER_MAX_MPS)
        )

    return select


def expected_vehicle_xy(track: dict, relative_s: float) -> np.ndarray:
    anchors = np.asarray(track["anchors"], dtype=float)
    return np.array(
        [
            np.interp(relative_s, anchors[:, 0], anchors[:, 1]),
            np.interp(relative_s, anchors[:, 0], anchors[:, 2]),
        ]
    )


def target_selector(track: dict):
    def select(result: dict) -> int:
        expected = expected_vehicle_xy(track, float(result["relative_s"]))
        candidates = []
        for label in sorted(set(result["labels"].tolist()) - {-1}):
            center = np.median(result["xyz"][result["labels"] == label, :2], axis=0)
            candidates.append((float(np.linalg.norm(center - expected)), int(label)))
        return (
            min(candidates)[1]
            if candidates and min(candidates)[0] <= TARGET_ASSOCIATION_GATE_M
            else -1
        )

    return select


def main() -> int:
    args = arguments()
    if args.self_check:
        assert PROFILE.fifo_frames == 7
        assert PROFILE.density["hdbscan_min_cluster_size"] == 3
        return common.self_check()
    run = RUNS[args.run]
    track = VEHICLE_TRACKS[str(run["track"])]
    bag = args.bag or DATA / str(run["bag"])
    start_s = float(run["start_s"]) if args.start_s is None else args.start_s
    end_s = float(run["end_s"]) if args.end_s is None else args.end_s
    output = args.output or common.BUNDLE / f"outputs/generated/{args.run}_clustered_points.csv"
    results = common.cluster_frames(
        read_frames(bag, start_s, end_s),
        keep(float(track["max_range_m"])),
        PROFILE,
    )
    common.write_points_csv(output, results, target_selector(track))
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
