#!/usr/bin/env python3
"""Apply the final parking-lot profile to a selected run."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

import clustering_common as common
from radar_cluster_merge import RadarClusterMergeParams
from rosbag_reader import read_frames


DATA = common.BUNDLE / "data/parking_lot"
# max_y_m은 최종 영상과 같은 전방 ROI 끝거리다.
RUNS = {
    "away20": {
        "bag": "camera_100546",
        "start_s": 0.0,
        "end_s": 15.5,
        "max_y_m": 125.0,
    },
    "static50": {
        "bag": "camera_095549",
        "start_s": 0.0,
        "end_s": 10.2,
        "max_y_m": 125.0,
    },
    "vehicle150": {
        "bag": "camera_103644",
        "start_s": 82.0,
        "end_s": 92.0,
        "max_y_m": 270.0,
    },
    "vehicle170": {
        "bag": "camera_103644",
        "start_s": 58.0,
        "end_s": 68.0,
        "max_y_m": 270.0,
    },
}

PROFILE = common.Profile(
    fifo_frames=2,  # 현재 프레임과 직전 1프레임을 함께 군집화한다.
    density={
        "hdbscan_min_cluster_size": 10,  # 군집 하나를 인정할 최소 누적 점 수다.
        "hdbscan_min_samples": 5,  # 외부 hdbscan에는 자기 자신 제외 4로 변환된다.
        "hdbscan_cluster_selection_epsilon": 2.0,  # 2.0 m 이내의 가까운 분할을 줄인다.
    },
    power_weight=0.25,  # 거리별 Power 점수 1 IQR을 공간 0.25 m처럼 반영한다.
    doppler_weight=0.40,  # Doppler 1 m/s를 공간 0.40 m처럼 반영한다.
    merge=RadarClusterMergeParams(
        base_point_gap_m=2.0,  # 가까운 군집을 합칠 기본 점 간격이다.
        azimuth_resolution_deg=4.0,  # 거리별 허용 간격 계산에 쓰는 방위 해상도다.
        max_point_gap_m=10.0,  # 병합할 두 군집의 최대 점 간격이다.
        max_range_gap_m=15.0,  # 병합할 두 군집의 최대 거리 차이다.
        max_azimuth_gap_deg=10.0,  # 병합할 두 군집의 최대 방위각 차이다.
        doppler_min_abs_mps=0.15,  # 이 속도 이상만 Doppler 병합 판단에 쓴다.
        doppler_max_abs_mps=10.0,  # 이 속도 이하만 Doppler 병합 판단에 쓴다.
        max_doppler_gap_mps=2.5,  # 병합할 두 군집의 최대 Doppler 차이다.
    ),
)


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", nargs="?", choices=tuple(RUNS), default="away20")
    parser.add_argument("--bag", type=Path)
    parser.add_argument("--start-s", type=float)
    parser.add_argument("--end-s", type=float)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--self-check", action="store_true")
    return parser.parse_args()


def keep(max_y_m: float):
    def select(frame: dict) -> np.ndarray:
        return (
            np.isfinite(frame["x"] + frame["y"] + frame["z"] + frame["power"] + frame["doppler"])
            & (frame["x"] >= -60.0)
            & (frame["x"] <= 40.0)
            & (frame["y"] >= 0.0)
            & (frame["y"] <= max_y_m)
            & (np.abs(frame["doppler"]) < 40.0)
        )

    return select


def main() -> int:
    args = arguments()
    if args.self_check:
        assert PROFILE.fifo_frames == 2 and PROFILE.power_weight == 0.25
        return common.self_check()
    run = RUNS[args.run]
    bag = args.bag or DATA / str(run["bag"])
    start_s = float(run["start_s"]) if args.start_s is None else args.start_s
    end_s = float(run["end_s"]) if args.end_s is None else args.end_s
    output_start_s = max(0.0, start_s - 1.0 / 40.0)  # 20 Hz에서 시작시각에 가장 가까운 프레임을 포함한다.
    warmup_s = max(0.0, output_start_s - (PROFILE.fifo_frames - 1) / 20.0)
    output = args.output or common.BUNDLE / f"outputs/generated/{args.run}_clustered_points.csv"
    results = common.cluster_frames(
        read_frames(bag, warmup_s, end_s),
        keep(float(run["max_y_m"])),
        PROFILE,
    )
    common.write_points_csv(
        output,
        (result for result in results if result["relative_s"] >= output_start_s),
    )
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
