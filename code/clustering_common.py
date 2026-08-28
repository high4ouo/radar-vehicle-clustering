#!/usr/bin/env python3
"""Shared HDBSCAN runner used by the final vehicle profiles."""

from __future__ import annotations

import csv
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable, Iterator

import numpy as np

import radar_clustering_core as core
from radar_cluster_merge import RadarClusterMergeParams


BUNDLE = Path(__file__).resolve().parents[1]


@dataclass(frozen=True)
class Profile:
    fifo_frames: int
    density: dict[str, float | int]
    backend: str  # "sklearn" 또는 외부 hdbscan 패키지를 뜻하는 "contrib"다.
    power_weight: float = 0.0
    doppler_weight: float = 0.0
    merge: RadarClusterMergeParams | None = None


def _features(
    xyz: np.ndarray,
    power: np.ndarray,
    doppler: np.ndarray,
    ranges: np.ndarray,
    profile: Profile,
) -> np.ndarray:
    if not profile.power_weight and not profile.doppler_weight:
        return xyz
    # Power는 거리 구간별 IQR로 정규화한 뒤 XYZ 옆의 추가 축으로 붙인다.
    power_score = (
        core.range_conditioned_power_score(power, ranges, 10.0, 3.0)
        if profile.power_weight
        else None
    )
    doppler_score = np.clip(doppler, -10.0, 10.0) if profile.doppler_weight else None
    return core.build_features(
        xyz,
        power_feature=power_score,
        doppler_feature=doppler_score,
        power_weight=profile.power_weight,
        doppler_weight=profile.doppler_weight,
    )


def cluster_frames(
    frames: Iterable[dict],
    keep_points: Callable[[dict], np.ndarray],
    profile: Profile,
) -> Iterator[dict]:
    if profile.fifo_frames < 1:
        raise ValueError("fifo_frames must be positive")
    history: deque[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]] = deque(
        maxlen=profile.fifo_frames
    )
    for frame in frames:
        keep = keep_points(frame)
        xyz = np.column_stack((frame["x"][keep], frame["y"][keep], frame["z"][keep]))
        power = np.asarray(frame["power"][keep], dtype=float)
        doppler = np.asarray(frame["doppler"][keep], dtype=float)
        frame_ranges = frame.get("range")
        ranges = (
            np.linalg.norm(xyz, axis=1)
            if frame_ranges is None
            else np.asarray(frame_ranges[keep], dtype=float)
        )
        history.append((xyz, power, doppler, _features(xyz, power, doppler, ranges, profile)))

        # FIFO 안의 점으로 군집을 정하되 출력에는 현재 프레임 점만 남긴다.
        stacked_xyz = np.concatenate([item[0] for item in history])
        stacked_power = np.concatenate([item[1] for item in history])
        stacked_doppler = np.concatenate([item[2] for item in history])
        stacked_features = np.concatenate([item[3] for item in history])
        method = "hdbscan_merged" if profile.merge is not None else "hdbscan"
        labels, diagnostics, _ = core.cluster_frame(
            stacked_xyz,
            stacked_features,
            method,
            power=stacked_power,
            doppler=stacked_doppler,
            hdbscan_backend=profile.backend,
            merge_params=profile.merge,
            **profile.density,
        )
        current_labels = labels[-len(xyz) :] if len(xyz) else np.empty(0, dtype=int)
        yield {
            "frame_index": int(frame["frame_index"]),
            "relative_s": float(frame["relative_s"]),
            "xyz": xyz,
            "power": power,
            "doppler": doppler,
            "labels": current_labels,
            "runtime_ms": float(
                diagnostics["_cluster_runtime_ms"] + diagnostics["_merge_runtime_ms"]
            ),
        }


def write_points_csv(
    output: Path,
    results: Iterable[dict],
    target_selector: Callable[[dict], int] | None = None,
) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + ".tmp")
    fields = (
        "frame_index",
        "relative_s",
        "x_m",
        "y_m",
        "z_m",
        "power",
        "doppler_mps",
        "cluster_label",
        "target_cluster",
        "runtime_ms",
    )
    with temporary.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for result in results:
            target = target_selector(result) if target_selector else -1
            for point, power, doppler, label in zip(
                result["xyz"], result["power"], result["doppler"], result["labels"]
            ):
                writer.writerow(
                    {
                        "frame_index": result["frame_index"],
                        "relative_s": f"{result['relative_s']:.6f}",
                        "x_m": f"{point[0]:.6f}",
                        "y_m": f"{point[1]:.6f}",
                        "z_m": f"{point[2]:.6f}",
                        "power": f"{power:.6f}",
                        "doppler_mps": f"{doppler:.6f}",
                        "cluster_label": int(label),
                        "target_cluster": int(label >= 0 and label == target),
                        "runtime_ms": f"{result['runtime_ms']:.6f}",
                    }
                )
    temporary.replace(output)


def self_check() -> int:
    first = np.array([[0.0, 10.0, 0.0], [0.1, 10.1, 0.0], [0.2, 10.0, 0.0]])
    frames = [
        {
            "frame_index": index,
            "relative_s": index * 0.05,
            "x": xyz[:, 0],
            "y": xyz[:, 1],
            "z": xyz[:, 2],
            "power": np.full(3, 1000.0),
            "doppler": np.zeros(3),
        }
        for index, xyz in enumerate((first, first + [0.0, 0.2, 0.0]))
    ]
    profile = Profile(
        fifo_frames=2,
        density={
            "hdbscan_min_cluster_size": 2,  # 합성 검사용 최소 군집 크기다.
            "hdbscan_min_samples_library": 1,  # 합성 검사용 밀도 이웃 수다.
            "hdbscan_cluster_selection_epsilon": 0.5,  # 합성 검사용 병합 거리다.
        },
        backend="sklearn",  # 차량 영상과 같은 sklearn HDBSCAN 경로를 검사한다.
    )
    results = list(cluster_frames(frames, lambda frame: np.ones(3, dtype=bool), profile))
    assert len(results) == 2 and all(len(result["labels"]) == 3 for result in results)
    print("self-check PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(self_check())
