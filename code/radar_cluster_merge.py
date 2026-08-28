#!/usr/bin/env python3
"""Conservative radar-aware post-processing for fragmented flat clusters.

This module does not create a second clustering algorithm.  It joins pairs of
already predicted clusters only when their closest top-view points, median
range, median azimuth, and (when available) median radial Doppler are mutually
compatible.  The range-dependent point-gap allowance uses the documented
RETINA azimuth resolution instead of a dataset-specific object label.

Power is reported but is deliberately not a hard merge gate: the available
``power`` field has not yet been calibrated as RCS and changes with scene and
range.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np


@dataclass(frozen=True)
class RadarClusterMergeParams:
    """Physical gates used by :func:`merge_radar_clusters`."""

    base_point_gap_m: float = 0.75
    azimuth_resolution_deg: float = 2.0
    max_point_gap_m: float = 4.0
    max_range_gap_m: float = 6.0
    max_azimuth_gap_deg: float = 4.0
    doppler_min_abs_mps: float = 0.15
    doppler_max_abs_mps: float = 5.0
    max_doppler_gap_mps: float = 0.75


def _circular_angle_gap_deg(first: float, second: float) -> float:
    return abs((first - second + 180.0) % 360.0 - 180.0)


def _finite_median(values: np.ndarray) -> float:
    finite = np.asarray(values, dtype=float)
    finite = finite[np.isfinite(finite)]
    return float(np.median(finite)) if len(finite) else float("nan")


def _valid_doppler_median(
    values: np.ndarray, params: RadarClusterMergeParams
) -> float:
    values = np.asarray(values, dtype=float)
    valid = (
        np.isfinite(values)
        & (np.abs(values) >= params.doppler_min_abs_mps)
        & (np.abs(values) <= params.doppler_max_abs_mps)
    )
    return float(np.median(values[valid])) if np.any(valid) else float("nan")


def _minimum_xy_gap(first: np.ndarray, second: np.ndarray) -> float:
    delta = first[:, None, :2] - second[None, :, :2]
    return float(np.sqrt(np.min(np.sum(delta * delta, axis=2))))


def merge_radar_clusters(
    xyz: np.ndarray,
    labels: np.ndarray,
    doppler: np.ndarray,
    power: np.ndarray,
    params: RadarClusterMergeParams | None = None,
) -> tuple[np.ndarray, dict[str, Any]]:
    """Merge compatible predicted clusters and return deterministic labels.

    Noise (label ``-1``) is never promoted.  The pairwise merge decisions are
    computed from the original clusters, then joined transitively with
    union-find.  Consequently a long structure can be reassembled from several
    locally compatible fragments, but a chain can also over-merge nearby
    objects; simulator GT metrics must therefore be checked after this step.
    """

    params = params or RadarClusterMergeParams()
    xyz = np.asarray(xyz, dtype=float)
    labels = np.asarray(labels, dtype=int)
    doppler = np.asarray(doppler, dtype=float)
    power = np.asarray(power, dtype=float)
    cluster_ids = sorted(set(labels) - {-1})
    diagnostics: dict[str, Any] = {
        "clusters_before_merge": len(cluster_ids),
        "clusters_after_merge": len(cluster_ids),
        "merge_edges": 0,
        "merge_pairs": [],
    }
    if len(cluster_ids) < 2:
        return labels.copy(), diagnostics

    stats: dict[int, dict[str, Any]] = {}
    for cluster_id in cluster_ids:
        mask = labels == cluster_id
        points = xyz[mask]
        ranges = np.linalg.norm(points, axis=1)
        azimuths = np.degrees(np.arctan2(points[:, 0], points[:, 1]))
        stats[cluster_id] = {
            "points": points,
            "range_m": float(np.median(ranges)),
            "azimuth_deg": float(np.median(azimuths)),
            "doppler_mps": _valid_doppler_median(doppler[mask], params),
            "power": _finite_median(power[mask]),
        }

    parent = {cluster_id: cluster_id for cluster_id in cluster_ids}

    def find(cluster_id: int) -> int:
        while parent[cluster_id] != cluster_id:
            parent[cluster_id] = parent[parent[cluster_id]]
            cluster_id = parent[cluster_id]
        return cluster_id

    def union(first: int, second: int) -> None:
        root_first, root_second = find(first), find(second)
        if root_first != root_second:
            parent[max(root_first, root_second)] = min(root_first, root_second)

    accepted_pairs: list[dict[str, float | int | None]] = []
    for first_index, first_id in enumerate(cluster_ids):
        first = stats[first_id]
        for second_id in cluster_ids[first_index + 1 :]:
            second = stats[second_id]
            range_gap = abs(first["range_m"] - second["range_m"])
            if range_gap > params.max_range_gap_m:
                continue
            azimuth_gap = _circular_angle_gap_deg(
                first["azimuth_deg"], second["azimuth_deg"]
            )
            if azimuth_gap > params.max_azimuth_gap_deg:
                continue
            point_gap = _minimum_xy_gap(first["points"], second["points"])
            nearer_range = min(first["range_m"], second["range_m"])
            allowed_gap = min(
                params.max_point_gap_m,
                params.base_point_gap_m
                + nearer_range
                * math.tan(math.radians(params.azimuth_resolution_deg)),
            )
            if point_gap > allowed_gap:
                continue
            doppler_first = first["doppler_mps"]
            doppler_second = second["doppler_mps"]
            doppler_gap: float | None = None
            if np.isfinite(doppler_first) and np.isfinite(doppler_second):
                doppler_gap = abs(doppler_first - doppler_second)
                if doppler_gap > params.max_doppler_gap_mps:
                    continue
            union(first_id, second_id)
            accepted_pairs.append(
                {
                    "first": first_id,
                    "second": second_id,
                    "point_gap_m": point_gap,
                    "allowed_point_gap_m": allowed_gap,
                    "range_gap_m": range_gap,
                    "azimuth_gap_deg": azimuth_gap,
                    "doppler_gap_mps": doppler_gap,
                }
            )

    roots = sorted({find(cluster_id) for cluster_id in cluster_ids})
    root_to_label = {root: index for index, root in enumerate(roots)}
    merged = np.full(len(labels), -1, dtype=int)
    for cluster_id in cluster_ids:
        merged[labels == cluster_id] = root_to_label[find(cluster_id)]
    diagnostics.update(
        {
            "clusters_after_merge": len(roots),
            "merge_edges": len(accepted_pairs),
            "merge_pairs": accepted_pairs,
        }
    )
    return merged, diagnostics
