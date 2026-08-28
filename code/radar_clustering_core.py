#!/usr/bin/env python3
"""Sensor-agnostic single-frame radar clustering core.

The caller owns coordinate conversion, ROI selection, Power/Doppler
normalization, and GT evaluation. This module only clusters one prepared
frame.
"""

from __future__ import annotations

from dataclasses import dataclass
from importlib.metadata import PackageNotFoundError, version
import time
import warnings
from typing import Any

import numpy as np
from scipy.spatial import cKDTree

# 최종 차량 영상은 sklearn 구현을 사용했다. 외부 구현은 backend 비교용이다.
from sklearn.cluster import HDBSCAN as SklearnHDBSCAN, OPTICS
from sklearn.mixture import GaussianMixture

try:
    import hdbscan as contrib_hdbscan
except Exception as exc:  # optional compiled backend; sklearn must still work
    contrib_hdbscan = None
    _CONTRIB_IMPORT_ERROR = exc
else:
    _CONTRIB_IMPORT_ERROR = None

from radar_cluster_merge import RadarClusterMergeParams, merge_radar_clusters

# 공개 실행 파일은 아래 cluster_frame()의 HDBSCAN 경로를 호출한다.

MAIN_VARIANTS = (
    ("hdbscan_merged", "HDBSCAN + merge"),
    ("optics_merged", "OPTICS + merge"),
    ("retina_dbscan_merged", "range-adaptive RETINA-DBSCAN + merge"),
)
ORACLE_VARIANT = ("gmm", "GMM GT-count oracle; no merge")
FINAL_VARIANTS = MAIN_VARIANTS + (ORACLE_VARIANT,)


@dataclass(frozen=True)
class RetinaDBSCANParams:
    """Range-adaptive spatial density settings; units are metres."""

    eps_base_m: float
    eps_slope: float
    eps_knee_m: float
    eps_max_m: float
    min_samples_near: int
    min_samples_far: int
    nmin_transition_m: float


def eps_of_range(ranges: np.ndarray, params: RetinaDBSCANParams) -> np.ndarray:
    grown = params.eps_base_m + params.eps_slope * np.clip(
        np.asarray(ranges, dtype=float) - params.eps_knee_m, 0.0, None
    )
    return np.clip(grown, params.eps_base_m, params.eps_max_m)


def min_samples_of_range(
    ranges: np.ndarray, params: RetinaDBSCANParams
) -> np.ndarray:
    return np.where(
        np.asarray(ranges) >= params.nmin_transition_m,
        params.min_samples_far,
        params.min_samples_near,
    ).astype(int)


def build_features(
    xyz: np.ndarray,
    *,
    power_feature: np.ndarray | None = None,
    doppler_feature: np.ndarray | None = None,
    power_weight: float = 1.0,
    doppler_weight: float = 1.0,
) -> np.ndarray:
    """Append already-normalized sensor features to XYZ."""

    xyz = np.asarray(xyz, dtype=float)
    if xyz.ndim != 2 or xyz.shape[1] != 3:
        raise ValueError(f"xyz must be N×3, got {xyz.shape}")
    columns = [xyz]
    for name, values, weight in (
        ("doppler_feature", doppler_feature, doppler_weight),
        ("power_feature", power_feature, power_weight),
    ):
        if values is None:
            continue
        values = np.asarray(values, dtype=float)
        if values.shape != (len(xyz),):
            raise ValueError(f"{name} must have {len(xyz)} values")
        if not np.isfinite(weight) or weight < 0:
            raise ValueError(f"{name} weight must be finite and non-negative")
        columns.append((float(weight) * values)[:, None])
    return np.column_stack(columns)


def relabel_stable(labels: np.ndarray, xyz: np.ndarray) -> np.ndarray:
    """Give clusters deterministic left-to-right/near-to-far IDs."""

    labels = np.asarray(labels, dtype=int)
    ordered = sorted(
        set(labels) - {-1},
        key=lambda label: tuple(np.round(np.mean(xyz[labels == label], axis=0), 6)),
    )
    mapping = {old: new for new, old in enumerate(ordered)}
    return np.asarray([mapping.get(int(label), -1) for label in labels], dtype=int)


def hdbscan_library_min_samples(backend: str, project_min_samples: int) -> int:
    """Translate the project's self-inclusive neighbor count per backend."""

    if project_min_samples < 1:
        raise ValueError("hdbscan_min_samples must be positive")
    if backend == "sklearn":
        return int(project_min_samples)
    if backend == "contrib":
        return max(1, int(project_min_samples) - 1)
    raise ValueError(f"unknown HDBSCAN backend: {backend}")


def _package_version(package: str) -> str:
    try:
        return version(package)
    except PackageNotFoundError:
        return "not-installed"


def retina_dbscan_labels(
    xyz: np.ndarray,
    features: np.ndarray,
    params: RetinaDBSCANParams | None = None,
) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    """Sparse exact DBSCAN expansion with symmetric range-adaptive eps."""

    if params is None:
        raise ValueError("retina_params must be fitted for the current sensor")
    xyz = np.asarray(xyz, dtype=float)
    features = np.asarray(features, dtype=float)
    if len(xyz) != len(features):
        raise ValueError("xyz and features must contain the same points")
    if not len(xyz):
        return np.empty(0, dtype=int), {
            "core": np.empty(0, dtype=bool),
            "eps": np.empty(0),
            "min_samples": np.empty(0, dtype=int),
        }

    ranges = np.linalg.norm(xyz, axis=1)
    eps = eps_of_range(ranges, params)
    min_samples = min_samples_of_range(ranges, params)
    candidates = cKDTree(features).query_ball_point(features, r=float(eps.max()))
    neighbours: list[np.ndarray] = []
    for point, candidate_ids in enumerate(candidates):
        ids = np.asarray(candidate_ids, dtype=int)
        delta = features[ids] - features[point]
        distance_sq = np.einsum("ij,ij->i", delta, delta)
        neighbours.append(ids[distance_sq <= eps[point] * eps[ids]])

    core = np.asarray([len(ids) for ids in neighbours]) >= min_samples
    labels = np.full(len(xyz), -1, dtype=int)
    visited = np.zeros(len(xyz), dtype=bool)
    cluster_id = 0
    for seed in range(len(xyz)):
        if visited[seed]:
            continue
        visited[seed] = True
        if not core[seed]:
            continue
        labels[seed] = cluster_id
        queue = [int(value) for value in neighbours[seed]]
        queued = set(queue)
        for point in queue:
            if not visited[point]:
                visited[point] = True
                if core[point]:
                    for neighbour in neighbours[point]:
                        value = int(neighbour)
                        if value not in queued:
                            queue.append(value)
                            queued.add(value)
            if labels[point] == -1:
                labels[point] = cluster_id
        cluster_id += 1
    return relabel_stable(labels, xyz), {
        "core": core,
        "eps": eps,
        "min_samples": min_samples,
    }


def cluster_frame(
    xyz: np.ndarray,
    features: np.ndarray,
    method: str,
    *,
    doppler: np.ndarray | None = None,
    power: np.ndarray | None = None,
    retina_params: RetinaDBSCANParams | None = None,
    hdbscan_min_cluster_size: int = 4,
    hdbscan_min_samples: int = 3,
    hdbscan_min_samples_library: int | None = None,
    hdbscan_backend: str = "contrib",
    hdbscan_cluster_selection_epsilon: float = 0.0,
    optics_min_samples: int = 4,
    optics_max_eps: float = 5.0,
    optics_xi: float = 0.05,
    gmm_components: int | None = None,
    merge: bool | None = None,
    merge_params: RadarClusterMergeParams | None = None,
) -> tuple[np.ndarray, dict[str, Any], str]:
    """Cluster one frame.  GT is neither accepted nor inspected."""

    xyz = np.asarray(xyz, dtype=float)
    features = np.asarray(features, dtype=float)
    if len(xyz) != len(features):
        raise ValueError("xyz and features must contain the same points")
    requested_merge = method.endswith("_merged") if merge is None else merge
    base_method = method.removesuffix("_merged")
    empty = {"core": np.zeros(len(xyz), dtype=bool)}
    started_ns = time.perf_counter_ns()

    if base_method == "hdbscan":
        # 두 HDBSCAN 패키지는 min_samples 정의가 달라 선택한 패키지를 먼저 확정한다.
        if hdbscan_backend not in {"sklearn", "contrib"}:
            raise ValueError(f"unknown HDBSCAN backend: {hdbscan_backend}")
        if hdbscan_min_samples_library is None:
            library_min_samples = hdbscan_library_min_samples(
                hdbscan_backend, hdbscan_min_samples
            )
            min_samples_source = "legacy_project_conversion"
        else:
            library_min_samples = int(hdbscan_min_samples_library)
            if library_min_samples < 1:
                raise ValueError("hdbscan_min_samples_library must be positive")
            min_samples_source = "native_library_value"
        epsilon = float(hdbscan_cluster_selection_epsilon)
        if not np.isfinite(epsilon) or epsilon < 0:
            raise ValueError("hdbscan_cluster_selection_epsilon must be finite and non-negative")
        backend_diagnostics = {
            "hdbscan_backend": hdbscan_backend,
            "hdbscan_package": (
                "scikit-learn" if hdbscan_backend == "sklearn" else "hdbscan"
            ),
            "hdbscan_package_version": _package_version(
                "scikit-learn" if hdbscan_backend == "sklearn" else "hdbscan"
            ),
            "hdbscan_project_min_samples": int(hdbscan_min_samples),
            "hdbscan_library_min_samples": library_min_samples,
            "hdbscan_min_samples_source": min_samples_source,
            "hdbscan_min_cluster_size": int(hdbscan_min_cluster_size),
            "hdbscan_cluster_selection_epsilon": epsilon,
            "hdbscan_cluster_selection_method": "eom",
        }
        if len(xyz) < max(hdbscan_min_cluster_size, library_min_samples):
            labels = np.full(len(xyz), -1, dtype=int)
        else:
            # 두 구현에 동일한 HDBSCAN 밀도값과 EOM 선택 방식을 전달한다.
            common = dict(
                min_cluster_size=hdbscan_min_cluster_size,
                min_samples=library_min_samples,
                metric="euclidean",
                cluster_selection_method="eom",
                cluster_selection_epsilon=epsilon,
                allow_single_cluster=True,
            )
            if hdbscan_backend == "sklearn":
                model = SklearnHDBSCAN(**common)
                backend_diagnostics["hdbscan_algorithm"] = "auto"
            else:
                if contrib_hdbscan is None:
                    raise RuntimeError(
                        "contrib hdbscan backend is unavailable: "
                        f"{type(_CONTRIB_IMPORT_ERROR).__name__}: {_CONTRIB_IMPORT_ERROR}"
                    )
                model = contrib_hdbscan.HDBSCAN(
                    **common,
                    algorithm="best",
                    approx_min_span_tree=True,
                    core_dist_n_jobs=1,
                )
                backend_diagnostics.update(
                    hdbscan_algorithm="best",
                    hdbscan_approx_min_span_tree=True,
                    hdbscan_core_dist_n_jobs=1,
                    hdbscan_match_reference_implementation=False,
                )
            # 군집 번호는 실행 순서 대신 공간 위치 순서로 다시 매겨 CSV를 안정화한다.
            labels = relabel_stable(model.fit_predict(features), xyz)
        diagnostics = {**empty, **backend_diagnostics}
        detail = (
            f"{hdbscan_backend} mcs{hdbscan_min_cluster_size}, "
            f"library-ms{library_min_samples} ({min_samples_source}), "
            f"epsilon{epsilon:g}, EOM"
        )
    elif base_method == "optics":
        if len(xyz) < optics_min_samples:
            labels, diagnostics = np.full(len(xyz), -1, dtype=int), empty
        else:
            with warnings.catch_warnings():
                warnings.filterwarnings("ignore", category=RuntimeWarning)
                warnings.filterwarnings(
                    "ignore", message="All reachability values are inf.*"
                )
                model = OPTICS(
                    min_samples=optics_min_samples,
                    max_eps=optics_max_eps,
                    xi=optics_xi,
                    min_cluster_size=optics_min_samples,
                    cluster_method="xi",
                    n_jobs=1,
                )
                labels = model.fit_predict(features)
            labels = relabel_stable(labels, xyz)
            diagnostics = {
                **empty,
                "reachability_inf_fraction": float(
                    np.mean(~np.isfinite(model.reachability_))
                ),
            }
        detail = f"N{optics_min_samples}, maxeps{optics_max_eps:g}, xi{optics_xi:g}"
    elif base_method == "retina_dbscan":
        if retina_params is None:
            raise ValueError("retina_params must be fitted for the current sensor")
        params = retina_params
        labels, diagnostics = retina_dbscan_labels(xyz, features, params)
        detail = (
            f"eps(r) {params.eps_base_m:g}..{params.eps_max_m:g}, "
            f"slope {params.eps_slope:g}, "
            f"N{params.min_samples_near}/{params.min_samples_far}"
        )
    elif base_method == "gmm":
        if gmm_components is None:
            raise ValueError("GMM is an oracle control and requires gmm_components")
        if not len(xyz) or gmm_components == 0:
            labels, diagnostics = np.full(len(xyz), -1, dtype=int), empty
        else:
            unique = len(np.unique(np.round(features, 4), axis=0))
            components = max(1, min(gmm_components, len(xyz), unique))
            labels = GaussianMixture(
                n_components=components,
                covariance_type="full",
                reg_covar=1e-3,
                n_init=1 if components == 1 else 5,
                random_state=17,
            ).fit(features).predict(features)
            labels, diagnostics = relabel_stable(labels, xyz), empty
        detail = f"components={gmm_components} GT oracle"
    else:
        raise ValueError(f"unknown clustering method: {method}")

    diagnostics = dict(diagnostics)
    diagnostics["_cluster_runtime_ms"] = (time.perf_counter_ns() - started_ns) / 1e6
    diagnostics["_merge_runtime_ms"] = 0.0
    diagnostics["clusters_before_merge"] = len(set(labels) - {-1})
    diagnostics["clusters_after_merge"] = len(set(labels) - {-1})
    diagnostics["merge_edges"] = 0
    if requested_merge:
        # HDBSCAN 뒤의 레이더 물리 조건 병합이며 noise 점은 군집으로 승격하지 않는다.
        diagnostics["_premerge_labels"] = labels.copy()
        merge_started_ns = time.perf_counter_ns()
        labels, merge_diagnostics = merge_radar_clusters(
            xyz,
            labels,
            np.zeros(len(xyz)) if doppler is None else doppler,
            np.zeros(len(xyz)) if power is None else power,
            merge_params or RadarClusterMergeParams(),
        )
        diagnostics.update(merge_diagnostics)
        diagnostics["_merge_runtime_ms"] = (
            time.perf_counter_ns() - merge_started_ns
        ) / 1e6
        detail += (
            f" + merge {merge_diagnostics['clusters_before_merge']}"
            f"->{merge_diagnostics['clusters_after_merge']}"
        )
    return labels, diagnostics, detail


# Compatibility surface for the completed experiment scripts.  New code should
# call build_features() and cluster_frame() explicitly instead of mutating these.
CLUSTER_INPUT_MODE = "xyz"
DOPPLER_MIN_ABS_MPS = 0.15
DOPPLER_MAX_ABS_MPS = 5.0
DOPPLER_FEATURE_WEIGHT_M_PER_MPS = 1.0
POWER_FEATURE_WEIGHT_M_PER_IQR = 0.25
RETINA_EPS_BASE_M = 1.00
RETINA_EPS_SLOPE = 0.010
RETINA_EPS_KNEE_M = 40.0
RETINA_EPS_MAX_M = 1.75
GMM_COMPONENTS = 1


def range_conditioned_power_score(
    power: np.ndarray, ranges: np.ndarray, bin_width_m: float, clip_iqr: float
) -> np.ndarray:
    score = np.zeros(len(power), dtype=float)
    finite = np.isfinite(power) & np.isfinite(ranges)
    bins = np.floor(ranges / max(float(bin_width_m), 1e-6)).astype(int)
    for bin_id in np.unique(bins[finite]):
        mask = finite & (bins == bin_id)
        values = np.asarray(power[mask], dtype=float)
        median = float(np.median(values))
        iqr = float(np.percentile(values, 75) - np.percentile(values, 25))
        score[mask] = (values - median) / max(iqr, 1.0)
    return np.clip(score, -abs(float(clip_iqr)), abs(float(clip_iqr)))


def spatial_params() -> RetinaDBSCANParams:
    return RetinaDBSCANParams(
        eps_base_m=RETINA_EPS_BASE_M,
        eps_slope=RETINA_EPS_SLOPE,
        eps_knee_m=RETINA_EPS_KNEE_M,
        eps_max_m=RETINA_EPS_MAX_M,
        min_samples_near=4,
        min_samples_far=3,
        nmin_transition_m=90.0,
    )


def moving_mask(doppler: np.ndarray) -> np.ndarray:
    return (
        np.isfinite(doppler)
        & (np.abs(doppler) >= DOPPLER_MIN_ABS_MPS)
        & (np.abs(doppler) <= DOPPLER_MAX_ABS_MPS)
    )


def shared_algorithm_features(
    xyz: np.ndarray, power: np.ndarray, doppler: np.ndarray
) -> tuple[np.ndarray, str]:
    power_feature = doppler_feature = None
    detail = ["XYZ"]
    if "doppler" in CLUSTER_INPUT_MODE:
        doppler_feature = np.where(moving_mask(doppler), doppler, 0.0)
        detail.append(f"Doppler×{DOPPLER_FEATURE_WEIGHT_M_PER_MPS:g}")
    if "power" in CLUSTER_INPUT_MODE:
        power_feature = range_conditioned_power_score(
            power, np.linalg.norm(xyz, axis=1), 10.0, 3.0
        )
        detail.append(f"PowerIQR×{POWER_FEATURE_WEIGHT_M_PER_IQR:g}")
    return (
        build_features(
            xyz,
            power_feature=power_feature,
            doppler_feature=doppler_feature,
            power_weight=POWER_FEATURE_WEIGHT_M_PER_IQR,
            doppler_weight=DOPPLER_FEATURE_WEIGHT_M_PER_MPS,
        ),
        "+".join(detail),
    )


def retina_adapted_feature_cluster(
    xyz: np.ndarray, features: np.ndarray
) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    return retina_dbscan_labels(xyz, features, spatial_params())


def cluster_variant(
    xyz: np.ndarray,
    power: np.ndarray,
    doppler: np.ndarray,
    _frame_ids: np.ndarray,
    variant: str,
) -> tuple[np.ndarray, dict[str, Any], str]:
    features, feature_detail = shared_algorithm_features(xyz, power, doppler)
    base_method = variant.removesuffix("_merged")
    if base_method == "retina_dbscan":
        # Historical gate experiments replaced this function temporarily.
        # Keep that one extension point only in the legacy wrapper.
        started_ns = time.perf_counter_ns()
        labels, diagnostics = retina_adapted_feature_cluster(xyz, features)
        labels = relabel_stable(labels, xyz)
        diagnostics = dict(diagnostics)
        diagnostics["_cluster_runtime_ms"] = (
            time.perf_counter_ns() - started_ns
        ) / 1e6
        diagnostics["_merge_runtime_ms"] = 0.0
        diagnostics["clusters_before_merge"] = len(set(labels) - {-1})
        diagnostics["clusters_after_merge"] = len(set(labels) - {-1})
        diagnostics["merge_edges"] = 0
        params = spatial_params()
        detail = (
            f"eps(r) {params.eps_base_m:g}..{params.eps_max_m:g}, "
            f"slope {params.eps_slope:g}, "
            f"N{params.min_samples_near}/{params.min_samples_far}"
        )
        if variant.endswith("_merged"):
            diagnostics["_premerge_labels"] = labels.copy()
            merge_started_ns = time.perf_counter_ns()
            labels, merge_diagnostics = merge_radar_clusters(
                xyz,
                labels,
                doppler,
                power,
                RadarClusterMergeParams(),
            )
            diagnostics.update(merge_diagnostics)
            diagnostics["_merge_runtime_ms"] = (
                time.perf_counter_ns() - merge_started_ns
            ) / 1e6
            detail += (
                f" + merge {merge_diagnostics['clusters_before_merge']}"
                f"->{merge_diagnostics['clusters_after_merge']}"
            )
        return labels, diagnostics, f"{detail} | {feature_detail}"

    labels, diagnostics, detail = cluster_frame(
        xyz,
        features,
        variant,
        doppler=doppler,
        power=power,
        retina_params=spatial_params(),
        gmm_components=GMM_COMPONENTS,
    )
    return labels, diagnostics, f"{detail} | {feature_detail}"


def self_check() -> None:
    rng = np.random.default_rng(7)
    xyz = np.vstack((rng.normal(0, 0.03, (8, 3)), rng.normal(2, 0.03, (8, 3))))
    features = build_features(xyz)
    labels, diagnostics = retina_dbscan_labels(
        xyz,
        features,
        RetinaDBSCANParams(
            eps_base_m=0.2,
            eps_slope=0.0,
            eps_knee_m=0.0,
            eps_max_m=0.2,
            min_samples_near=4,
            min_samples_far=4,
            nmin_transition_m=90.0,
        ),
    )
    assert len(set(labels) - {-1}) == 2
    assert diagnostics["core"].all()
    assert len(cluster_frame(xyz, features, "hdbscan")[0]) == len(xyz)
    sparse_labels, _, _ = cluster_frame(
        xyz[:3], features[:3], "hdbscan",
        hdbscan_min_cluster_size=3,
        hdbscan_min_samples_library=4,
    )
    assert np.all(sparse_labels == -1)
    assert hdbscan_library_min_samples("sklearn", 3) == 3
    assert hdbscan_library_min_samples("contrib", 3) == 2
    try:
        cluster_frame(xyz, features, "hdbscan", hdbscan_backend="unknown")
    except ValueError:
        pass
    else:
        raise AssertionError("unknown HDBSCAN backend must fail")
    if contrib_hdbscan is not None:
        contrib_labels, diagnostics, _ = cluster_frame(
            xyz,
            features,
            "hdbscan",
            hdbscan_backend="contrib",
            hdbscan_cluster_selection_epsilon=0.25,
        )
        assert len(contrib_labels) == len(xyz)
        assert diagnostics["hdbscan_library_min_samples"] == 2
    assert len(cluster_frame(xyz, features, "optics")[0]) == len(xyz)


if __name__ == "__main__":
    self_check()
    print("self-check: OK")
