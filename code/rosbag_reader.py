#!/usr/bin/env python3
"""Read radar frames from a finalized ROS 2 SQLite bag."""

from __future__ import annotations

from pathlib import Path
import sqlite3
from typing import Iterator

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


RADAR_TOPIC = "/radar_pcl2"
FIELDS = ("x", "y", "z", "power", "doppler")


def database_path(path: Path) -> Path:
    if path.is_file() and path.suffix == ".db3":
        return path
    candidates = sorted(path.glob("*.db3"))
    if len(candidates) != 1:
        raise RuntimeError(f"expected one .db3 in {path}, found {len(candidates)}")
    return candidates[0]


def pointcloud_array(message) -> np.ndarray:
    fields = {field.name: field for field in message.fields}
    missing = [name for name in FIELDS if name not in fields]
    if missing:
        raise RuntimeError(f"PointCloud2 missing fields: {missing}")
    names = FIELDS + (("range_m",) if "range_m" in fields else ())
    endian = ">" if message.is_bigendian else "<"
    dtype = np.dtype(
        {
            "names": names,
            "formats": [f"{endian}f4"] * len(names),
            "offsets": [int(fields[name].offset) for name in names],
            "itemsize": int(message.point_step),
        }
    )
    return np.frombuffer(
        message.data,
        dtype=dtype,
        count=int(message.width) * int(message.height),
    ).copy()


def read_frames(path: Path, start_s: float, end_s: float) -> Iterator[dict]:
    database = database_path(path)
    connection = sqlite3.connect(f"file:{database}?mode=ro&immutable=1", uri=True)
    try:
        topic = connection.execute(
            "SELECT id,type FROM topics WHERE name=?", (RADAR_TOPIC,)
        ).fetchone()
        if topic is None:
            raise RuntimeError(f"{RADAR_TOPIC} not found in {database}")
        topic_id, type_name = int(topic[0]), str(topic[1])
        first_record_ns = int(
            connection.execute(
                "SELECT timestamp FROM messages WHERE topic_id=? ORDER BY timestamp LIMIT 1",
                (topic_id,),
            ).fetchone()[0]
        )
        message_type = get_message(type_name)
        rows = connection.execute(
            "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (topic_id,),
        )
        for frame_index, (record_ns, payload) in enumerate(rows):
            relative_s = (int(record_ns) - first_record_ns) / 1e9
            if relative_s < start_s:
                continue
            if relative_s > end_s:
                break
            points = pointcloud_array(deserialize_message(payload, message_type))
            xyz = np.column_stack([points[name] for name in ("x", "y", "z")])
            computed_range = np.linalg.norm(xyz, axis=1)
            published_range = (
                np.asarray(points["range_m"], dtype=float)
                if "range_m" in points.dtype.names
                else computed_range
            )
            yield {
                "frame_index": frame_index,
                "relative_s": relative_s,
                **{name: np.asarray(points[name], dtype=float) for name in FIELDS},
                "range": np.where(np.isfinite(published_range), published_range, computed_range),
            }
    finally:
        connection.close()
