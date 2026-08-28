# LRR Vehicle Radar Clustering

**Parking lot:** HDBSCAN FIFO2 reached a vehicle-cluster F1 of **0.8568** with a p95 runtime of **42.31 ms**, within the radar's 50 ms frame period.

**Long range:** HDBSCAN FIFO5 reached a vehicle-cluster F1 of **0.8964** with a p95 runtime of **16.44 ms**. Vehicle returns were observed up to **261 m**; at the far end, intermittent raw radar returns were the main source of flicker.

## Qualitative Results

Click a preview to open the full MP4.

### Moving vehicle in a parking lot · about 20 m

[![Moving vehicle in a parking lot](assets/previews/parking_away20.gif)](videos/parking_lot/away20_hdbscan_fifo2.mp4)

### Static vehicle in a parking lot · about 50 m

[![Static vehicle in a parking lot](assets/previews/parking_static50.gif)](videos/parking_lot/static50_hdbscan_fifo2.mp4)

### Static vehicle at 150 m

[![Static vehicle at 150 m](assets/previews/long_range_150m.gif)](videos/long_range/150m_fifo2_fifo7.mp4)

### Static vehicle at 170 m

[![Static vehicle at 170 m](assets/previews/long_range_170m.gif)](videos/long_range/170m_fifo2_fifo7.mp4)

### Approaching vehicle · up to about 175 m

[![Approaching vehicle from 175 m](assets/previews/long_range_175m.gif)](videos/long_range/parking_175m.mp4)

### Road sequence · about 50–261 m

[![Long-range road sequence](assets/previews/long_range_road.gif)](videos/long_range/road_50m_to_261m.mp4)

## Method

```text
rosbag → ROI and feature preprocessing → FIFO accumulation → HDBSCAN → cluster merging → labeled CSV
```

HDBSCAN is applied after FIFO accumulation and Power/Doppler preprocessing. Nearby clusters are merged in a final post-processing step.

| Scene | HDBSCAN parameters | Input |
|---|---|---|
| Parking lot | FIFO2, `min_cluster_size=8`, `min_samples=4`, `cluster_selection_epsilon=1.5 m` | XYZ + Power 0.25 + Doppler 0.40 |
| Long range | FIFO5, `min_cluster_size=3`, `min_samples=2`, `cluster_selection_epsilon=2.0 m` | XYZ after Doppler motion gating |

The videos use `sklearn.cluster.HDBSCAN`. Here, epsilon refers to HDBSCAN's `cluster_selection_epsilon`, not DBSCAN's `eps`.

## Run

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
source /opt/ros/humble/setup.bash

python code/cluster_parking_lot.py away20 --bag /path/to/rosbag
python code/cluster_long_range_vehicle.py parking_175m --bag /path/to/rosbag
```

- Parking-lot scenes: `away20`, `static50`, `vehicle150`, `vehicle170`
- Long-range scenes: `parking_175m`, `road_261m`

Each script writes a CSV with a `cluster_label` for every radar point.

Raw rosbags are not included. The scripts reproduce the clustering labels used in the videos; RViz capture and MP4 composition are not part of this repository.

## Repository

```text
code/       clustering and rosbag processing
results/    evaluation summaries
videos/     qualitative results
```

## Acknowledgement

This work was supported by the Electronics and Telecommunications Research Institute (ETRI).
