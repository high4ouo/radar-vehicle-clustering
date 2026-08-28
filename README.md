# LRR 차량 레이더 클러스터링

주차장과 장거리 도로에서 수집한 LRR 포인트 클라우드에 HDBSCAN을 적용했다. 원거리에서 한 프레임의 점이 적어지는 문제는 짧은 FIFO 누적과 Power·Doppler 전처리로 보완했다.

## 결과 영상

이미지를 누르면 원본 MP4가 열린다.

### 주차장 이동 차량 · 약 20 m

[![주차장 이동 차량](assets/previews/parking_away20.gif)](videos/parking_lot/away20_hdbscan_fifo2.mp4)

### 주차장 정지 차량 · 약 50 m

[![주차장 정지 차량](assets/previews/parking_static50.gif)](videos/parking_lot/static50_hdbscan_fifo2.mp4)

### 장거리 정지 차량 · 150 m

[![150 m 정지 차량](assets/previews/long_range_150m.gif)](videos/long_range/150m_fifo2_fifo7.mp4)

### 장거리 정지 차량 · 170 m

[![170 m 정지 차량](assets/previews/long_range_170m.gif)](videos/long_range/170m_fifo2_fifo7.mp4)

### 주차장 접근 차량 · 최대 약 175 m

[![175 m 접근 차량](assets/previews/long_range_175m.gif)](videos/long_range/parking_175m.mp4)

### 도로 주행 차량 · 약 50–261 m

[![장거리 도로 주행 차량](assets/previews/long_range_road.gif)](videos/long_range/road_50m_to_261m.mp4)

## 결과

| 실험 | 설정 | 차량 군집 F1 | p95 실행시간 |
|---|---|---:|---:|
| 주차장 | HDBSCAN FIFO2 | 0.8568 | 42.31 ms |
| 장거리 | HDBSCAN FIFO5 | 0.8964 | 16.44 ms |

주차장 설정은 LRR의 20 Hz 주기인 50 ms 안에서 동작했다. 장거리 도로에서는 차량을 약 261 m까지 확인했지만, 거리가 멀어질수록 레이더 반사점 자체가 간헐적으로 들어와 결과도 함께 끊겼다.

## 방법

```text
rosbag → ROI/전처리 → FIFO 누적 → HDBSCAN → 가까운 군집 병합 → CSV
```

| 실험 | 주요 파라미터 | 입력 특징 |
|---|---|---|
| 주차장 | FIFO2, `min_cluster_size=8`, `min_samples=4`, `cluster_selection_epsilon=1.5 m` | XYZ + Power 0.25 + Doppler 0.40 |
| 장거리 | FIFO5, `min_cluster_size=3`, `min_samples=2`, `cluster_selection_epsilon=2.0 m` | Doppler 이동점 선택 후 XYZ |

영상 기준 구현은 `sklearn.cluster.HDBSCAN`이다. 위 epsilon은 일반 DBSCAN의 `eps`가 아니라 HDBSCAN의 `cluster_selection_epsilon`이다.

## 실행

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
source /opt/ros/humble/setup.bash

python code/cluster_parking_lot.py away20 --bag /path/to/rosbag
python code/cluster_long_range_vehicle.py parking_175m --bag /path/to/rosbag
```

주차장 장면은 `away20`, `static50`, `vehicle150`, `vehicle170`, 장거리 장면은 `parking_175m`, `road_261m`을 지원한다. 결과는 점별 `cluster_label`이 포함된 CSV로 저장된다.

원본 rosbag은 공개하지 않는다. 제공 코드는 영상과 같은 클러스터링 결과를 만들지만 RViz 화면과 MP4까지 자동으로 생성하지는 않는다.

## 구성

```text
code/       클러스터링 코드
results/    평가 요약 CSV/JSON
videos/     결과 영상
```

## Acknowledgement

본 연구는 한국전자통신연구원(ETRI)의 지원을 받아 수행되었습니다.
