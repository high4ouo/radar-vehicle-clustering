# LRR 차량 레이더 포인트 클라우드 클러스터링

> 희소하고 거리별 밀도가 달라지는 LRR 레이더 포인트에서 차량을 안정적으로 묶기 위해, 실제 주차장과 장거리 도로 환경에서 밀도 기반 클러스터링을 설계·평가한 연구이다.

이 저장소는 주차장과 장거리 차량 실험에서 사용한 클러스터링 코드, 최종 파라미터, 평가 결과와 정성 영상으로 구성된다. 원본 rosbag 데이터는 포함하지 않는다.

## 한눈에 보기

| 항목 | 내용 |
|---|---|
| 연구 질문 | 희소한 레이더 포인트만으로 차량을 구분하면서 20 Hz 처리 주기를 만족할 수 있는가? |
| 실험 환경 | 주차장, 150/170/175 m 원거리 주차장, 50–261 m 도로 주행 |
| 최종 방법 | HDBSCAN + FIFO 누적 + 거리별 Power/Doppler 전처리 + 물리 조건 기반 군집 병합 |
| 주차장 결과 | HDBSCAN이 RETINA-DBSCAN보다 차량 군집 F1이 높고 평균 실행시간이 약 2.2배 빨랐음 |
| 실시간 설정 | HDBSCAN FIFO2, p95 `42.31 ms`로 LRR 20 Hz의 `50 ms` 주기 충족 |
| 장거리 결과 | FIFO5 차량 군집 F1 `0.896`, p95 `16.44 ms`; 원거리 성능 저하는 클러스터링뿐 아니라 센서 반환의 간헐성에 크게 좌우됨 |

## 연구 개요 — STAR

### S — 배경과 문제

LRR 레이더는 카메라와 달리 거리와 방사속도를 직접 측정할 수 있지만, 한 물체에서 얻는 포인트 수가 적고 거리·방향·환경에 따라 포인트 밀도가 크게 변한다. 특히 거리가 증가하면 반사 Power가 감소하고 차량 포인트가 프레임마다 끊겨, 하나의 고정 반경을 사용하는 일반적인 클러스터링만으로는 차량을 안정적으로 묶기 어렵다.

또한 사용한 LRR 센서는 20 Hz로 새 프레임을 제공하므로, 온라인 적용을 위해서는 한 프레임의 처리를 다음 프레임이 들어오는 `50 ms` 안에 끝내야 한다.

### T — 연구 목표

1. 실제 차량 주행 구간에서 레이더 포인트와 차량 위치를 동기화해 평가 기준을 만든다.
2. HDBSCAN과 거리 적응형 RETINA-DBSCAN의 차량 군집 품질과 실행시간을 비교한다.
3. FIFO 누적 프레임 수를 조절해 정확도와 20 Hz 처리시간 사이의 운용점을 결정한다.
4. 150 m 이상의 장거리에서 차량 포인트가 어디까지 관측되고, 성능 저하가 센서 반환과 클러스터링 중 어디에서 발생하는지 구분한다.

### A — 수행 방법

```text
카메라·레이더 시간 동기화
        ↓
차량 궤적 및 배경 구조 GT 생성
        ↓
ROI와 유효 포인트 선택
        ↓
XYZ + 거리별 Power 정규화 + Doppler 특징 구성
        ↓
FIFO 1/2/5/7/10 프레임 누적
        ↓
알고리즘별 27개 파라미터 조합 평가
        ↓
거리·방위각·Doppler가 유사한 군집 병합
        ↓
차량 F1·배경 Semantic F1·과분할률·실행시간 평가
```

- 주차장에서는 차량뿐 아니라 유리벽, 건물, 도로 끝 수목을 semantic 영역으로 표시해 배경 구조가 차량 군집에 섞이는 정도도 함께 평가했다.
- 거리 증가에 따른 Power 분포 차이를 줄이기 위해 거리 구간별 Power를 정규화하고, Doppler를 특징값에 반영했다.
- 장거리 이동 차량은 주변 정지 차량과 배경 clutter를 줄이기 위해 `|Doppler| >= 0.15 m/s`인 포인트만 선택했다.
- 밀도 클러스터링 후 공간 거리, 거리축 차이, 방위각과 Doppler가 물리적으로 유사한 군집을 병합했다.

### R — 핵심 결과와 결론

#### 1. 주차장: HDBSCAN 선택

약 20 km/h 직선 주행의 held-out 52프레임에서 FIFO7 기준으로 비교했다.

| 알고리즘 | 차량 Point Recall | 차량 Cluster F1 | Semantic F1 | 과분할률 | 평균 / p95 |
|---|---:|---:|---:|---:|---:|
| **HDBSCAN** | **0.9992** | **0.8204** | 0.9266 | 18.42% | **134.83 / 171.48 ms** |
| RETINA-DBSCAN | 0.9950 | 0.7943 | **0.9288** | **18.05%** | 292.99 / 377.58 ms |

HDBSCAN은 차량 포인트를 더 깨끗하게 묶었고 평균 실행시간이 약 2.2배 빨랐다. RETINA-DBSCAN은 배경 구조의 Semantic F1과 과분할률에서 근소하게 우세했지만, 차량 군집 품질과 처리시간을 함께 고려해 HDBSCAN을 선택했다.

평가 원본: [알고리즘 비교 CSV](results/parking_lot/fifo_selected_summary.csv) · [최종 설정 JSON](results/parking_lot/final_configs_fifo.json)

#### 2. 20 Hz 실시간 운용점: FIFO2

FIFO3의 차량 F1은 FIFO2보다 약 `0.002` 높았지만 처리시간이 `50 ms`를 넘었다. 정확도를 거의 유지하면서 20 Hz 주기 안에 처리되는 FIFO2를 최종 실시간 설정으로 선정했다.

| 설정 | 차량 Cluster F1 | Semantic F1 | 평균 / p95 |
|---|---:|---:|---:|
| **HDBSCAN FIFO2** | **0.8568** | **0.9175** | **35.64 / 42.31 ms** |

50 m 정지 차량의 클러스터 중앙값은 실제 배치 거리와 가까운 `48.672 m`로 측정됐다.

평가 원본: [FIFO2/FIFO4 비교 CSV](results/parking_lot/hdbscan_fifo2_fifo4_heldout_summary.csv)

#### 3. 장거리 차량: 검출 가능 거리와 간헐성 확인

- 150 m와 170 m 정지 차량 모두 포인트가 관측됐지만, 170 m에서는 Power가 더 낮고 출력이 더 자주 끊겼다.
- 주차장 접근 실험에서는 최대 약 `175.22 m`에서 접근하는 차량을 분리했다.
- 도로 실험에서는 약 50 m부터 `261.44 m`까지 주행하는 차량을 추적했으며, 더 먼 구간에서는 차량 포인트가 2개 수준으로 감소하고 간헐적으로 출력됐다.

장거리 최종 운용점은 FIFO5이다. FIFO7의 F1 이득은 `0.0016`에 불과했지만 과분할률이 약 2.0%p 높아, 정확도와 안정성의 절충으로 FIFO5를 선택했다.

| FIFO5 held-out | 값 |
|---|---:|
| Target Point Recall | 0.9142 |
| Matched Cluster Precision | 0.8792 |
| Matched Cluster F1 | **0.8964** |
| 차량 탐지율 | 97.3% |
| 평균 / p95 실행시간 | **9.65 / 16.44 ms** |

장거리에서 차량이 보이지 않는 원인을 센서 반환과 클러스터링으로 나눠 확인했다.

| 장면 | 원본 차량 반환 | 차량 군집 표시 | 반환이 있을 때 군집 회수율 |
|---|---:|---:|---:|
| 175 m 주차장 접근 | 92.8% | 92.5% | **99.7%** |
| 장거리 도로 평가 구간 | 28.7% | 26.6% | **92.5%** |

따라서 장거리 도로에서 나타난 깜빡임의 주된 원인은 클러스터링 실패만이 아니라, 거리와 실험 환경에 따라 차량 반사점 자체가 간헐적으로 들어오는 센서 관측 한계였다.

평가 원본: [FIFO 비교](results/long_range/operating_point_summary.csv) · [거리별 결과](results/long_range/operating_point_distance_summary.csv) · [장면별 결과](results/long_range/operating_point_scene_summary.csv)

## 최종 클러스터링 설정

| 구분 | 구현 | 주요 파라미터 | 입력과 후처리 |
|---|---|---|---|
| 주차장 | `sklearn.cluster.HDBSCAN` | FIFO2, `min_cluster_size=8`, `min_samples=4`, `cluster_selection_epsilon=1.5 m` | XYZ + Power `0.25` + Doppler `0.40`, 군집 병합 |
| 장거리 차량 | `sklearn.cluster.HDBSCAN` | FIFO5, `min_cluster_size=3`, `min_samples=2`, `cluster_selection_epsilon=2.0 m` | Doppler 이동 gate 이후 XYZ |

이 문서의 HDBSCAN(ε)은 HDBSCAN의 `cluster_selection_epsilon`을 사용한 설정을 뜻한다. 일반 DBSCAN의 고정 이웃 반경 `eps`와는 다른 파라미터다.

현재 코드는 결과 영상을 생성할 때 사용한 scikit-learn HDBSCAN을 재현 기준으로 유지한다. 외부 `hdbscan` 구현의 품질과 실행시간 비교는 기존 결과를 보존한 별도 실험으로 진행하며, 채택할 경우 비교 근거와 함께 별도 커밋으로 반영한다.

## 결과 영상

### 주차장

| 장면 | 설명 | 영상 |
|---|---|---|
| 이동 차량 | 약 20 m에서 멀어지는 차량, HDBSCAN FIFO2 | [영상 보기](videos/parking_lot/away20_hdbscan_fifo2.mp4) |
| 정지 차량 | 약 50 m 정지 차량, HDBSCAN FIFO2 | [영상 보기](videos/parking_lot/static50_hdbscan_fifo2.mp4) |

### 장거리 차량

| 장면 | 설명 | 영상 |
|---|---|---|
| 150 m | HDBSCAN FIFO2와 FIFO7 비교 | [영상 보기](videos/long_range/150m_fifo2_fifo7.mp4) |
| 170 m | HDBSCAN FIFO2와 FIFO7 비교 | [영상 보기](videos/long_range/170m_fifo2_fifo7.mp4) |
| 175 m 주차장 | 원거리에서 레이더 방향으로 접근하는 차량 | [영상 보기](videos/long_range/parking_175m.mp4) |
| 50–261 m 도로 | 거리가 증가하는 이동 차량과 원거리 포인트 간헐성 | [영상 보기](videos/long_range/road_50m_to_261m.mp4) |

## 실행 방법

### 환경

- Python 3
- ROS 2 Humble
- `requirements.txt`의 Python 패키지

```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
source /opt/ros/humble/setup.bash
```

### 주차장

```bash
python code/cluster_parking_lot.py away20 \
  --bag /path/to/camera_100546

python code/cluster_parking_lot.py static50 \
  --bag /path/to/camera_095549

python code/cluster_parking_lot.py vehicle150 \
  --bag /path/to/camera_103644

python code/cluster_parking_lot.py vehicle170 \
  --bag /path/to/camera_103644
```

### 장거리 차량

```bash
python code/cluster_long_range_vehicle.py parking_175m \
  --bag /path/to/camera_200530

python code/cluster_long_range_vehicle.py road_261m \
  --bag /path/to/camera_201136
```

결과는 점별 `cluster_label`이 포함된 CSV로 저장된다. `cluster_label=-1`은 어느 군집에도 포함되지 않은 noise를 뜻한다.

## 재현 범위와 제한사항

- 원본 rosbag 데이터는 저장소에 포함하지 않는다. 실행할 때 `--bag`에 로컬 rosbag 디렉터리를 지정해야 한다.
- 실행 코드는 최종 영상과 같은 ROI, 특징값, FIFO와 HDBSCAN 설정으로 클러스터링 CSV를 생성한다.
- 제공된 MP4는 정성 결과 기록이다. 위 명령만으로 카메라 합성, RViz 화면과 MP4가 다시 생성되지는 않는다.
- 장거리 결과의 `target_cluster`는 확인 궤적과 가장 가까운 군집을 선택한 평가 결과이며, 학습된 차량 분류기의 출력이 아니다.
- 실행시간은 클러스터링 호출 직전부터 label 계산 완료까지 측정한 CPU 오프라인 결과로, ROS node 전체의 hard real-time 보장을 뜻하지 않는다.

## 저장소 구성

```text
code/                         클러스터링 및 rosbag 처리 코드
results/parking_lot/          주차장 평가 요약
results/long_range/           장거리 평가 요약
videos/parking_lot/           주차장 결과 영상
videos/long_range/            장거리 차량 결과 영상
README.md                     연구·실행·결과 안내
requirements.txt              검증한 Python 패키지 버전
```

## 사사 (Acknowledgement)

본 연구는 한국전자통신연구원(ETRI)의 지원을 받아 수행되었습니다.
