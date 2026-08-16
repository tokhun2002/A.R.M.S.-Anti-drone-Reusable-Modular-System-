# arms_detection_node 동작 상세

A.R.M.S. 의 표적(빨간 풍선) 검출 노드. `/arms/image_raw` 를 받아 표적 하나를
`/arms/detections` 로 발행한다. 한 프로세스 안에서 **YOLO + 고전 CV(HSV·ABSDIFF)**
를 함께 돌리고, **detect-then-track FSM** 으로 추적 구간 비용을 줄이며, **시간 연속성**
을 확인해 헛-LOCK 을 막는다.

- 코드: `SW/arms_ws/src/arms_detection/arms_detection/arms_detection_node.py`
- 배포: GPU 도커 컨테이너(`docker-compose.jetson.yml`). 소스는 바인드마운트라
  코드 수정은 `docker restart docker-arms_detection-1` 로 반영.

---

## 1. 토픽 / 인터페이스

| 방향 | 토픽 | 타입 | 설명 |
| --- | --- | --- | --- |
| 구독 | `/arms/image_raw` | `sensor_msgs/Image` | 카메라/replay 영상 (rgb8·bgr8·mono8) |
| 발행 | `/arms/detections` | `arms_msgs/DetectionArray` | 표적 **0 또는 1개** (best 하나) |
| 발행 | `/arms/roi_image` | `sensor_msgs/Image` | 추적 표적 확대 크롭 (bgr8) |
| 발행 | `/arms/debug_image` | `sensor_msgs/Image` | 시각화 (bgr8) |
| 발행 | `/arms/debug_absdiff` | `sensor_msgs/Image` | absdiff 이진 마스크 (mono8) |
| 발행 | `/arms/detector_status` | `std_msgs/Float32MultiArray` | `[yolo, hsv, absdiff, mode]` — UI 표시용 |

발행 최적화: `roi_image`·`debug_image`·`debug_absdiff` 는 **구독자가 있을 때만** 그려서
발행한다(없으면 연산 전부 생략).

---

## 2. 전체 파이프라인 (`_cb_image`)

프레임 한 장이 들어올 때마다:

1. **빈 프레임 스킵** (`width/height/data == 0`).
2. `_frame_i += 1` — YOLO 스로틀용 프레임 카운터.
3. **BGR 변환** (`imgmsg_to_bgr`): rgb8 이면 채널을 뒤집어 BGR 로.
4. **다운스케일**: `proc_width`(기본 640) 로 축소해 검출한다(CV 비용 절감).
   원본(`bgr_full`)은 ROI 확대 크롭용으로 따로 보관. 출력은 정규화 좌표라 정확도 무관.
5. **`_TargetTracker.update(bgr, detect_fn, cfg)`** 호출 → 발행할 박스(또는 None).
   여기서 `detect_fn = _run_detectors` 이며, **FSM 이 검출을 언제/어디에 돌릴지 결정**한다.
6. **`DetectionArray` 발행** — 표적 있으면 1개, 없으면 빈 배열.
7. 표적 있으면 **ROI 크롭 발행**(full-res 에서), 구독자 있을 때만.
8. **디버그 이미지 발행**, `publish_debug` 이고 구독자 있을 때만.

> 노드는 한 프레임에 **표적 1개만** 발행한다. 여러 표적 선택은 하지 않는다.
> control 의 상태머신은 이 박스의 `confidence` 가 `detection_confidence_threshold`
> (기본 0.32) 이상이고 `lock_duration_sec` 만큼 연속되면 LOCK 한다.

---

## 3. 검출기 3종 (우선순위: YOLO > 융합 CV > 단일 CV)

### 3.1 YOLO — `_detect_yolo`

같은 프로세스에서 ultralytics 로 직접 추론한다(서브프로세스/서버 없음).

- 입력: **BGR ndarray 그대로**. ultralytics 가 내부에서 RGB 변환하므로 **미리 뒤집으면
  R/B 가 두 번 바뀌어 빨강 성능이 급락**한다(과거 버그, 지금은 BGR 그대로 넣음).
- `imgsz`(기본 320), `conf`(기본 0.32 = 카메라 학습모델 F1 최적점), `iou`(0.45)
  는 환경변수(`ARMS_IMGSZ/ARMS_CONF/ARMS_IOU`)로 주입.
- 결과 중 **최고 confidence 박스 1개** 를 정규화 좌표로 반환.
- 모델(`ARMS_MODEL`)/ultralytics 가 없으면(SITL/호스트) **조용히 비활성** → CV 로만 동작.

**OOM/실패 복구 (핵심 안전장치):** TensorRT 엔진은 첫 `predict()` 에서 지연 로딩되는데
GPU 메모리 부족 등으로 실패할 수 있다. 그때:

- 예외를 잡아 노드가 죽지 않게 한다(죽으면 재시작→CUDA 컨텍스트 미해제로 device busy 폭주).
- `ARMS_YOLO_RETRY_SEC`(기본 3초) 쿨다운을 걸고 그동안 None 반환(→ CV 폴백).
- **연속 `ARMS_YOLO_MAX_FAILS`(기본 3)회 실패하면 YOLO 를 영구 비활성**(`self._yolo=None`)
  하고 CV 로만 동작. (계속 재시도하면 executor 가 블로킹돼 검출이 아예 멈추는 걸 방지.)
- 이후 한 번이라도 성공하면 실패 카운터를 리셋하고 복구 로그를 남긴다.

### 3.2 HSV — `_detect_hsv` (+ `_red_probability`)

하드 `inRange` 대신 **빨강 soft-확률 맵 + 형태 + 텍스처** 를 결합한다.

- **`_red_probability`**: 픽셀별 0~1 빨강 점수 =
  `hue_score × sat_score × val_low × val_high`.
  - hue: OpenCV hue(0~179)에서 빨강이 0/179 양끝에 걸리므로 **원형 hue 거리의 Gaussian**
    (`hue_sigma`)로 평가 → 경계 하드컷 없음.
  - sat: 시그모이드(`sat_center/sat_scale`) — 채도 낮으면 감점.
  - val: 너무 어둡(val_low)거나 완전 포화된 LED/조명(val_high)은 감점(하드컷은 안 함).
- 확률맵을 `prob_threshold`(0.20)로 이진화 → `MORPH_CLOSE`(작은 구멍만 메움, 소형 점은 보존).
- 각 컨투어에 대해 게이팅+점수:
  - **면적 게이트**: `hsv.min_area_ratio ~ max_area_ratio` 밖이면 제외.
  - **형태 점수**: circularity(원형도) 와 aspect(가로세로비) 중 큰 값. 1~2px 원거리
    점은 contourArea 가 0이 될 수 있어 aspect 로 대신 평가. `min_circularity` 미만 제외.
  - **색 점수**: 박스 영역의 `red_prob` 평균.
  - **텍스처 점수**: 주변(context)의 라플라시안 분산 → **배경이 매끈(하늘)하면 높고,
    어수선(지상 클러터)하면 낮음**(`texture_scale`). 빨간 건물/현수막 억제.
  - 최종 score = `color × (0.35+0.65·shape) × (0.5+0.5·aspect) × (0.15+0.85·smooth)`.
- confidence = `min(0.60, 0.45·color + 0.35·shape + 0.10·size + 0.10·smooth)`.
  **상한 0.60** — 단독으로는 LOCK 임계값을 넘지 못하게 캡(§5 정책).
- class_name = `"hsv_red"`.

### 3.3 ABSDIFF — `_detect_absdiff`

색·형태와 무관하게 **배경과 다른 점** 을 찾는다(원거리 소형 표적에 강함).

- gray → `pre_blur` → 배경추정 `bg_blur` Gaussian → `absdiff(gray, bg)` → `diff_thresh`
  이진화 → `MORPH_CLOSE`(원거리 수-픽셀 표적이 OPEN 으로 사라지지 않게 close 만).
- **클러터 게이트**: 컨투어 수가 `absdiff.max_blobs`(기본 100) 초과면 어수선한 장면으로
  보고 통째로 버림.
- 각 컨투어 점수:
  - **면적 게이트**: `absdiff.min_area_ratio ~ max_area_ratio`.
  - **대비**: diff patch 의 최대값.
  - **색 점수**: `red_prob` 평균 — 색을 필수로 하진 않되(바닥 0.15) 붉은 점을 강하게 우선.
  - **형태 점수**: circularity/aspect.
  - score = `contrast × area^0.3 × (0.15+0.85·red) × (0.35+0.65·shape)`.
  - **`hint`(HSV 박스)가 주어지면** 그 중심 근처일수록 가점(Gaussian proximity) —
    HSV 와 같은 곳을 보는 점을 우선.
- confidence = `min(0.60, 0.35·size + 0.30·contrast + 0.20·red + 0.15·shape)`. 상한 0.60.
- class_name = `"absdiff_spot"`.

---

## 4. CV 융합 & YOLO proposal (원거리 표적 살리기)

### 4.1 `_fuse_cv` — HSV+ABSDIFF 합의

HSV 박스와 ABSDIFF 박스가 **같은 위치**(중심거리 `fusion.max_center_dist`, 박스 크면
비례 확장)를 가리킬 때만 하나로 융합한다.

- 중심 = confidence 가중 평균, 크기 = 두 박스 중 큰 값.
- confidence = `min(0.60, 0.5·(w_hsv+w_abs))`. 역시 상한 0.60.
- class_name = `"cv_fused"`. **두 독립 신호(색/대비)가 합의**했다는 뜻이라 단일 CV 보다 신뢰.

### 4.2 `_detect_yolo_proposal` — CV 위치를 확대해 YOLO 재확인

전체프레임 YOLO 가 놓친 작은 표적을, **CV 융합 위치 주변을 `yolo.proposal_crop_px`(192px)
고정 크기로 크롭 → YOLO 재추론**한다. ultralytics 가 작은 크롭을 `imgsz` 로 확대하므로
원본에서 수 픽셀인 풍선도 학습 때 ROI 샘플과 비슷한 크기로 보여 인식된다. 확인되면
그 YOLO 박스를 full-frame 좌표로 역변환해 채택한다. (FULL 모드에서만 동작.)

---

## 5. 검출 stack & 획득 정책 (`_detect_stack`)

한 이미지에 대한 우선순위 파이프라인:

1. `run_yolo` 이면 전체프레임 YOLO.
2. `need_cv`(= 상태표시 필요 or (allow_cv & YOLO 놓침)) 이면 HSV·ABSDIFF·융합 실행.
3. YOLO 가 놓쳤고 융합 후보가 있으면 **§4.2 proposal-crop YOLO** 재시도.
4. `report_status` 면 `/arms/detector_status` 발행.
5. **winner 선정**: `YOLO > cv_fused > (hsv or absdiff)`. `allow_cv=False` 면 YOLO 만.

**헛-LOCK 방지 정책 (가장 중요):**

- 모든 CV confidence 는 **0.60 상한**이라, 한 프레임의 CV 후보는 confidence 가 낮게
  유지된다. 이를 LOCK 임계값 위로 올려주는 승격은 **시간 연속성**을 확인한 뒤에만
  `_TargetTracker` 가 수행한다(§6). 즉 CV 후보는 여러 프레임 같은 위치에서 합의해야만
  승격되고, 순간적인 빨간 조명/건물은 승격되지 못해 LOCK 되지 않는다.
- `acquire.yolo_only=True` 로 두면 획득 단계에서 CV 를 아예 배제(YOLO 전용)할 수도 있다
  (기본 False = CV 후보도 함께 쓰되 시간확인으로 억제).

---

## 6. detect-then-track FSM (`_TargetTracker`)

검출을 매 프레임 전체로 돌리면 비싸고 흔들린다. 그래서 어느 정도 연속 검출되면
**CSRT/KCF 트래커로 전환해 ROI 만 추적**한다. control 엔 완전히 투명(더 연속적인
`/arms/detections` 를 낼 뿐). 상태 3개: **ACQUIRE → TRACK → LOST**.

트래커/opencv-contrib 가 없으면 자동으로 **매프레임 검출**로 폴백(기능 정상).

### 6.1 ACQUIRE (`_do_acquire`)

- 매 프레임 `detect_fn` 실행. 없으면 streak 리셋.
- 검출 중심이 직전과 `confirm_dist`(0.035) 이내면 **`hit_streak` 증가**, 아니면 1로 리셋.
- 중심을 **등속도 Kalman**(`_CenterKalman`)으로 스무딩.
- **CV 후보 승격 (핵심):**
  - `cv_fused` 이고 streak ≥ `cv_confirm_frames`(5) → confidence 를 `cv_confirm_conf`(0.72)로.
  - `hsv_red` 이고 streak ≥ `hsv_confirm_frames`(8) → confidence 를 `hsv_confirm_conf`(0.68)로.
  - 그 외 CV → confidence 를 0.60 으로 캡(승격 안 함).
  - 즉 **순간적인 빨간 조명/건물은 몇 프레임 못 버텨 승격 못 하고**, 진짜 표적만
    여러 프레임 같은 자리에서 합의해 임계값을 넘는다.
- `hit_streak ≥ confirm_frames`(3) 이고 트래커 init 성공하면 **TRACK 전환**.
- 반환: ACQUIRE 중에도 실검출 박스를 그대로 발행(승격된 confidence 포함).

### 6.2 TRACK (`_do_track`)

- CSRT/KCF `update` 로 ROI 추적 → 실패하면 **LOST**.
- 추적 박스도 Kalman 스무딩 + 모션(속도 EMA) 갱신.
- `redetect_interval`(10) 프레임마다 **박스 주변 ROI 크롭 재검출**(`detect_fn(roi)`)로
  드리프트 보정: 검출되면 그 박스로 재init(스냅), 안 되면 `unconfirmed++`.
- `unconfirmed ≥ max_unconfirmed`(3) 누적되면 드리프트 의심 → **LOST**.
- 발행 confidence 는 승격된 `_last_conf` 를 유지(추적 중 원래 낮은 CV 점수로 안 떨어짐).

### 6.3 LOST (`_do_lost`)

- 매 프레임 재획득 시도. 검출이 **예측 위치 근처**(Kalman/속도 외삽 + `match_dist`)면
  재init → **TRACK** 복귀.
- `reacquire_frames`(8) 안에 못 찾으면 **ACQUIRE** 로.
- LOST 중엔 **발행 안 함(None)** → control 이 마지막 표적을 홀드.

---

## 7. detector_status (UI 연동)

`/arms/detector_status` = `Float32MultiArray([yolo, hsv, absdiff, mode])`.

- 각 값: `0~1` = 그 검출기의 confidence, `-1` = off/이번 프레임 미실행, `-2` = 사용불가(YOLO OOM).
- `mode`: `0` = FULL(전체프레임 획득), `1` = ROI(추적 크롭).
- `debug.detector_status`(기본 true) 로 on/off. arms_ui 가 SEARCH/LOCK 화면 좌상단에
  세 검출기 %와 모드를 표시한다(표시 전용 — LOCK 판단에는 영향 없음).

---

## 8. 파라미터 레퍼런스

### 검출기 on/off · 처리
| 파라미터 | 기본 | 설명 |
| --- | --- | --- |
| `use_yolo` / `use_hsv` / `use_absdiff` | true | 검출기 on/off |
| `yolo.acquire_interval` | 2 | ACQUIRE/LOST 중 YOLO 를 N프레임마다 실행 |
| `yolo.proposal_crop_px` | 192 | CV 위치 확대검증 크롭 크기[px] |
| `proc_width` | 640 | 검출 처리 가로 해상도(0=원본) |
| `publish_debug` | true | 디버그 영상 발행 |

### HSV
| 파라미터 | 기본 | 설명 |
| --- | --- | --- |
| `hsv.hue_sigma` | 12.0 | 원형 hue Gaussian 폭 |
| `hsv.prob_threshold` | 0.20 | 빨강 확률 이진화 임계 |
| `hsv.sat_center` / `sat_scale` | 75 / 22 | 채도 시그모이드 |
| `hsv.val_min` / `val_max_center` / `val_scale` | 30 / 225 / 18 | 명도 게이팅 |
| `hsv.min_area_ratio` / `max_area_ratio` | 0.00003 / 0.02 | 면적비 게이트 |
| `hsv.full_conf_area_ratio` | 0.01 | confidence 포화 면적비 |
| `hsv.min_circularity` | 0.20 | 형태 점수 하한 |
| `hsv.texture_scale` | 120 | 배경 매끈함 스케일 |

### ABSDIFF
| 파라미터 | 기본 | 설명 |
| --- | --- | --- |
| `absdiff.diff_thresh` | 25 | 배경 대비 임계 |
| `absdiff.bg_blur` / `pre_blur` | 15 / 1 | 배경추정/전처리 Gaussian(홀수) |
| `absdiff.max_area_ratio` | 0.05 | blob 최대 면적비 |
| `absdiff.max_blobs` | 100 | 초과 시 장면 클러터로 보고 억제(0=끔) |
| `absdiff.min_area_ratio` | 0.00003 | blob 최소 면적비 |
| `absdiff.full_conf_area_ratio` | 0.01 | confidence 포화 면적비 |

### 융합 · 획득 승격
| 파라미터 | 기본 | 설명 |
| --- | --- | --- |
| `fusion.max_center_dist` | 0.035 | HSV↔ABSDIFF 융합 중심 허용거리 |
| `acquire.yolo_only` | false | 획득을 YOLO 전용으로(CV 배제) |
| `acquire.cv_confirm_frames` / `cv_confirm_conf` | 5 / 0.72 | cv_fused 승격 조건/값 |
| `acquire.hsv_confirm_frames` / `hsv_confirm_conf` | 8 / 0.68 | hsv_red 승격 조건/값 |
| `debug.detector_status` | true | detector_status 발행 |

### detect-then-track
| 파라미터 | 기본 | 설명 |
| --- | --- | --- |
| `track.enable` | true | detect-then-track on/off |
| `track.tracker_type` | CSRT | "CSRT" \| "KCF" |
| `track.confirm_frames` | 3 | ACQUIRE→TRACK 연속 검출 수 |
| `track.confirm_dist` | 0.035 | 연속 판정 중심거리 |
| `track.redetect_interval` | 10 | TRACK 재검출 주기 |
| `track.redetect_margin` | 2.0 | 재검출 ROI 크롭 확장배율 |
| `track.match_dist` | 0.1 | LOST 재획득 예측 게이팅 거리 |
| `track.reacquire_frames` | 8 | LOST 재획득 창 |
| `track.max_unconfirmed` | 3 | TRACK 미확인 허용(드리프트 안전장치) |
| `track.min_box_px` | 4 | 트래커 init 최소 박스 |
| `roi.margin` | 1.8 | ROI 확대 크롭 배율 |

### 환경변수 (컨테이너 compose 가 주입)
| 변수 | 기본 | 설명 |
| --- | --- | --- |
| `ARMS_MODEL` | (없음) | YOLO 가중치 경로. 없으면 YOLO 비활성 |
| `ARMS_CONF` | 0.32 | YOLO confidence 임계 |
| `ARMS_IOU` | 0.45 | YOLO NMS IoU |
| `ARMS_IMGSZ` | 320 | YOLO 입력 크기 |
| `ARMS_YOLO_RETRY_SEC` | 3.0 | 실패 후 재시도 쿨다운 |
| `ARMS_YOLO_MAX_FAILS` | 3 | 연속 실패 시 YOLO 영구 비활성 |

---

## 9. 요약 — 상태별로 무엇이 도는가

| 상황 | detect_fn 경로 | YOLO | CV | 발행 |
| --- | --- | --- | --- | --- |
| **ACQUIRE** | 전체프레임 | N프레임마다 + CV위치 확대검증 | HSV·ABSDIFF·융합(시간확인 승격) | 실검출 박스 |
| **TRACK** | CSRT/KCF 추적 + 주기적 ROI 재검출 | 재검출 때 항상 | ROI 크롭 안 | 추적 박스 |
| **LOST** | 전체프레임 재획득 | N프레임마다 | 예측근처만 채택 | 없음(홀드) |

**degradation:** YOLO 가 OOM/부재면 자동으로 **CV(HSV+ABSDIFF+융합)만으로** 동작하며,
시간 연속성 승격 로직은 그대로라 헛-LOCK 억제가 유지된다.
