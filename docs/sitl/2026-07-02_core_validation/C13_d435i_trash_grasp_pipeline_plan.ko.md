# C13 - D435i 기반 쓰레기 물체 파지 파이프라인

## 목적

Intel RealSense D435i의 color/depth를 이용해 책상 위 작은 쓰레기 물체를 찾고, 파지 후보 위치를 계산한 뒤 LeRobot 6D action sequence로 변환한다.

이 task의 핵심은 실제 파지를 바로 시도하는 것이 아니라, **camera frame에서 본 물체 위치가 robot frame의 안전한 목표점으로 바뀌는 과정**을 검증하는 것이다.

## 1. 사람용

### 사람이 이해해야 하는 것

완료 후 학생은 아래 질문에 답할 수 있어야 한다.

```text
Q1. D435i color frame과 depth frame을 왜 align해야 하는가?
Q2. depth image에서 책상과 물체를 어떻게 구분하는가?
Q3. camera 좌표계의 3D point를 robot 좌표계로 바꾸려면 무엇이 필요한가?
Q4. 파지 후보가 workspace 밖이면 왜 motion을 보내면 안 되는가?
Q5. 실제 파지 전에 synthetic test와 SITL test를 먼저 하는 이유는 무엇인가?
```

### 기준 자료

| 항목 | 기준 |
|---|---|
| camera | Intel RealSense D435i |
| frame 처리 | depth를 color frame에 align |
| Python package | optional `pyrealsense2` |
| 1차 segmentation | depth 기반 table/object separation |
| 1차 grasp | top-down centroid grasp |
| robot command | 기존 LeRobot 6D contract |

### 사람이 직접 해야 하는 부분

```text
[ ] D435i가 컴퓨터에서 인식되는지 확인한다.
[ ] 카메라가 손/팔/책상/물체를 볼 수 있게 고정한다.
[ ] calibration 파일의 camera-to-robot transform이 실제 설치와 맞는지 확인한다.
[ ] segmentation 결과 이미지 또는 mask를 눈으로 확인한다.
[ ] grasp point가 실제 물체 중심 근처인지 확인한다.
[ ] C12 parity가 blocked면 실제 motion으로 넘어가지 않는다.
```

### AI에게 도움받을 수 있는 부분

AI는 실험의 주체가 아니라 보조 도구다. 사람이 카메라 위치, 물체, calibration, 파지 전략을 정한 뒤, 필요할 때 아래 작업을 부탁할 수 있다.

```text
[ ] D435i capture 예제 코드 정리를 돕는다.
[ ] depth/color frame align 결과를 확인하는 작은 script 초안을 돕는다.
[ ] segmentation mask 저장 또는 시각화 script 초안을 돕는다.
[ ] camera point를 robot frame으로 바꾸는 계산을 검산한다.
[ ] 사람이 정한 파지 절차를 LeRobot 6D action sequence로 옮기는 것을 돕는다.
[ ] 산출물 markdown 초안을 정리한다.
```

AI에게 보조 작업을 맡길 때는 아래 문서를 참고할 수 있다.

```text
docs/sitl/2026-07-02_core_validation/ai_agent/C13_d435i_trash_grasp_pipeline_plan.ai.ko.md
```

### 진행 순서

1. 사람이 D435i와 물체 배치를 정한다.
2. 사람이 camera-to-robot calibration 방법을 정한다.
3. D435i를 연결하고 aligned frame capture만 먼저 확인한다.
4. 실제 물체를 놓고 segmentation mask를 눈으로 확인한다.
5. 사람이 grasp point가 타당한지 판단한다.
6. grasp point가 workspace 안에 있을 때 SITL action replay를 한다.
7. 실제 hardware motion은 C12 parity가 pass일 때만 별도 판단한다.

### 실행 예시

```bash
python3 <D435i_capture_script>.py --save-frame isaacsim_test/artifacts/d435i_frame
python3 <segmentation_or_grasp_script>.py --input isaacsim_test/artifacts/d435i_frame
```

위 명령의 script 이름은 사람이 선택한 구현 방식에 맞춰 정한다. 중요한 것은 자동화 자체가 아니라 frame, mask, grasp point, action sequence가 각각 검토 가능한 evidence로 남는 것이다.

### 중단 기준

```text
[ ] D435i aligned depth/color frame을 얻지 못한다.
[ ] segmentation mask가 물체가 아니라 손/팔/책상을 잡는다.
[ ] object mask가 80 pixel 미만이다.
[ ] depth 값이 0 또는 NaN이다.
[ ] camera-to-robot transform이 없다.
[ ] grasp point가 workspace 밖이다.
[ ] C12 parity가 blocked인데 real motion을 시도하려 한다.
```

### 산출물 경로

```text
docs/sitl/2026-07-02_core_validation/artifacts/C13_d435i_trash_grasp_<name>.md
```

### 보고서 템플릿

````markdown
# C13 D435i Trash Object Grasp - <name>

## 실행 정보

- Date:
- Operator:
- Camera serial:
- Calibration file:
- C12 parity status:
- Execution mode: synthetic / D435i capture / SITL / real guarded

## 사람이 확인한 것

| 항목 | 결과 | 근거 |
|---|---|---|
| aligned color/depth frame | pass/fail | image/report path |
| segmentation mask | pass/fail | image/report path |
| grasp point | pass/fail | JSON/report |
| workspace check | pass/fail | JSON/report |
| 6D action sequence | pass/fail | JSON/report |
| SITL replay | pass/fail/blocked | JSON/report |

## 판단

```text
[ ] guarded physical grasp trial로 넘어가도 된다.
[ ] physical grasp trial로 넘어가면 안 된다. 이유:
```
````

### 승인 기준

```text
[ ] synthetic perception pipeline test가 통과했다.
[ ] D435i frame capture 또는 blocker가 기록되어 있다.
[ ] 사람이 segmentation/grasp point를 눈으로 확인했다.
[ ] action sequence가 기존 6D contract를 유지한다.
[ ] 실제 motion 여부는 C12 상태와 분리해서 판단했다.
```
