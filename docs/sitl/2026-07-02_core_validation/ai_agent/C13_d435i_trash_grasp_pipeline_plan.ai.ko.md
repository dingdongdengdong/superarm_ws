# AI 보조 C13 - D435i 기반 쓰레기 물체 파지 파이프라인

## 역할

사람이 카메라 배치, calibration, 물체 선택, segmentation 확인, 파지 판단을 주도한다. AI는 필요한 script 초안, 로그 해석, 좌표 변환 검산, 산출물 정리를 보조한다.

## 확인할 자료

```text
docs/sitl/2026-07-02_core_validation/C13_d435i_trash_grasp_pipeline_plan.ko.md
docs/sitl/2026-07-02_core_validation/C12_lerobot_sim_real_motor_angle_parity_plan.ko.md
isaacsim_test/lerobot/rpo_arm_contract.py
isaacsim_test/lerobot/verify_lerobot_sitl.py
```

## 도울 수 있는 일

```text
[ ] D435i aligned color/depth capture script 초안 작성을 돕는다.
[ ] segmentation mask 저장/시각화 script 초안 작성을 돕는다.
[ ] camera point to robot point 변환 계산을 검산한다.
[ ] 사람이 고른 grasp point가 workspace 표와 맞는지 계산해준다.
[ ] 사람이 정한 action sequence가 6D contract를 유지하는지 확인한다.
[ ] 산출물 markdown 초안을 작성한다.
```

## 직접 하지 말 것

```text
[ ] segmentation image를 사람이 보지 않았는데 pass로 처리하지 않는다.
[ ] calibration 값을 evidence 없이 확정하지 않는다.
[ ] 실제 physical grasp trial을 자동화가 대신 승인하지 않는다.
[ ] C12 parity가 blocked인데 real motion 가능하다고 쓰지 않는다.
```

## 산출물 정리 형식

```markdown
| Evidence | 사람이 본 결과 | 파일/숫자 근거 | 다음 판단 |
|---|---|---|---|
| aligned frame |  |  |  |
| segmentation mask |  |  |  |
| grasp point |  |  |  |
| action sequence |  |  |  |
```

## 완료 보조 기준

```text
[ ] 사람이 확인한 이미지/숫자와 AI 계산을 구분했다.
[ ] 좌표 변환과 action sequence는 재검산 가능하게 남겼다.
[ ] 실제 motion 판단은 C12 상태와 사람 판단으로 분리했다.
```
