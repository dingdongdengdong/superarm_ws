# 07 - 검증 게이트

## 목표

안전하지 않거나 품질이 낮은 데이터가 학습에 들어가지 않도록 hardware bring-up,
teleoperation, recording, training, evaluation 사이에 명확한 gate를 둡니다.

## Gate A - Hardware safe to move

```text
[ ] Emergency stop이 동작합니다.
[ ] Power limit이 설정되어 있습니다.
[ ] 올바른 CAN interface를 알고 있습니다.
[ ] 올바른 motor ID를 알고 있습니다.
[ ] Joint sign을 검증했습니다.
[ ] Joint soft limit이 설정되어 있습니다.
[ ] AmazingHand가 안전하게 open/close됩니다.
[ ] 느린 움직임 중 cable collision이 없습니다.
```

Gate A 통과 전에는 데이터를 기록하지 않습니다.

## Gate B - LeRobot wrapper safe

```text
[ ] Robot type이 instantiate됩니다.
[ ] connect()와 disconnect()가 반복 동작합니다.
[ ] get_observation()이 모든 expected key를 반환합니다.
[ ] send_action()이 joint target을 clamp합니다.
[ ] send_action()이 relative movement를 clamp합니다.
[ ] send_action()이 hand scalar를 clamp합니다.
[ ] No-op action이 arm을 예상 밖으로 움직이지 않습니다.
[ ] Tiny 0.5 degree joint command가 예상 joint를 움직입니다.
```

Gate B 통과 전에는 RoboParty policy를 학습하지 않습니다.

## Gate C - Teleoperation usable

```text
[ ] Operator가 cube-to-tray trial 5개를 연속 완료할 수 있습니다.
[ ] Arm이 table이나 tray를 치지 않습니다.
[ ] Hand timing을 제어할 수 있습니다.
[ ] Camera view가 전체 task를 보여줍니다.
[ ] Reset position이 repeatable합니다.
[ ] 실패 demo를 식별할 수 있습니다.
```

Gate C 통과 전에는 baseline data를 기록하지 않습니다.

## Gate D - Dataset quality

```text
[ ] Episode count가 target과 일치합니다.
[ ] Missing camera frame이 없습니다.
[ ] Missing action key가 없습니다.
[ ] 예상 밖 action spike가 없습니다.
[ ] Task label이 일관됩니다.
[ ] 실패 episode를 제거하거나 label했습니다.
[ ] Dataset visualizer에서 video가 usable함을 확인했습니다.
```

Gate D 통과 전에는 baseline을 학습하지 않습니다.

## Gate E - Policy evaluation

```text
[ ] Evaluation은 training과 같은 fixture를 사용합니다.
[ ] Emergency stop operator가 있습니다.
[ ] 20 trial을 기록했습니다.
[ ] Success rate를 계산했습니다.
[ ] Failure label을 지정했습니다.
[ ] 다음 dataset 변경은 관찰된 failure에 기반합니다.
```

## 최소 보고 형식

생성 파일:

```text
docs/task_guides/evaluation_log.md
```

형식:

```markdown
# Evaluation Log

| Date | Dataset | Policy | Trials | Successes | Success rate | Top failure |
| --- | --- | --- | ---: | ---: | ---: | --- |
| 2026-06-22 | rpo5_ah_cube_tray_v1 | act_rpo5_ah_cube_tray_v1 | 20 | 0 | 0% | not run |
```

## 완료 조건

```text
[ ] 모든 gate에 owner가 있습니다.
[ ] Gate pass/fail을 markdown에 기록했습니다.
[ ] 검토되지 않은 debug data로 policy를 학습하지 않았습니다.
[ ] Real-robot evaluation result를 문서화했습니다.
```
