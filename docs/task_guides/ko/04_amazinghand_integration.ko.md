# 04 - AmazingHand 통합

## 목표

AmazingHand를 먼저 안전한 scalar gripper로 장착하고 제어합니다. 8-servo raw dexterity는
이후 작업으로 남깁니다.

## 첫 control surface

LeRobot에는 feature 하나만 노출합니다.

```text
amazinghand_grasp.pos
```

의미:

```text
0.0 = open
1.0 = close / power grasp
```

AmazingHand adapter는 이 scalar를 8개 servo target으로 변환합니다.

## Mechanical tasks

```text
[ ] RoboParty wrist mounting pattern을 측정했습니다.
[ ] AmazingHand mounting pattern을 측정했습니다.
[ ] wrist-to-hand adapter를 설계했습니다.
[ ] cable exit direction을 확인했습니다.
[ ] hand mass와 wrist torque margin을 확인했습니다.
[ ] 첫 adapter를 print 또는 machining했습니다.
[ ] hand를 장착하고 neutral pose에서 collision이 없는지 확인했습니다.
```

## Electrical tasks

```text
[ ] AmazingHand servo voltage를 확인했습니다.
[ ] serial bus adapter를 확인했습니다.
[ ] 8개 servo ID를 할당 또는 검증했습니다.
[ ] serial/power wiring에 strain relief를 추가했습니다.
[ ] fuse 또는 current protection을 추가했습니다.
[ ] hand power와 serial connector를 labeling했습니다.
```

## Software smoke test

먼저 official AmazingHand Python example을 사용합니다. 이후 adapter에는 아래 세 동작만
구현합니다.

```python
connect()
set_grasp_scalar(value: float)
disconnect()
```

모든 command는 clamp합니다.

```python
value = max(0.0, min(1.0, float(value)))
```

## Servo mapping 문서

생성 파일:

```text
docs/task_guides/amazinghand_servo_map.md
```

테이블 형식:

```markdown
| Servo ID | Finger | Open target | Closed target | Safe min | Safe max | Verified |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | thumb/index/middle/ring | measured open target | measured closed target | measured safe min | measured safe max | yes/no |
```

## 완료 조건

```text
[ ] Python에서 hand가 open/close 됩니다.
[ ] servo ID를 문서화했습니다.
[ ] open/closed target을 문서화했습니다.
[ ] scalar command가 [0.0, 1.0]으로 clamp됩니다.
[ ] hand가 servo overload 없이 foam cube를 잡을 수 있습니다.
[ ] hand cable이 wrist 또는 table과 충돌하지 않습니다.
```
