# AI C03 - Leader Output을 6D SITL Action으로 변환

## 목표

C02 raw output을 근거로 leader output을 Robotov2.0 right arm + AmazingHand 6D SITL contract로 변환하는 mapping을 문서화한다.

## 읽을 파일

```text
isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py
isaacsim_test/lerobot/rpo_arm_isaacsim.yaml
isaacsim_test/lerobot/verify_lerobot_sitl.py
docs/sitl/2026-07-02_core_validation/C02_leader_arm_output_shape.ko.md
docs/sitl/2026-07-02_core_validation/ai_agent/C02_leader_arm_output_shape.ai.ko.md
```

## 경로 구분

| 목적 | 사용 경로 |
|---|---|
| verifier / policy replay / deterministic command 검증 | `IsaacSimRpoArmRobot.send_action()` |
| live leader teleop / recording | `/leader/joint_commands` 입력 + `IsaacSimRpoArmRobot.teleop_step()` |

## 구현 후보

실제 script 구현이 필요하면 다음 파일을 후보로 사용한다.

```text
isaacsim_test/lerobot/leader_to_sitl_action.py
```

초기 함수 interface 후보:

```python
def normalize_leader_to_sitl_action(raw_values: list[float]) -> list[float]:
    """Convert leader raw vector to 6D SITL action."""
```

## 기본 clamp 기준

| SITL action key | lower | upper |
|---|---:|---:|
| `right_arm_pitch_joint.pos` | -1.57 | 1.57 |
| `right_arm_roll_joint.pos` | -1.0 | 0.25 |
| `right_arm_yaw_joint.pos` | -1.57 | 1.57 |
| `right_elbow_pitch_joint.pos` | -0.6 | 1.57 |
| `right_elbow_yaw_joint.pos` | -1.57 | 1.57 |
| `amazinghand_grasp.pos` | 0.0 | 1.0 |

## 산출물 경로

```text
docs/sitl/2026-07-02_core_validation/artifacts/C03_leader_to_sitl_mapping_<name>.md
```

## 산출물 필수 내용

- C02 artifact path.
- raw output length.
- `raw leader -> /leader/joint_commands` mapping.
- `normalized 6D action -> send_action()` mapping.
- no-op example.
- tiny action example.
- hardware command 금지 문구.

## 완료 기준

```text
[ ] C02 raw output을 근거로 mapping이 작성되어 있다.
[ ] 6D output order가 명확하다.
[ ] clamp 기준이 있다.
[ ] `send_action()` 검증 경로와 `/leader/joint_commands` live teleop 경로가 분리되어 있다.
[ ] 실제 hardware command 금지가 명시되어 있다.
```
