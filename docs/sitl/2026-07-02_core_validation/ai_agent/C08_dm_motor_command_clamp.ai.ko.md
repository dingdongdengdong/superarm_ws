# AI C08 - DM Motor Command Clamp

## 목표

DM4340P 실제 command로 넘어가기 전에 6D action clamp 정책과 hardware-free test 기준을 문서화한다. 이 task는 clamp 설계/검증 문서이며 실제 motor command를 보내지 않는다.

## 읽을 파일

```text
docs/sitl/2026-07-02_core_validation/C07_sitl_to_real_mapping_table.ko.md
docs/sitl/2026-07-02_core_validation/C03_leader_to_sitl_6d_action.ko.md
docs/sitl/2026-07-02_core_validation/C04_robotov2_urdf_joint_limit_audit.ko.md
isaacsim_test/lerobot/isaacsim_rpo_arm_robot.py
isaacsim_test/test_v2_roboparty_config.py
```

Optional if present:

```text
isaacsim_test/lerobot/rpo_arm_contract.py
isaacsim_test/test_rpo_arm_contract.py
```

## 작성할 산출물

```text
docs/sitl/2026-07-02_core_validation/artifacts/C08_dm_motor_command_clamp_<name>.md
```

## 필수 test cases

```text
[ ] 정상 6D action
[ ] 짧은 action padding
[ ] 긴 action truncation
[ ] arm joint out-of-range clamp
[ ] hand scalar lower/upper clamp
[ ] NaN/inf reject
[ ] hardware status pending이면 publish 금지
```

## 실행 가능한 검증

기존 static contract:

```bash
python3 isaacsim_test/test_v2_roboparty_config.py
```

Optional contract test가 있으면:

```bash
PYTHONPATH=. python3 isaacsim_test/test_rpo_arm_contract.py
```

## 산출물에 포함할 표

```markdown
| Case | Input | Expected | Result | Evidence |
|---|---|---|---|---|
| short action padding | `[0.1]` | 6D padded vector | `<pass/fail>` | `<test/log>` |
| long action truncation | `[0,1,2,3,4,0.5,99]` | first 6 only | `<pass/fail>` | `<test/log>` |
| hand upper clamp | `[0,0,0,0,0,2]` | last value `1.0` | `<pass/fail>` | `<test/log>` |
```

## Stop condition

```text
[ ] 실제 motor command 발생
[ ] clamp test 없이 C08 pass 보고
[ ] NaN/inf 정책 누락
[ ] C07 mapping table과 clamp 기준 불일치
[ ] hardware pending 상태인데 publish 허용
```
