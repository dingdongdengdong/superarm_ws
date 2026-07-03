# AI 보조 C12 - LeRobot Sim-Real Motor Angle Parity

## 역할

사람이 sim-real parity 실험을 설계하고 판단한다. AI는 기존 문서/코드에서 6D 순서와 mapping 근거를 찾아주고, 사람이 기록한 target/observed 값을 표나 계산 script로 정리하는 보조 역할을 한다.

## 확인할 자료

```text
docs/sitl/2026-07-02_core_validation/C12_lerobot_sim_real_motor_angle_parity_plan.ko.md
docs/sitl/2026-07-02_core_validation/C03_leader_to_sitl_6d_action.ko.md
docs/sitl/2026-07-02_core_validation/C07_sitl_to_real_mapping_table.ko.md
docs/sitl/2026-07-02_core_validation/C08_dm_motor_command_clamp.ko.md
isaacsim_test/lerobot/rpo_arm_contract.py
isaacsim_test/lerobot/verify_lerobot_sitl.py
```

## 도울 수 있는 일

```text
[ ] 6D feature 순서를 찾아 표로 정리한다.
[ ] C07 mapping 후보와 현재 code contract가 맞는지 비교한다.
[ ] 사람이 정한 target list가 너무 큰지 눈에 띄는 위험을 표시한다.
[ ] target/SITL observed/real observed 표에서 error와 sign mismatch 후보를 계산한다.
[ ] 산출물 markdown 초안을 작성한다.
```

## 직접 하지 말 것

```text
[ ] 사람이 정하지 않은 motor target을 제안 확정값처럼 쓰지 않는다.
[ ] 실제 hardware motion을 자동화가 대신 승인하지 않는다.
[ ] sign 후보를 evidence 없이 확정하지 않는다.
[ ] C11이 blocked인데 hand parity를 pass로 처리하지 않는다.
```

## 산출물 정리 형식

```markdown
| Feature | Target | SITL observed | Real observed | Error | Sign OK | 사람 판단 |
|---|---:|---:|---:|---:|---|---|
```

## 완료 보조 기준

```text
[ ] 사람이 기록한 값만 근거로 계산했다.
[ ] pass/fail/blocked 판단은 사람 판단란에 남겼다.
[ ] AI 계산은 재검산 가능하게 식이나 script를 함께 남겼다.
```
