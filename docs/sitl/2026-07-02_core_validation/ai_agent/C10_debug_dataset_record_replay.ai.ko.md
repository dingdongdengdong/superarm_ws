# AI C10 - Debug Dataset 10 Episode Record/Replay

## 목표

C09 schema check 뒤 Isaac Sim SITL debug dataset 10 episode record/replay 계획과 QA report를 작성한다. 실제 hardware command는 금지한다.

## 읽을 파일

```text
docs/sitl/2026-07-02_core_validation/C10_debug_dataset_record_replay.ko.md
docs/sitl/2026-07-02_core_validation/C09_dataset_schema_check.ko.md
isaacsim_test/lerobot/run_smartphone_teleop.sh
isaacsim_test/README.md
```

## 작성할 산출물

```text
docs/sitl/2026-07-02_core_validation/artifacts/C10_debug_dataset_record_replay_<name>.md
```

## Record command template

환경에 맞게 `<debug_repo_id>`만 바꾼다. 실제 실행 여부와 결과는 report에 명확히 적는다.

```bash
python lerobot/scripts/control_robot.py \
  --robot.type=isaacsim_rpo_arm \
  --control.type=record \
  --control.repo_id=<debug_repo_id> \
  --control.single_task="Teleoperate the Robotov2 right arm and AmazingHand scalar grasp in Isaac Sim SITL." \
  --control.fps=30 \
  --control.num_episodes=10
```

## QA 필수 항목

```text
[ ] 10 episodes created or blocker documented
[ ] no empty episode
[ ] observation.state shape `(6,)`
[ ] action shape `(6,)`
[ ] no NaN/inf
[ ] action last value in `[0.0, 1.0]`
[ ] replay or inspection result documented
[ ] hardware_commanded=false
```

## Status policy

```text
PASS = 10 episode record + replay/inspection + schema QA 통과
PARTIAL = record는 됐지만 replay/inspection이 부족함
BLOCKED = SITL/LeRobot/HF repo/env 문제로 record 불가
FAIL = schema mismatch, empty episode, NaN/inf, hardware command 위험
```

## Stop condition

```text
[ ] C09 schema check 실패
[ ] 실제 hardware command 필요
[ ] debug repo_id가 불명확
[ ] upload 여부가 불명확해서 외부 공개 위험
[ ] record 결과를 검토하지 않고 pass 보고
```
