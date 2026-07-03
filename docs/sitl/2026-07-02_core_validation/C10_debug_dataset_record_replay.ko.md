# C10 - Debug Dataset 10 Episode Record/Replay

## 목적

C09 dataset schema가 맞는 것을 확인한 뒤, Isaac Sim SITL에서 debug dataset 10 episode를 기록하고 replay 가능성을 검토하는 계획을 세운다.

이 task는 실제 hardware를 움직이지 않는다. 학생이 배워야 하는 핵심은 dataset을 많이 모으기 전에 작은 debug dataset으로 schema, timing, action/state alignment, replay 가능성을 먼저 확인하는 것이다.

## 1. 사람용

### 학습 목표

```text
[ ] debug dataset과 본 dataset의 차이를 설명할 수 있다.
[ ] episode, fps, single_task, repo_id의 의미를 설명할 수 있다.
[ ] record와 replay가 각각 무엇을 검증하는지 설명할 수 있다.
[ ] dataset QA report에 어떤 evidence를 남겨야 하는지 설명할 수 있다.
```

### 왜 10 episode인가

처음부터 큰 dataset을 만들면 schema 오류, frame drop, wrong action order, bad task label이 뒤늦게 발견된다. 10 episode는 작지만 반복 문제를 찾기에 충분하고, 실패해도 버리기 쉽다.

### 선행 조건

```text
[ ] C05 no-op 통과
[ ] C06 sweep 통과 또는 partial 상태가 명확히 기록됨
[ ] C09 schema check 통과
[ ] Isaac Sim SITL follower 실행 가능
[ ] 실제 hardware command 없음
```

### 기록할 dataset 기준

| 항목 | 시작값 |
|---|---|
| episodes | 10 |
| fps | 30 |
| robot type | `isaacsim_rpo_arm` |
| state/action shape | `(6,)` |
| hand policy | `amazinghand_grasp.pos` scalar |
| task text | "Teleoperate the Robotov2 right arm and AmazingHand scalar grasp in Isaac Sim SITL." |

### 실행 전 점검

```text
[ ] `observation.state`와 `action` feature names가 같다.
[ ] leader input 또는 scripted action이 SITL follower로만 간다.
[ ] camera를 record한다면 camera key와 resolution이 문서화되어 있다.
[ ] dataset repo_id가 test/debug용이다.
[ ] upload 여부가 의도대로 설정되어 있다.
```

### Record/replay 계획

실제 명령은 환경의 LeRobot CLI 버전에 맞춰 조정한다. 아래는 기록해야 할 정보의 기준이다.

```bash
python lerobot/scripts/control_robot.py \
  --robot.type=isaacsim_rpo_arm \
  --control.type=record \
  --control.repo_id=<debug_repo_id> \
  --control.single_task="Teleoperate the Robotov2 right arm and AmazingHand scalar grasp in Isaac Sim SITL." \
  --control.fps=30 \
  --control.num_episodes=10
```

Replay 또는 dataset inspection에서는 최소한 아래를 확인한다.

```text
[ ] 10 episodes가 생성되었다.
[ ] 각 episode 길이가 0이 아니다.
[ ] `observation.state` shape가 `(6,)`이다.
[ ] `action` shape가 `(6,)`이다.
[ ] NaN/inf가 없다.
[ ] action 마지막 값은 `[0.0, 1.0]` 범위다.
[ ] replay에서 feature mismatch error가 없다.
```

### 산출물 경로

```text
docs/sitl/2026-07-02_core_validation/artifacts/C10_debug_dataset_record_replay_<name>.md
```

### 보고서 템플릿

````markdown
# C10 Debug Dataset Record/Replay - <name>

## Learning Summary

- debug dataset을 먼저 만드는 이유:
- record와 replay가 각각 검증한 것:
- schema 문제가 없다고 판단한 근거:

## Dataset Metadata

| Item | Value |
|---|---|
| repo_id/path | `<value>` |
| episodes | `10` |
| fps | `30` |
| state shape | `(6,)` |
| action shape | `(6,)` |

## QA Checklist

```text
[ ] 10 episodes exist
[ ] no empty episode
[ ] no NaN/inf
[ ] feature names match C09
[ ] replay works or blocker documented
```

## Decision

```text
[ ] 본 dataset 수집 계획으로 넘어가도 된다.
[ ] 본 dataset 수집으로 넘어가면 안 된다. 이유:
```
````

### 승인 기준

```text
[ ] C09 schema 기준이 유지된다.
[ ] 10 episode 기록 또는 기록 불가 blocker가 명확하다.
[ ] replay/inspection 결과가 있다.
[ ] dataset QA report가 있다.
[ ] 실제 hardware command 없이 SITL에서만 수행했다.
```

## 2. AI Agent 내부 문서

AI agent에게 위임할 때는 아래 내부 실행 문서를 직접 전달한다.

```text
docs/sitl/2026-07-02_core_validation/ai_agent/C10_debug_dataset_record_replay.ai.ko.md
```
