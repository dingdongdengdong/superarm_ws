# 01 - 소스 고정과 인벤토리

## 목표

하드웨어 bring-up이나 LeRobot 커스텀 작업을 시작하기 전에 사용할 upstream source와
정확한 commit을 기록합니다.

## 대상 source

```text
RoboParty root:        roboto_origin/
RoboParty hardware:    roboto_origin/modules/rpo_hardware/
RoboParty deploy:      roboto_origin/modules/roboparty_deploy/
RoboParty description: roboto_origin/modules/rpo_description/
RoboParty XR teleop:   roboto_origin/modules/roboparty_xr_teleop/
AmazingHand:           AmazingHand/
LeRobot:               lerobot/
```

## 기록 명령

각 외부 repo에서 아래 명령을 실행하고 결과를 source lock 문서에 기록합니다.

```bash
git remote -v
git rev-parse HEAD
git status --short
```

기록 파일:

```text
docs/task_guides/source_lock.md
```

현재 형식:

```markdown
| Component | Remote | Commit | Local state |
| --- | --- | --- | --- |
| roboto_origin | upstream URL | exact commit hash | clean / dirty |
| AmazingHand | upstream URL | exact commit hash | clean / dirty |
| LeRobot | upstream URL | exact commit hash | clean / dirty |
```

## 인벤토리 확인

아래 경로가 존재하는지 확인합니다.

```text
roboto_origin/modules/rpo_hardware
roboto_origin/modules/roboparty_deploy
roboto_origin/modules/rpo_description
roboto_origin/modules/roboparty_xr_teleop
AmazingHand/PythonExample
lerobot/src/lerobot/robots
```

## 완료 조건

```text
[ ] 각 upstream repo의 remote URL을 기록했습니다.
[ ] 각 upstream repo의 commit hash를 기록했습니다.
[ ] dirty repo는 어떤 파일이 변경되었는지 명시했습니다.
[ ] RoboParty 하드웨어 버전이 V1.0인지 V2.0인지 기록했습니다.
[ ] 외부 repo를 dependency로 둘지 submodule로 둘지 결정했습니다.
```
