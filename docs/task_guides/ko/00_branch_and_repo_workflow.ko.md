# 00 - 브랜치와 저장소 워크플로

## 목표

작업별 브랜치를 명확히 만들고, 원격 저장소에는 의도한 프로젝트 파일만 push합니다.

## 현재 저장소 상태

```text
Workspace: /workspaces/superarm_ws
Remote:    https://github.com/dingdongdengdong/superarm_ws.git
Default:   main
```

워크스페이스에는 외부 upstream clone이 있습니다.

```text
AmazingHand/
lerobot/
roboto_origin/
```

이 디렉터리들은 root 저장소의 `.gitignore`에 들어가 있으므로 실수로 staging하지
않습니다. 전체 워크스페이스에서 `git add -A`를 실행하지 말고, 필요한 파일을
명시적으로 staging합니다.

## 브랜치 규칙

문서와 기준 계획은 `main`에 둡니다. 큰 작업은 아래 브랜치에서 진행합니다.

```text
tasks/source-lock-inventory
tasks/so100-leader-follower-data
tasks/roboparty-arm-bringup
tasks/amazinghand-integration
tasks/lerobot-custom-robot
tasks/dataset-training-eval
tasks/validation-gates
```

새 작업을 시작할 때:

```bash
git switch tasks/roboparty-arm-bringup
git pull --ff-only
```

## 안전한 staging 규칙

좋은 예:

```bash
git add docs/task_guides
git add roleandr.md
git add integration_guide
```

피해야 할 예:

```bash
git add -A
```

## 외부 repo 처리 방침

```text
Option A: 외부 의존성으로 유지
  현재 권장 방식입니다. root repo에는 문서와 로컬 통합 코드만 commit합니다.

Option B: git submodule로 변환
  root repo에서 upstream commit을 재현 가능하게 관리해야 할 때 사용합니다.

Option C: 필요한 파일만 vendor
  작은 adapter 파일에만 적합합니다. 전체 upstream repo vendoring은 피합니다.
```

## 완료 조건

```text
[ ] 작업 브랜치가 로컬과 원격에 존재합니다.
[ ] 관련 파일만 staging했습니다.
[ ] commit 메시지가 작업 내용을 설명합니다.
[ ] branch가 origin에 push되었습니다.
[ ] GitHub에서 markdown 파일을 확인할 수 있습니다.
```
