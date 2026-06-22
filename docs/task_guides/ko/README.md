# RoboParty + AmazingHand + LeRobot 작업 가이드

이 디렉터리는 `docs/task_guides/`의 주요 작업 가이드를 한국어로 정리한
버전입니다. 명령어, 브랜치 이름, 파일 경로는 영어 원본과 동일하게 유지합니다.

## 관련 한국어 문서

- [전체 프로젝트 계획](../../../roleandr.ko.md)
- [LeRobot custom config 설명](../../../lerobot_custom_config_whole_arm_hand_control.ko.md)

## AI 에이전트 사용 규칙

AI 에이전트는 각 작업 가이드를 해당 브랜치의 기준 문서로 사용해야 합니다.
체크리스트를 순서대로 처리하고, 관련 없는 파일은 수정하지 않으며, 하드웨어
결과와 테스트 결과는 각 가이드에서 지정한 마크다운 로그에 기록합니다.

큰 작업은 하나의 브랜치에서 진행합니다.

```text
tasks/source-lock-inventory
tasks/so100-leader-follower-data
tasks/roboparty-arm-bringup
tasks/amazinghand-integration
tasks/lerobot-custom-robot
tasks/dataset-training-eval
tasks/validation-gates
```

작업 브랜치를 마무리하기 전에는 해당 가이드의 검증 단계를 실행하고, 체크리스트
또는 로그를 업데이트한 뒤 브랜치를 push합니다.

## 권장 순서

1. [브랜치와 저장소 워크플로](00_branch_and_repo_workflow.ko.md)
2. [소스 고정과 인벤토리](01_source_lock_and_inventory.ko.md)
3. [SO-100 리더/팔로워 데이터 워크플로](02_so100_leader_follower_data.ko.md)
4. [RoboParty 암 bring-up](03_roboparty_arm_bringup.ko.md)
5. [AmazingHand 통합](04_amazinghand_integration.ko.md)
6. [LeRobot 커스텀 로봇 래퍼](05_lerobot_custom_robot.ko.md)
7. [데이터셋, 학습, 평가](06_dataset_training_eval.ko.md)
8. [검증 게이트](07_validation_gates.ko.md)

## 프로젝트 규칙

모든 텔레오퍼레이션 경로는 같은 LeRobot action 인터페이스 뒤에 두어야 합니다.

```text
rpo_arm_j1.pos
rpo_arm_j2.pos
rpo_arm_j3.pos
rpo_arm_j4.pos
rpo_arm_j5.pos
amazinghand_grasp.pos
```

이 규칙을 지키면 텔레오퍼레이션, 데이터셋 기록, 정책 학습, 정책 추론이 같은
형식을 공유하게 됩니다.
