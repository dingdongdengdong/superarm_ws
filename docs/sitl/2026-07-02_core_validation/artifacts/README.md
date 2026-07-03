# 핵심 검증부 산출물 폴더

이 폴더에는 C01-C10을 수행한 사람이 남기는 markdown report를 저장합니다.

Runtime JSON, screenshot, contact sheet처럼 실행 중 생성되는 파일은 여기에 복사하지 않습니다. 그런 파일은 `isaacsim_test/artifacts/` 아래에 두고, 이 폴더의 report에서 경로만 링크합니다.

## 파일명 규칙

```text
C01_lerobot_wrapper_layout_<name>.md
C02_leader_raw_output_<name>.md
C03_leader_to_sitl_mapping_<name>.md
C04_urdf_joint_limit_audit_<name>.md
C05_sitl_no_op_test_<name>.md
C06_sitl_joint_sweep_test_<name>.md
C07_hardware_parity_checklist_<name>.md
C08_dm_motor_command_clamp_<name>.md
C09_dataset_schema_check_<name>.md
C10_debug_dataset_record_replay_<name>.md
```

`<name>`에는 작성자 이름, 날짜, 또는 팀 내부 식별자를 넣습니다.

## 판정 규칙

| 판정 | 의미 | 다음 단계 |
|---|---|---|
| `pass` | task 문서의 승인 기준을 모두 만족한다. | 다음 task로 진행 가능 |
| `fail` | 실행은 되었지만 승인 기준을 만족하지 못했다. | 원인 분석 후 같은 task 재실행 |
| `blocked` | 필요한 장치, script, runtime, 산출물이 없어 판단할 수 없다. | blocker를 해결하기 전까지 다음 task 금지 |

## report에 반드시 들어갈 내용

```text
[ ] 실행 날짜와 작성자가 있다.
[ ] 실행한 명령이 있다.
[ ] 생성된 evidence 경로가 있다.
[ ] pass/fail/blocked 판정이 있다.
[ ] 실제 DM4340P motor, CAN bus, AmazingHand servo에 command를 보내지 않았는지 기록되어 있다.
[ ] 다음 task로 넘어가도 되는지 명시되어 있다.
```
