# AI 보조 실행 문서

이 폴더는 핵심 검증부 task에서 AI 도움을 받을 때 참고하는 보조 지침입니다.

사람용 판단 문서는 상위 폴더의 C01-C13 문서를 사용합니다. 이 폴더의 문서는 repo 조사, 명령 출력 해석, 산출물 초안 작성, 계산 검산을 더 직접적으로 적습니다.

## Task 목록

| Task | 내부 문서 |
|---|---|
| C01 | [C01](C01_lerobot_wrapper_layout.ai.ko.md) |
| C02 | [C02](C02_leader_arm_output_shape.ai.ko.md) |
| C03 | [C03](C03_leader_to_sitl_6d_action.ai.ko.md) |
| C04 | [C04](C04_robotov2_urdf_joint_limit_audit.ai.ko.md) |
| C05 | [C05](C05_sitl_no_op_test.ai.ko.md) |
| C06 | [C06](C06_sitl_joint_sweep_test.ai.ko.md) |
| C07 | [C07](C07_sitl_to_real_mapping_table.ai.ko.md) |
| C08 | [C08](C08_dm_motor_command_clamp.ai.ko.md) |
| C09 | [C09](C09_dataset_schema_check.ai.ko.md) |
| C10 | [C10](C10_debug_dataset_record_replay.ai.ko.md) |
| C11 | [C11](C11_amazinghand_hardware_control_test_plan.ai.ko.md) |
| C12 | [C12](C12_lerobot_sim_real_motor_angle_parity_plan.ai.ko.md) |
| C13 | [C13](C13_d435i_trash_grasp_pipeline_plan.ai.ko.md) |

## 공통 금지사항

- 실제 DM4340P motor, CAN bus, AmazingHand servo에 command를 보내지 않는다.
- repo-local evidence 없이 “통과”라고 쓰지 않는다.
- screenshot-only evidence를 numerical pass/fail evidence로 과장하지 않는다.
- C02 raw output이 없으면 C03 mapping을 확정하지 않는다.
- C11-C13에서 AI는 보조 도구다. 실험 설계, 실제 장비 실행, pass/fail 판단은 사람이 한다.
