# RoboParty + AmazingHand + LeRobot Task Guides

These guides split the project into task-sized workstreams. Use them as the
working checklist for the RoboParty 5-DOF arm, AmazingHand, LeRobot wrapper,
SO-100 proxy data workflow, and first imitation-learning baseline.

## For AI agents

These guides are written for humans and AI agents. An agent should treat each
task guide as the source of truth for one branch, follow the checklist in order,
avoid touching unrelated files, and record hardware/test results in the markdown
logs named by that guide.

Use one branch per large task:

```text
tasks/source-lock-inventory
tasks/so100-leader-follower-data
tasks/roboparty-arm-bringup
tasks/amazinghand-integration
tasks/lerobot-custom-robot
tasks/dataset-training-eval
tasks/validation-gates
```

Before finishing a task branch, the agent should run the relevant verification
steps from that guide, update the matching log/checklist, and push the branch.

## Recommended order

1. [Branch and repo workflow](00_branch_and_repo_workflow.md)
2. [Source lock and inventory](01_source_lock_and_inventory.md)
3. [SO-100 leader/follower data workflow](02_so100_leader_follower_data.md)
4. [RoboParty arm bring-up](03_roboparty_arm_bringup.md)
5. [AmazingHand integration](04_amazinghand_integration.md)
6. [LeRobot custom robot wrapper](05_lerobot_custom_robot.md)
7. [Dataset, training, and evaluation](06_dataset_training_eval.md)
8. [Validation gates](07_validation_gates.md)

## Project rule

Keep all teleoperation paths behind the same LeRobot action interface:

```text
rpo_arm_j1.pos
rpo_arm_j2.pos
rpo_arm_j3.pos
rpo_arm_j4.pos
rpo_arm_j5.pos
amazinghand_grasp.pos
```

This keeps teleoperation, dataset recording, policy training, and policy
inference aligned.
