# RoboParty + AmazingHand + LeRobot Task Guides

These guides split the project into task-sized workstreams. Use them as the
working checklist for the RoboParty 5-DOF arm, AmazingHand, LeRobot wrapper,
SO-100 proxy data workflow, and first imitation-learning baseline.

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
