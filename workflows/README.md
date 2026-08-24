# Motion Workflows

`scripts/run_workflow.sh` 会先 source `install/setup.bash`，默认使用 `ROS_DOMAIN_ID=55`，然后启动 Python workflow runner。

## 运行

```bash
cd /home/user/joint_controller
./scripts/run_workflow.sh workflows/example_box_workflow.yaml
```

只打印动作，不真的发布 topic / 调 service：

```bash
./scripts/run_workflow.sh workflows/example_box_workflow.yaml --dry-run
```

交互控制：

- `Ctrl+C` 一次：停止当前 workflow，不再继续发布后续任务。
- `Ctrl+C` 连续两次：立即向 `/robot_poweron` 发布 `False`。
- `confirm_after: true` 的步骤完成后，需要在 runner 里按 Enter 才会继续。
- `--confirm-all` 可以让每一步都需要确认；`--no-confirm` 可以忽略 YAML 里的确认点。

## YAML 格式

关节姿态写在 `poses` 里。字段可以用 `l1` ~ `l7`、`r1` ~ `r7`：

```yaml
poses:
  全零位:
    l1: 0.0
    l2: 0.0
    l3: 0.0
    l4: 0.0
    l5: 0.0
    l6: 0.0
    l7: 0.0
    r1: 0.0
    r2: 0.0
    r3: 0.0
    r4: 0.0
    r5: 0.0
    r6: 0.0
    r7: 0.0
```

关节任务：

```yaml
- name: 运动到某个姿态
  action: joint
  pose: 全零位
  order: [lrjoint4, lrjoint2]
  confirm_after: true
  delay_after: 1.0
```

`order` 里 `lrjoint2` 表示同时写左/右 2 号关节；也可以写成分组：

```yaml
order:
  - lrjoint1
  - [lrjoint3, lrjoint4, lrjoint5, lrjoint6, lrjoint7]
  - lrjoint2
```

笛卡尔增量任务：

```yaml
- name: 上升10cm
  action: cartesian_increment
  request:
    lz: 0.10
    rz: 0.10
  vel: 0.02
  acc: 0.05
```

未写出的笛卡尔字段会自动补 `0.0`，`vel`、`acc` 默认使用 `defaults`。
