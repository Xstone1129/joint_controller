# 下位机非 ROS TCP hardware_server

## 当前状态

本目录已经加入非 ROS TCP 服务、会话/lease、Direct/ROS 硬件互斥、配置事务，
以及真实 arm/lift worker。服务默认使用 `dry-run` 后端；它不会启动 EtherCAT、
IGH 驱动或电机。真实模式启动时会先按 `hardware_io.yaml` 自动配置并启动 EtherCAT，
真实 worker 只有在显式 `--real-backend` 且客户端进入 Direct 后启动。

arm worker 直接使用现有 IGH 共享内存 ABI，绝对角度限制来自下位机 URDF；lift worker
使用 `lift_ethercat_cli --command-mode` 的行命令管道，不模拟终端按键。

LIFT TCP 控制支持 `LIFT_ENABLE`、`LIFT_DISABLE`、`LIFT_HOLD`、`LIFT_JOG`、
`LIFT_HOME` 和 `LIFT_ZERO`。`LIFT_HOME` 回到当前零偏定义的逻辑 `0.0 m`，
`LIFT_ZERO` 在静止且 PDO 正常时保存当前位置为新的逻辑零点。

## 启动服务

```bash
cd /home/user/joint_controller
./tools/hardware/run_hardware_server.sh --help
./tools/hardware/run_hardware_server.sh
sudo ./tools/hardware/run_hardware_server.sh --real-backend
```

默认监听 `0.0.0.0:7447`，配置来自：

```text
/home/user/joint_controller/tools/hardware/hardware_io.yaml
```

可用环境变量覆盖：

```bash
HARDWARE_SERVER_HOST=192.168.1.60 \
HARDWARE_SERVER_PORT=7447 \
./tools/hardware/run_hardware_server.sh
```

完成带电验收后，可以安装 systemd unit；安装命令不会立即启动：

```bash
sudo install -m 0644 systemd/hardware-server.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable hardware-server.service
```

第一次带电验收前不要执行 `systemctl start hardware-server.service`。

也可以使用仓库自带控制脚本：

```bash
cd /home/user/joint_controller
./scripts/hardware_server_ctl.sh install   # 首次安装，不启动
./scripts/hardware_server_ctl.sh enable    # 可选：开机启动
./scripts/hardware_server_ctl.sh start
./scripts/hardware_server_ctl.sh status
./scripts/hardware_server_ctl.sh clients   # 当前 TCP 连接
./scripts/hardware_server_ctl.sh logs       # 最近日志
./scripts/hardware_server_ctl.sh follow     # 实时日志
./scripts/hardware_server_ctl.sh monitor    # 实时日志监控
./scripts/hardware_server_ctl.sh reload     # daemon-reload + restart，代码更新后使用
./scripts/hardware_server_ctl.sh pause     # stop 的别名
./scripts/hardware_server_ctl.sh stop
```

`restart` 和 `reload` 会断开已有 TCP 客户端；客户端必须重新连接并发送 `HELLO`。

`probe` 会发送一个 `HELLO` 帧验证协议链路：

```bash
HARDWARE_SERVER_HOST=192.168.2.20 ./scripts/hardware_server_ctl.sh probe
```

服务日志由 systemd journal 管理。连接建立、断开、请求类型、请求失败和心跳超时会写入
`journalctl -u hardware-server.service`；完整控制 payload 不会写入日志。需要查看网络层数据
时可临时使用 `sudo tcpdump -ni <控制网卡> -X 'tcp port 7447'`，诊断结束后应立即停止抓包。

真实 arm worker 的 `igh_driver` 输出按启动实例写入 `/tmp/heavy_v1_igh_driver-*.log`，
最近一次日志为 `/tmp/heavy_v1_igh_driver.latest.log`。

## 与 ROS2 的互斥

ROS 真机 `start.sh` 和 Direct 服务共用：

```text
/run/lock/junior-hardware-owner.lock
```

ROS 真机启动会持有这把锁；Direct 服务只有在进入 `ENTER_DIRECT` 时才申请这把锁。
因此服务处于监听状态时不会占用 EtherCAT，但 ROS2 真机运行期间 Direct 请求会被拒绝。

第一版不自动停止 ROS2，也不强杀 ROS 进程。切换模式必须先由操作者停止当前模式，
确认硬件已停止，再启动另一种模式。

## 配置原则

下位机的 `tools/hardware/hardware_io.yaml` 是运行时唯一权威配置。服务器支持：

```text
GET_CONFIG
GET_CONFIG_SCHEMA
BEGIN_CONFIG
PATCH_CONFIG
VALIDATE_CONFIG
COMMIT_CONFIG
ROLLBACK_CONFIG
GET_CONFIG_HISTORY
```

远程只允许修改白名单字段。升降 `min_position_m/max_position_m` 和机械臂限位 URDF
是下位机现场安全参数，不能通过 TCP 修改。提交要求当前 revision 匹配，且硬件处于 IDLE、没有 lease；
服务器会校验、备份旧 YAML、写持久提交历史，再通过 fsync 和原子替换更新配置文件。
事务归发起会话所有；TCP 断线会自动回滚未提交修改。

## 安全说明

- `dry-run` 是默认值；`--real-backend` 会启动下位机 arm/lift worker，并且只在
  通过 ROS/IGH 占用检查后进入。
- TCP 服务不会接收原始 EtherCAT 帧。
- 运动命令必须先进入 Direct 并获取唯一 lease。
- lease 心跳超过 0.75 秒没有更新时，服务器释放 lease、退出 Direct 并释放硬件锁。
- 机械臂绝对限位来自 `right_left_arm.urdf`，升降正常范围固定为下位机配置；驱动器自身
  电流/力矩保护仍是最后一道硬件保护。
- Remote hardware console 的交互式 Direct 会话可临时请求 Lift 测试范围 `[-1.0, 1.0] m`；
  该覆盖只存在于当前 worker 内存中，不写入配置。普通服务和 ROS 运行仍使用配置范围
  `[-1.0, 0.0] m`。Arm 的 URDF 限制始终生效。
- 不要在 ROS2 真机栈运行时启动任何会配置 EtherCAT 的独立工具。
- `run_hardware_server.sh --real-backend` 会在服务进程启动前自动执行 EtherCAT 配置；上位机
  客户端不发送 EtherCAT 配置。仅在已由其他受控流程完成配置时，才可设置
  `HARDWARE_SERVER_SKIP_CONFIG=1` 跳过。
- 配置 `server.tls.enabled: true` 后，服务端会强制 mTLS 客户端证书；未配置证书时只应
  在机器人专用隔离网段使用明文 TCP。

## 离线测试

```bash
cd /home/user/joint_controller
python3 -m pytest -q tools/hardware/test_network_server.py tools/hardware/test_ethercat_hardware.py
```

测试只验证协议、lease、配置事务和文件锁，不启动 ROS 或真实硬件。
