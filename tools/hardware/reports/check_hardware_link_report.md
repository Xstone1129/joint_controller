# Heavy V1 下位机双臂/升降硬件链路自检报告

- **检测时间**: 2026-09-11 10:18:18
- **检测对象**: Master0/1 双臂 14 轴 + Master2 升降 1 轴
- **检测方式**: 临时拉起专用网卡并启动 EtherLab 后只读枚举；无 ROS、无运动命令
- **配置文件**: `/home/user/joint_controller/tools/hardware/hardware_io.yaml`

## 临时网卡配置

- 完整模式：仅临时设置三条 EtherCAT 网卡为管理状态 UP。
- 不配置 IP、不加入 bridge；退出时停止临时 EtherLab 并恢复原管理状态。

| 网卡 | 进入前管理状态 | 临时操作 | 判定 |
|---|---|---|---|

| `enp3s0` | DOWN | 已临时设置 UP | ✅ |
| `enp4s0` | DOWN | 已临时设置 UP | ✅ |
| `enp5s0` | DOWN | 已临时设置 UP | ✅ |
| EtherLab 主站 | 三主站启动 | `/etc/init.d/ethercat start` 成功 | ✅ |
| EtherCAT 从站发现 | Master0/1/2 | 7/7/1 在等待窗口内达到 | ✅ |
## 1. 配置映射

| 资源 | Master | 网卡 | 期望从站 | 判定 |
|---|---:|---|---:|---|
| 左臂 | Master0 | `enp3s0` | 7 | ✅ 配置匹配 |
| 右臂 | Master1 | `enp4s0` | 7 | ✅ 配置匹配 |
| 升降 | Master2 | `enp5s0` | 1 | ✅ 配置匹配 |

## 2. EtherCAT 物理网卡

| Master | 功能 | 网卡 | 链路 | IP/bridge 约束 | 判定 |
|---:|---|---|---|---|---|
| Master0 | 左臂 | `enp3s0` | UP | 无普通 IP/无 bridge | ✅ |
| Master1 | 右臂 | `enp4s0` | UP | 无普通 IP/无 bridge | ✅ |
| Master2 | 升降 | `enp5s0` | UP | 无普通 IP/无 bridge | ✅ |

## 3. EtherCAT 主站、从站与设备节点

| Master | 功能 | 设备节点 | 期望从站 | 实际 | 判定 |
|---:|---|---|---:|---|---|
| Master0 | 左臂 | `/dev/EtherCAT0` | 7 | link=UP, slaves=7 | ✅ |

<details><summary>Master0 从站只读快照（7 行）</summary>

```text
0  4096:0  PREOP  +  EYOU_ServoModule_ECAT_V145
1  4097:0  PREOP  +  ZeroErr Driver
2  4097:1  PREOP  +  0x00000000:0x00000000
3  4097:2  ???    +  0x00000000:0x00000000
4  4097:3  ???    +  0x00000000:0x00000000
5  4097:4  ???    +  0x00000000:0x00000000
6  4097:5  ???    +  0x00000000:0x00000000
```

</details>
| Master0 从站身份 | 非零 vendor/product、可识别状态 | 5 个未知身份/状态行 | ❌ |
| Master1 | 右臂 | `/dev/EtherCAT1` | 7 | link=UP, slaves=7 | ✅ |

<details><summary>Master1 从站只读快照（7 行）</summary>

```text
0  4103:0  PREOP  +  EYOU_ServoModule_ECAT_V145
1  4104:0  PREOP  +  ZeroErr Driver
2  4105:0  PREOP  +  EYOU_ServoModule_ECAT_V145
3  4105:1  PREOP  +  0x00000000:0x00000000
4  4105:2  ???    +  0x00000000:0x00000000
5  4105:3  ???    +  0x00000000:0x00000000
6  4105:4  ???    +  0x00000000:0x00000000
```

</details>
| Master1 从站身份 | 非零 vendor/product、可识别状态 | 4 个未知身份/状态行 | ❌ |
| Master2 | 升降 | `/dev/EtherCAT2` | 1 | link=UP, slaves=1 | ✅ |

<details><summary>Master2 从站只读快照（1 行）</summary>

```text
0  0:0  PREOP  +  0x00000000:0x00000000
```

</details>
| Master2 从站身份 | 非零 vendor/product、可识别状态 | 1 个未知身份/状态行 | ❌ |

## 4. 升降从站身份与 PDO（只读）

| 项目 | 期望 | 实际 | 判定 |
|---|---|---|---|
| Master2 升降身份/PDO | 位置0 + vendor/product + PDO 可读 | EtherCAT interface: enp5s0 (up) == EtherCAT master == Master2 Phase: Idle Active: no Slaves: 1 Ethernet devices: Main: 90:b3:d5:54:30:10 (attached) Link: UP Tx frames: 1059 Tx bytes: 63540 Rx frames: 1057 Rx bytes: 63420 Tx errors: 0 Tx frame rate [1/s]: 0 0 0 Tx rate [KByte/s]: 0.0 0.0 0.0 Rx frame rate [1/s]: 0 0 0 Rx rate [KByte/s]: 0.0 0.0 0.0 Common: Tx frames: 1059 Tx bytes: 63540 Rx frames: 1057 Rx bytes: 63420 Lost frames: 2 Tx frame rate [1/s]: 0 0 0 Tx rate [KByte/s]: 0.0 0.0 0.0 Rx frame rate [1/s]: 0 0 0 Rx rate [KByte/s]: 0.0 0.0 0.0 Loss rate [1/s]: 0 0 0 Frame loss [%]: 0.0 0.0 0.0 Distributed clocks: Reference clock: None DC reference time: 0 Application time: 0 2000-01-01 00:00:00.000000000 == EtherCAT slaves == 0 0:0 PREOP + 0x00000000:0x00000000 == Selected slave PDOs == Use these confirmed launch arguments (do not copy until wiring and PDOs are checked): slave_position:=0 slave_vendor_id:=0x00000000 slave_product_code:=0x00000000 | ❌ |

<details><summary>Master2 升降身份/PDO 原始只读输出</summary>

```text
EtherCAT interface: enp5s0 (up)
== EtherCAT master ==
Master2
  Phase: Idle
  Active: no
  Slaves: 1
  Ethernet devices:
    Main: 90:b3:d5:54:30:10 (attached)
      Link: UP
      Tx frames:   1059
      Tx bytes:    63540
      Rx frames:   1057
      Rx bytes:    63420
      Tx errors:   0
      Tx frame rate [1/s]:      0      0      0
      Tx rate [KByte/s]:      0.0    0.0    0.0
      Rx frame rate [1/s]:      0      0      0
      Rx rate [KByte/s]:      0.0    0.0    0.0
    Common:
      Tx frames:   1059
      Tx bytes:    63540
      Rx frames:   1057
      Rx bytes:    63420
      Lost frames: 2
      Tx frame rate [1/s]:      0      0      0
      Tx rate [KByte/s]:      0.0    0.0    0.0
      Rx frame rate [1/s]:      0      0      0
      Rx rate [KByte/s]:      0.0    0.0    0.0
      Loss rate [1/s]:          0      0      0
      Frame loss [%]:         0.0    0.0    0.0
  Distributed clocks:
    Reference clock:   None
    DC reference time: 0
    Application time:  0
                       2000-01-01 00:00:00.000000000
== EtherCAT slaves ==
0  0:0  PREOP  +  0x00000000:0x00000000
== Selected slave PDOs ==

Use these confirmed launch arguments (do not copy until wiring and PDOs are checked):
slave_position:=0 slave_vendor_id:=0x00000000 slave_product_code:=0x00000000
```

</details>

- 临时 EtherLab 主站已停止。

- 临时网卡状态已恢复到进入脚本前的管理状态。

## 5. 诊断边界

- 只检查双臂 14 轴与升降 1 轴；不检查底盘、腰部、夹爪、相机、雷达或其它串口。
- 仅临时启停 `/etc/init.d/ethercat` 以完成枚举；不会 rescan、写 PDO、上电或使能。退出时会停止主站并恢复网卡管理状态。
- 三主站物理链路和从站数量正确，只代表 EtherCAT 枚举层通过，不等于驱动 OP 或机器人允许运动。

## 6. 汇总

- ✅ 通过: 14
- ⚠️ 警告/未检查: 0
- ❌ 失败: 4
- ⏭️ 跳过: 0
- **总判定**: ❌ 存在链路异常，请按报告排查
