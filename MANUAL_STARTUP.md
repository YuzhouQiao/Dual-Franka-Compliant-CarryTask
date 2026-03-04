# 手动启动指南

**⚠️ 推荐使用自动脚本: `./start_dual_arm_task.sh`**

仅当需要单独调试各组件时使用本指南。

---

## ⚠️ 重要发现

**MoveIt2 会自动启动 MuJoCo！**

`sim_dual_moveit.launch.py` 的第223行自动包含了 MuJoCo 启动：
```python
mujoco_ros2_node = IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(
        franka_bringup_path + '/launch/sim/launch_mujoco_ros_server.launch'
    ), ...
)
```

**因此，不要单独启动 MuJoCo，否则会导致节点重复！**

---

## 正确的手动启动步骤

### 步骤 1：启动 MoveIt2（会自动启动 MuJoCo）

打开 **第一个终端**，运行：

```bash
cd ~/mujoco_franka/franka_ws
source install/setup.bash
ros2 launch franka_moveit_config sim_dual_moveit.launch.py use_rviz:=true
```

**预期输出：**
```
[mujoco_server] [Configure]: Starting Simulation in paused mode
[FrankaMjHardwareSystem] [initSim]: Robot successfully initialized!
[move_group] You can start planning now!
```

**等待约 30-40 秒** 直到看到 RViz 窗口打开并显示两个 Panda 机械臂。

---

### 步骤 2：启动任务节点

打开 **第二个终端**，运行：

## 关键等待点

| 阶段 | 最长等待时间 | 检查内容 |
|------|-----------|--------|
| 仿真启动 | 20 秒 | `robot successfully initialized` |
| MoveIt2 启动 | 15 秒 | RViz 显示机器人 + MotionPlanning 面板 |
| 总计 | 35 秒 | 系统完全就绪 |

---

## 故障排查

### 仿真卡住（第一个终端没有输出）

```bash
# Ctrl+C 停止
# 再次运行
ros2 launch franka_bringup dual_franka_sim.launch.py
```

### RViz 没有出现

```bash
# 在第二个终端中，Ctrl+C 停止
# 检查日志是否有错误
ros2 launch franka_moveit_config sim_dual_moveit.launch.py use_rviz:=true
```

### 完全重启

```bash
# 在两个终端都按 Ctrl+C
pkill -9 mujoco_node
pkill -9 ros2
sleep 3

# 重新开始第一步
```

---

## 验证系统状态

在第三个终端中，可以运行这些命令检查：

```bash
# 检查所有节点
ros2 node list

# 检查控制器
ros2 control list_controllers

# 查看关节状态
ros2 topic echo /joint_states --rate=5
```

---

## 这是最稳定的方式！

相比自动脚本，分两个终端手动启动更容易看到和排查问题。
