# 📁 工作区文件说明

**工作区根目录:** `~/mujoco_franka/franka_ws/`

---

## ⭐ 主要文件（按重要性排序）

### 1️⃣ 启动脚本

| 文件名 | 用途 | 推荐度 |
|--------|------|--------|
| `start_dual_arm_task.sh` | **主要启动脚本** - 启动完整系统（MoveIt2+MuJoCo+任务） | ⭐⭐⭐⭐⭐ |
| `check_environment.sh` | 环境检测工具 - 验证 ROS2/MuJoCo/依赖 | ⭐⭐⭐⭐ |
| `diagnose.sh` | 诊断工具 - 生成详细错误报告 | ⭐⭐⭐⭐ |

### 2️⃣ 文档

| 文件名 | 内容 | 推荐阅读顺序 |
|--------|------|------------|
| `README.md` | **主要文档** - 完整使用指南和常见问题 | 1️⃣ 首先阅读 |
| `DEBUG_GUIDE.md` | 调试技巧和高级诊断方法 | 3️⃣ 遇到问题时 |
| `MANUAL_STARTUP.md` | 手动分步启动说明（用于调试） | 4️⃣ 需要单独控制组件时 |
| `README_START.md` | ⚠️ 历史文档（已过时） | ❌ 不推荐 |
| `FILES_OVERVIEW.md` | 本文件 - 文件清单说明 | 2️⃣ 了解结构时 |

---

## 📂 目录结构

```
~/mujoco_franka/franka_ws/
│
├── src/                           # 源代码目录
│   └── multipanda_ros2/
│       └── dual_arm_carry_task/   # 双臂搬运任务包
│           ├── src/
│           │   └── dual_arm_carry_task.cpp  # 主任务节点（460行，状态机实现）
│           ├── launch/
│           │   └── dual_arm_carry_task.launch.py
│           ├── CMakeLists.txt
│           └── package.xml
│
├── build/                         # 编译输出（自动生成）
├── install/                       # 安装文件（自动生成）
│   └── franka_description/share/franka_description/mujoco/franka/
│       └── objects.xml            # ⚠️ 场景物体配置（铝合金杆）
│
├── log/                           # ROS2 日志（自动生成）
└── launch_logs/                   # 启动脚本日志（start_dual_arm_task.sh 生成）
    ├── moveit_mujoco_YYYYMMDD_HHMMSS.log  # MoveIt2+MuJoCo 输出
    └── task_YYYYMMDD_HHMMSS.log           # 任务节点输出
```

---

## 🗑️ 已删除的文件（历史记录）

这些文件在 2026-01-26 被清理，因为它们：
- 使用了错误的启动顺序（重复启动 MuJoCo）
- 被新的统一脚本替代
- 包含过时信息

| 文件名 | 删除原因 |
|--------|---------|
| `start_carry_task_demo.sh` | 错误地单独启动了 MuJoCo（与 MoveIt2 冲突） |
| `start_interactive_sim.sh` | 过时的启动方法 |
| `start_with_logs.sh` | 基于错误架构的日志版本 |
| `CARRY_TASK_GUIDE.md` | 包含过时的启动说明 |
| `SIMPLE_START.md` | 被 README.md 替代 |
| `PROBLEM_ANALYSIS.md` | 临时调试文档 |

**关键发现:** `sim_dual_moveit.launch.py` 第223行自动启动 MuJoCo，不需要单独启动！

---

## 🔑 关键配置文件

### 机器人描述
- `install/franka_description/share/franka_description/mujoco/franka/`
  - `scene.xml` - 主场景文件
  - `objects.xml` - ⚠️ **编辑此文件修改场景物体**
  - `dual_panda.xml` - 双臂机器人定义

### MoveIt2 配置
- `src/franka_ros2/franka_moveit_config/`
  - `config/` - 运动规划参数
  - `launch/sim_dual_moveit.launch.py` - ⚠️ **关键启动文件（内含 MuJoCo 启动）**

### 任务配置
- `src/multipanda_ros2/dual_arm_carry_task/launch/dual_arm_carry_task.launch.py`
  - 参数: `approach_height`, `grasp_height`, `lift_height`
  - 话题重映射

---

## 📝 日志文件说明

### 自动生成的日志

| 位置 | 内容 | 清理方式 |
|------|------|---------|
| `launch_logs/` | 启动脚本输出 | 手动删除或 `rm -rf launch_logs/` |
| `log/` | ROS2 内部日志 | `colcon build` 时自动清理 |
| `build/` | 编译日志 | `rm -rf build/` 后重新编译 |

### 查看实时日志

```bash
# 实时查看 MoveIt2+MuJoCo
tail -f launch_logs/moveit_mujoco_*.log

# 实时查看任务节点
tail -f launch_logs/task_*.log

# 搜索错误
grep -i error launch_logs/*.log

# 搜索警告
grep -i warn launch_logs/*.log
```

---

## 🚀 工作流程

### 正常使用流程
```
1. cd ~/mujoco_franka/franka_ws
2. source install/setup.bash
3. ./start_dual_arm_task.sh
4. 等待 50 秒
5. 观察任务执行
6. Ctrl+C 停止
```

### 修改场景物体
```
1. 编辑 install/franka_description/share/franka_description/mujoco/franka/objects.xml
2. 修改 <body name="obj_rod_01"> 的 pos 属性
3. 重新启动系统
```

### 修改任务参数
```
1. 编辑 src/multipanda_ros2/dual_arm_carry_task/src/dual_arm_carry_task.cpp
2. 修改第 98-99 行的速度缩放因子
3. colcon build --packages-select dual_arm_carry_task
4. source install/setup.bash
5. 重新启动
```

### 调试问题
```
1. ./diagnose.sh  # 生成诊断报告
2. cat diagnostic_report_*.txt  # 查看报告
3. cat launch_logs/task_*.log  # 查看详细日志
4. 参考 DEBUG_GUIDE.md
```

---

## ⚠️ 注意事项

### ❌ 不要做
- ❌ 不要单独启动 MuJoCo（`ros2 launch franka_bringup ...`）
- ❌ 不要使用已删除的旧脚本
- ❌ 不要同时运行多个启动脚本
- ❌ 不要在没有 source 的情况下运行命令

### ✅ 应该做
- ✅ 始终使用 `start_dual_arm_task.sh`
- ✅ 每次新终端都要 `source install/setup.bash`
- ✅ 遇到问题先运行 `diagnose.sh`
- ✅ 查看日志文件获取详细信息

---

## 🔄 更新记录

**2026-01-26:**
- ✅ 清理了 6 个过时文件
- ✅ 创建统一启动脚本 `start_dual_arm_task.sh`
- ✅ 更新所有文档指向新方法
- ✅ 修复重复启动 MuJoCo 的架构问题

**关键修复:** 发现 MoveIt2 自动启动 MuJoCo，无需单独启动

---

**需要帮助？** 参考 `README.md` 的常见问题部分！
