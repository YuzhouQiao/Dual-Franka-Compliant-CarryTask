#!/bin/bash

# 双臂搬运任务 - 完整启动脚本
# 功能: 生成SRDF → 启动 MoveIt2 (自动启动MuJoCo) → 等待初始化 → 启动任务节点
# 更新: 2026-01-31 - 增强SRDF诊断，添加ACM验证

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="${WORKSPACE_DIR}/launch_logs"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
DIAG_LOG="${LOG_DIR}/diagnosis_${TIMESTAMP}.log"

# 保存 PID 到文件以便停止
PID_FILE="${WORKSPACE_DIR}/.running_pids.txt"

# 创建日志目录
mkdir -p "$LOG_DIR"

# 清理函数
cleanup_processes() {
    echo ""
    echo "正在清理所有进程..."
    if [ -f "$PID_FILE" ]; then
        while read pid; do
            if ps -p $pid > /dev/null 2>&1; then
                kill -9 $pid 2>/dev/null || true
            fi
        done < "$PID_FILE"
        rm -f "$PID_FILE"
    fi
    pkill -9 -f "ros2 launch franka_moveit_config" 2>/dev/null || true
    pkill -9 -f "ros2 launch dual_arm_carry_task" 2>/dev/null || true
    pkill -9 -f "mujoco_ros" 2>/dev/null || true
    pkill -9 -f "mujoco_node" 2>/dev/null || true
    pkill -9 -f "rviz2" 2>/dev/null || true
    pkill -9 -f "move_group" 2>/dev/null || true
    pkill -9 -f "dual_arm_carry_task" 2>/dev/null || true
    sleep 2
    echo "✓ 清理完成"
}

# 捕获 Ctrl+C 信号
trap 'echo ""; echo "检测到 Ctrl+C，正在停止所有进程..."; cleanup_processes; exit 0' INT TERM

echo "========================================"
echo "  双臂搬运任务 - 完整启动"
echo "========================================"
echo ""
echo "日志目录: $LOG_DIR"
echo "诊断日志: $DIAG_LOG"
echo ""

# 0. Source 工作空间
echo "[0/4] 加载ROS2工作空间环境..."
source "${WORKSPACE_DIR}/install/setup.bash"
echo "✓ 环境已加载"
echo ""

# 1. 清理旧进程
echo "[1/4] 清理旧进程..."
cleanup_processes
echo ""

# 2. 生成/更新静态SRDF文件（确保碰撞规则正确）
echo "[2/4] 生成静态SRDF文件（确保碰撞规则正确加载）..."
SRDF_XACRO="${WORKSPACE_DIR}/install/franka_moveit_config/share/franka_moveit_config/srdf/dual_panda.srdf.xacro"
SRDF_OUTPUT="${WORKSPACE_DIR}/install/franka_moveit_config/share/franka_moveit_config/srdf/dual_panda_generated.srdf"
SRDF_SRC_OUTPUT="${WORKSPACE_DIR}/src/multipanda_ros2/franka_moveit_config/srdf/dual_panda_generated.srdf"

if [ -f "$SRDF_XACRO" ]; then
    xacro "$SRDF_XACRO" arm_id_1:=mj_left arm_id_2:=mj_right hand_1:=true hand_2:=true > "$SRDF_OUTPUT" 2>/dev/null
    cp "$SRDF_OUTPUT" "$SRDF_SRC_OUTPUT" 2>/dev/null || true
    
    COLLISION_RULES=$(grep -c "disable_collisions" "$SRDF_OUTPUT" 2>/dev/null || echo "0")
    echo "✓ SRDF已生成: $SRDF_OUTPUT"
    echo "  碰撞禁用规则数量: $COLLISION_RULES"
    
    # 记录诊断日志
    echo "=== SRDF 诊断 [$(date)] ===" > "$DIAG_LOG"
    echo "规则数量: $COLLISION_RULES" >> "$DIAG_LOG"
    
    # 验证关键规则并记录
    MISSING_RULES=0
    echo "关键规则验证:" >> "$DIAG_LOG"
    
    # 检查相邻关节规则
    for arm in mj_left mj_right; do
        for i in 0 1 2 3 4 5 6 7; do
            j=$((i+1))
            if [ $j -le 8 ]; then
                if grep -qE "${arm}_link${i}.*${arm}_link${j}|${arm}_link${j}.*${arm}_link${i}" "$SRDF_OUTPUT"; then
                    echo "  ✓ ${arm}_link${i} <-> ${arm}_link${j}" >> "$DIAG_LOG"
                else
                    echo "  ✗ ${arm}_link${i} <-> ${arm}_link${j} 缺失!" >> "$DIAG_LOG"
                    MISSING_RULES=$((MISSING_RULES+1))
                fi
            fi
        done
    done
    
    # base_link 规则
    for arm in mj_left mj_right; do
        if grep -qE "base_link.*${arm}_link0|${arm}_link0.*base_link" "$SRDF_OUTPUT"; then
            echo "  ✓ base_link <-> ${arm}_link0" >> "$DIAG_LOG"
        else
            echo "  ✗ base_link <-> ${arm}_link0 缺失!" >> "$DIAG_LOG"
            MISSING_RULES=$((MISSING_RULES+1))
        fi
    done
    
    # 终端输出
    if grep -q "mj_right_link0.*mj_right_link1\|mj_right_link1.*mj_right_link0" "$SRDF_OUTPUT"; then
        echo "  ✓ 关键碰撞规则 (link0↔link1) 已包含"
    else
        echo "  ⚠ 警告: 未找到 link0↔link1 碰撞规则!"
    fi
    
    if grep -q "base_link.*mj_left_link0\|mj_left_link0.*base_link" "$SRDF_OUTPUT"; then
        echo "  ✓ base_link 碰撞规则已包含"
    else
        echo "  ⚠ 警告: 未找到 base_link 碰撞规则!"
    fi
    
    if [ $MISSING_RULES -gt 0 ]; then
        echo "  ⚠ 缺失 $MISSING_RULES 条关键规则，详见: $DIAG_LOG"
    fi
else
    echo "✗ 错误: 找不到SRDF xacro文件: $SRDF_XACRO"
    exit 1
fi
echo ""

# 3. 启动 MoveIt2 (自动启动 MuJoCo)
echo "[3/4] 启动 MoveIt2 和 MuJoCo..."
echo "注意: MoveIt2 会自动启动 MuJoCo 仿真"

MOVEIT_LOG="${LOG_DIR}/moveit_mujoco_${TIMESTAMP}.log"
echo "日志: $MOVEIT_LOG"

ros2 launch franka_moveit_config sim_dual_moveit.launch.py > "$MOVEIT_LOG" 2>&1 &
MOVEIT_PID=$!
echo "进程 PID: $MOVEIT_PID"
echo $MOVEIT_PID > "$PID_FILE"

# 等待 MoveIt2 和 MuJoCo 完全启动
echo "等待 MoveIt2 和 MuJoCo 完全启动 (45秒)..."
for i in {1..45}; do
    sleep 1
    printf "\r  进度: %d/45 秒" $i
done
echo ""

# 检查进程是否还在运行
if ! ps -p $MOVEIT_PID > /dev/null; then
    echo "✗ MoveIt2 启动失败，请检查日志: $MOVEIT_LOG"
    echo "最后20行日志:"
    tail -20 "$MOVEIT_LOG"
    exit 1
fi

# 检查是否有碰撞错误
if grep -q "Start state appears to be in collision" "$MOVEIT_LOG" 2>/dev/null; then
    echo "⚠ 警告: 检测到碰撞状态问题，但继续执行..."
    echo "" >> "$DIAG_LOG"
    echo "=== 碰撞警告 ===" >> "$DIAG_LOG"
    grep -A2 "Start state appears to be in collision\|collision between" "$MOVEIT_LOG" >> "$DIAG_LOG" 2>/dev/null || true
fi

# 验证 MoveIt 是否正确加载了 SRDF
echo "验证 robot_description_semantic 参数..."
echo "" >> "$DIAG_LOG"
echo "=== 运行时验证 ===" >> "$DIAG_LOG"

# 等待 move_group 节点完全启动（最多等待10秒）
for attempt in {1..10}; do
    if ros2 node list 2>/dev/null | grep -q "/move_group"; then
        break
    fi
    sleep 1
done

SRDF_PARAM=$(ros2 param get /move_group robot_description_semantic 2>/dev/null | head -1)
if echo "$SRDF_PARAM" | grep -q "String value"; then
    RUNTIME_RULES=$(ros2 param get /move_group robot_description_semantic 2>/dev/null | grep -c "disable_collisions" || echo "0")
    echo "  ✓ robot_description_semantic 已加载 ($RUNTIME_RULES 条规则)"
    echo "robot_description_semantic: 已加载 ($RUNTIME_RULES 条规则)" >> "$DIAG_LOG"
else
    echo "  ⚠ 无法验证 robot_description_semantic 参数（可能 move_group 尚未完全启动）"
    echo "robot_description_semantic: 验证失败" >> "$DIAG_LOG"
fi

echo ""
echo "✓ MoveIt2 和 MuJoCo 已启动"
echo ""

# 4. 启动任务节点
echo "[4/4] 启动双臂搬运任务节点..."
sleep 5

TASK_LOG="${LOG_DIR}/dual_arm_task_${TIMESTAMP}.log"
echo "日志: $TASK_LOG"

ros2 launch dual_arm_carry_task dual_arm_carry_task.launch.py > "$TASK_LOG" 2>&1 &
TASK_PID=$!
echo "进程 PID: $TASK_PID"
echo $TASK_PID >> "$PID_FILE"

sleep 3

# 检查任务节点是否启动
if ! ps -p $TASK_PID > /dev/null; then
    echo "✗ 任务节点启动失败，请检查日志: $TASK_LOG"
    exit 1
fi

echo ""
echo "========================================"
echo "  ✓ 所有组件已启动"
echo "========================================"
echo ""
echo "组件状态:"
echo "  • MoveIt2 + MuJoCo: PID $MOVEIT_PID"
echo "  • 任务节点: PID $TASK_PID"
echo ""
echo "日志位置:"
echo "  • MoveIt/MuJoCo: $MOVEIT_LOG"
echo "  • 任务节点: $TASK_LOG"
echo "  • 诊断日志: $DIAG_LOG"
echo ""
echo "提示:"
echo "  • 使用 'tail -f $TASK_LOG' 查看任务日志"
echo "  • 使用 'tail -f $MOVEIT_LOG' 查看 MoveIt 日志"
echo "  • 使用 'cat $DIAG_LOG' 查看诊断报告"
echo "  • 按 Ctrl+C 停止所有进程"
echo ""
echo "RViz 验证:"
echo "  • Planning Scene 中应只显示 1 个附着的铝棒（紫色）"
echo "  • 如显示 2 个铝棒，说明独立碰撞对象未移除"
echo "  • 附着物体会随机械臂一起移动并高亮显示"
echo ""
echo "手动停止命令:"
echo "  pkill -9 -f 'ros2 launch|mujoco|rviz|move_group'"
echo ""
echo "MuJoCo 控制:"
echo "  • 空格键: 暂停/继续仿真"
echo "  • Backspace: 重置仿真"
echo ""
echo "========================================"
echo "所有组件已启动，按 Ctrl+C 停止"
echo "========================================"
echo ""

# 等待用户按 Ctrl+C
wait
