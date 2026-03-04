#!/bin/bash

############################################
# 快速诊断脚本 - 检查当前运行状态和错误
############################################

WORKSPACE_DIR="$HOME/mujoco_franka/franka_ws"
LOG_DIR="$WORKSPACE_DIR/launch_logs"

# 颜色
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  系统诊断报告${NC}"
echo -e "${BLUE}========================================${NC}\n"

# 1. 检查 ROS2 进程
echo -e "${YELLOW}[1] ROS2 进程状态${NC}"
if pgrep -f "ros2" > /dev/null; then
    echo -e "${GREEN}✓ ROS2 进程运行中${NC}"
    ps aux | grep -E "ros2|mujoco|rviz" | grep -v grep | awk '{print "  PID " $2 ": " $11 " " $12 " " $13}'
else
    echo -e "${RED}✗ 未检测到 ROS2 进程${NC}"
fi
echo ""

# 2. 检查 ROS2 节点
echo -e "${YELLOW}[2] ROS2 节点${NC}"
if [ ! -z "$ROS_DISTRO" ]; then
    NODE_LIST=$(timeout 5 ros2 node list 2>/dev/null)
    if [ $? -eq 0 ] && [ ! -z "$NODE_LIST" ]; then
        echo -e "${GREEN}✓ ROS2节点列表:${NC}"
        echo "$NODE_LIST" | sed 's/^/  /'
    else
        echo -e "${RED}✗ 无法连接到 ROS2 节点（可能正在启动或网络未就绪）${NC}"
    fi
else
    echo -e "${RED}✗ ROS2 环境未加载，运行: source $WORKSPACE_DIR/install/setup.bash${NC}"
fi
echo ""

# 3. 检查控制器
echo -e "${YELLOW}[3] 控制器状态${NC}"
if [ ! -z "$ROS_DISTRO" ]; then
    CTRL_LIST=$(timeout 5 ros2 control list_controllers 2>/dev/null)
    if [ $? -eq 0 ] && [ ! -z "$CTRL_LIST" ]; then
        echo "$CTRL_LIST"
    else
        echo -e "${RED}✗ 控制器管理器未响应（可能正在启动）${NC}"
    fi
else
    echo -e "${RED}✗ ROS2 环境未加载${NC}"
fi
echo ""

# 4. 检查最新日志
echo -e "${YELLOW}[4] 最新日志文件分析${NC}"
if [ -d "$LOG_DIR" ]; then
    LATEST_LOGS=$(ls -t $LOG_DIR/*.log 2>/dev/null | head -3)
    if [ ! -z "$LATEST_LOGS" ]; then
        echo -e "${GREEN}找到日志文件:${NC}"
        for log in $LATEST_LOGS; do
            SIZE=$(du -h "$log" | cut -f1)
            MTIME=$(stat -c %y "$log" | cut -d' ' -f1,2 | cut -d'.' -f1)
            echo -e "  - $(basename $log) (${SIZE}, 修改时间: $MTIME)"
        done
        echo ""
        
        # 分析错误
        echo -e "${YELLOW}[5] 日志错误分析${NC}"
        for log in $LATEST_LOGS; do
            echo -e "${BLUE}--- $(basename $log) ---${NC}"
            
            # 统计错误
            ERROR_COUNT=$(grep -i "\[ERROR\]" "$log" 2>/dev/null | wc -l)
            WARN_COUNT=$(grep -i "\[WARN\]" "$log" 2>/dev/null | wc -l)
            
            if [ $ERROR_COUNT -gt 0 ]; then
                echo -e "${RED}  发现 $ERROR_COUNT 个 ERROR${NC}"
                echo -e "${RED}  最近5个错误:${NC}"
                grep -i "\[ERROR\]" "$log" | tail -5 | sed 's/^/    /'
            else
                echo -e "${GREEN}  ✓ 无 ERROR${NC}"
            fi
            
            if [ $WARN_COUNT -gt 0 ]; then
                echo -e "${YELLOW}  发现 $WARN_COUNT 个 WARN${NC}"
            else
                echo -e "${GREEN}  ✓ 无 WARN${NC}"
            fi
            echo ""
        done
        
        echo -e "${BLUE}查看完整日志命令:${NC}"
        for log in $LATEST_LOGS; do
            echo -e "  tail -100 $log"
        done
    else
        echo -e "${YELLOW}未找到日志文件${NC}"
    fi
else
    echo -e "${YELLOW}日志目录不存在: $LOG_DIR${NC}"
fi
echo ""

# 5. 搜索错误
echo -e "${YELLOW}[5] 最近的错误 (最新3条)${NC}"
if [ -d "$LOG_DIR" ] && [ ! -z "$(ls $LOG_DIR/*.log 2>/dev/null)" ]; then
    ERRORS=$(grep -i "error\|fail\|exception" $LOG_DIR/*.log 2>/dev/null | tail -3)
    if [ ! -z "$ERRORS" ]; then
        echo -e "${RED}发现错误:${NC}"
        echo "$ERRORS" | while read line; do
            echo -e "  ${RED}→${NC} $line"
        done
    else
        echo -e "${GREEN}✓ 未发现明显错误${NC}"
    fi
else
    echo -e "${YELLOW}无日志文件可分析${NC}"
fi
echo ""

# 6. 搜索警告
echo -e "${YELLOW}[6] 最近的警告 (最新3条)${NC}"
if [ -d "$LOG_DIR" ] && [ ! -z "$(ls $LOG_DIR/*.log 2>/dev/null)" ]; then
    WARNINGS=$(grep -i "warn" $LOG_DIR/*.log 2>/dev/null | tail -3)
    if [ ! -z "$WARNINGS" ]; then
        echo -e "${YELLOW}发现警告:${NC}"
        echo "$WARNINGS" | while read line; do
            echo -e "  ${YELLOW}!${NC} $line"
        done
    else
        echo -e "${GREEN}✓ 未发现警告${NC}"
    fi
else
    echo -e "${YELLOW}无日志文件可分析${NC}"
fi
echo ""

# 7. 生成完整错误报告
echo -e "${YELLOW}[7] 生成完整诊断报告${NC}"
REPORT_FILE="$WORKSPACE_DIR/diagnostic_report_$(date +%Y%m%d_%H%M%S).txt"

cat > "$REPORT_FILE" << EOF
========================================
双臂搬运任务 - 诊断报告
生成时间: $(date)
========================================

1. 系统信息
-----------
ROS_DISTRO: ${ROS_DISTRO:-未设置}
工作空间: $WORKSPACE_DIR

2. 运行中的进程
----------------
$(ps aux | grep -E "ros2|mujoco|rviz" | grep -v grep)

3. ROS2 节点列表
-----------------
$(timeout 3 ros2 node list 2>&1 || echo "无法获取节点列表")

4. 控制器状态
--------------
$(timeout 3 ros2 control list_controllers 2>&1 || echo "无法获取控制器状态")

5. 最新日志中的错误
--------------------
EOF

if [ -d "$LOG_DIR" ] && [ ! -z "$(ls $LOG_DIR/*.log 2>/dev/null)" ]; then
    grep -i "error\|fail\|exception" $LOG_DIR/*.log 2>/dev/null | tail -20 >> "$REPORT_FILE" || echo "未发现错误" >> "$REPORT_FILE"
else
    echo "无日志文件" >> "$REPORT_FILE"
fi

cat >> "$REPORT_FILE" << EOF

6. 最新日志中的警告
--------------------
EOF

if [ -d "$LOG_DIR" ] && [ ! -z "$(ls $LOG_DIR/*.log 2>/dev/null)" ]; then
    grep -i "warn" $LOG_DIR/*.log 2>/dev/null | tail -20 >> "$REPORT_FILE" || echo "未发现警告" >> "$REPORT_FILE"
else
    echo "无日志文件" >> "$REPORT_FILE"
fi

echo -e "${GREEN}✓ 完整报告已保存到:${NC}"
echo -e "  $REPORT_FILE"
echo ""
echo -e "${BLUE}查看报告: cat $REPORT_FILE${NC}"
echo -e "${BLUE}或使用文本编辑器打开: gedit $REPORT_FILE &${NC}"
echo ""

# 8. 快速修复建议
echo -e "${YELLOW}[8] 常见问题快速修复${NC}"
echo -e "${BLUE}问题1: 控制器未加载${NC}"
echo -e "  → ros2 control list_controllers"
echo -e "  如果没有 joint_trajectory_controller，检查 MuJoCo 仿真是否正常启动"
echo ""
echo -e "${BLUE}问题2: MoveIt2 无法规划${NC}"
echo -e "  → 检查 planning group 名称是否正确"
echo -e "  → ros2 topic echo /move_group/monitored_planning_scene --once"
echo ""
echo -e "${BLUE}问题3: 机械臂不移动${NC}"
echo -e "  → 检查控制器和 MuJoCo 仿真的通信"
echo -e "  → ros2 topic hz /joint_states"
echo ""

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  诊断完成${NC}"
echo -e "${GREEN}========================================${NC}\n"
