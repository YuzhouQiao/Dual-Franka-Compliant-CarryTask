#!/bin/bash

############################################
# 环境检测脚本 - 验证系统配置
############################################

WORKSPACE_DIR="$HOME/mujoco_franka/franka_ws"

# 颜色输出
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  双臂搬运任务 - 环境检测${NC}"
echo -e "${BLUE}========================================${NC}\n"

PASS=0
FAIL=0

# 1. 检查终端模拟器
echo -e "${YELLOW}[1] 检查终端模拟器...${NC}"
if command -v xterm &> /dev/null; then
    echo -e "${GREEN}✓ xterm 可用${NC}"
    TERMINAL="xterm"
    ((PASS++))
elif command -v gnome-terminal &> /dev/null; then
    echo -e "${GREEN}✓ gnome-terminal 可用${NC}"
    TERMINAL="gnome-terminal"
    ((PASS++))
elif command -v konsole &> /dev/null; then
    echo -e "${GREEN}✓ konsole 可用${NC}"
    TERMINAL="konsole"
    ((PASS++))
else
    echo -e "${RED}✗ 未找到终端模拟器！${NC}"
    echo "  请运行: sudo apt install xterm"
    ((FAIL++))
fi
echo ""

# 2. 检查 ROS2 环境
echo -e "${YELLOW}[2] 检查 ROS2 环境...${NC}"
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}✗ ROS2 环境未加载${NC}"
    echo "  请运行: source $WORKSPACE_DIR/install/setup.bash"
    ((FAIL++))
else
    echo -e "${GREEN}✓ ROS_DISTRO: $ROS_DISTRO${NC}"
    ((PASS++))
fi
echo ""

# 3. 检查工作空间
echo -e "${YELLOW}[3] 检查工作空间...${NC}"
if [ -d "$WORKSPACE_DIR/install" ]; then
    echo -e "${GREEN}✓ 工作空间已编译: $WORKSPACE_DIR${NC}"
    ((PASS++))
else
    echo -e "${RED}✗ 工作空间未编译${NC}"
    echo "  请运行: cd $WORKSPACE_DIR && colcon build"
    ((FAIL++))
fi
echo ""

# 4. 检查关键功能包
echo -e "${YELLOW}[4] 检查关键功能包...${NC}"

PACKAGES=(
    "dual_arm_carry_task"
    "franka_bringup"
    "franka_moveit_config"
    "franka_description"
)

for pkg in "${PACKAGES[@]}"; do
    if [ -d "$WORKSPACE_DIR/install/$pkg" ]; then
        echo -e "${GREEN}  ✓ $pkg${NC}"
        ((PASS++))
    else
        echo -e "${RED}  ✗ $pkg 未安装${NC}"
        ((FAIL++))
    fi
done
echo ""

# 5. 检查启动脚本
echo -e "${YELLOW}[5] 检查启动脚本...${NC}"
SCRIPTS=(
    "start_carry_task_demo.sh"
    "start_interactive_sim.sh"
)

for script in "${SCRIPTS[@]}"; do
    if [ -x "$WORKSPACE_DIR/$script" ]; then
        echo -e "${GREEN}  ✓ $script (可执行)${NC}"
        ((PASS++))
    elif [ -f "$WORKSPACE_DIR/$script" ]; then
        echo -e "${YELLOW}  ! $script (不可执行)${NC}"
        echo "    运行: chmod +x $WORKSPACE_DIR/$script"
        ((PASS++))
    else
        echo -e "${RED}  ✗ $script 不存在${NC}"
        ((FAIL++))
    fi
done
echo ""

# 6. 检查 MuJoCo 场景文件
echo -e "${YELLOW}[6] 检查场景配置...${NC}"
OBJECTS_XML="$WORKSPACE_DIR/install/franka_description/share/franka_description/mujoco/franka/objects.xml"
if [ -f "$OBJECTS_XML" ]; then
    if grep -q "obj_rod_01" "$OBJECTS_XML"; then
        echo -e "${GREEN}✓ 铝合金杆已配置 (obj_rod_01)${NC}"
        ROD_POS=$(grep -A1 "obj_rod_01" "$OBJECTS_XML" | grep "pos=" | grep -oP 'pos="\K[^"]*')
        echo "  位置: $ROD_POS"
        ((PASS++))
    else
        echo -e "${RED}✗ 未找到 obj_rod_01${NC}"
        ((FAIL++))
    fi
else
    echo -e "${RED}✗ objects.xml 不存在${NC}"
    ((FAIL++))
fi
echo ""

# 7. 检查 Python 依赖
echo -e "${YELLOW}[7] 检查可选工具...${NC}"
if command -v rviz2 &> /dev/null; then
    echo -e "${GREEN}  ✓ RViz2 可用${NC}"
    ((PASS++))
else
    echo -e "${RED}  ✗ RViz2 未安装${NC}"
    ((FAIL++))
fi
echo ""

# 总结
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  检测结果${NC}"
echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}通过: $PASS${NC}"
echo -e "${RED}失败: $FAIL${NC}"
echo ""

if [ $FAIL -eq 0 ]; then
    echo -e "${GREEN}✅ 环境配置正确，可以启动任务！${NC}\n"
    echo -e "${BLUE}启动命令:${NC}"
    echo -e "  cd ~/mujoco_franka/franka_ws"
    echo -e "  ./start_carry_task_demo.sh"
    echo ""
    exit 0
else
    echo -e "${RED}❌ 发现 $FAIL 个问题，请先修复${NC}\n"
    exit 1
fi
