#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("dual_arm_move_test");
  
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  RCLCPP_INFO(node->get_logger(), "正在启动双臂控制节点...");

  // 1. 创建双臂规划组
  static const std::string PLANNING_GROUP = "dual_arm";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  // 2. 设置规划参数
  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setMaxAccelerationScalingFactor(0.5);
  move_group.setPlanningTime(10.0); // 给它多一点时间思考

  // 3. 打印当前关节值（调试用）
  std::vector<double> current_joints = move_group.getCurrentJointValues();
  RCLCPP_INFO(node->get_logger(), "当前关节数量: %zu", current_joints.size());

  // 4. 定义一个安全的“准备姿态”
  // Franka 标准 Ready 姿态：{0, -pi/4, 0, -3pi/4, 0, pi/2, pi/4}
  // 我们双臂有 14 个关节，前 7 个是左臂，后 7 个是右臂
  std::vector<double> target_joints;

  // 左臂 (Panda 1)
  target_joints.push_back(0.0);
  target_joints.push_back(-0.785); // -45度
  target_joints.push_back(0.0);
  target_joints.push_back(-2.356); // -135度
  target_joints.push_back(0.0);
  target_joints.push_back(1.571);  // 90度
  target_joints.push_back(0.785);  // 45度

  // 右臂 (Panda 2) - 也可以用同样的姿态，或者镜像
  target_joints.push_back(0.0);
  target_joints.push_back(-0.785);
  target_joints.push_back(0.0);
  target_joints.push_back(-2.356);
  target_joints.push_back(0.0);
  target_joints.push_back(1.571);
  target_joints.push_back(0.785);

  // 5. 设置关节目标
  move_group.setJointValueTarget(target_joints);

  // 6. 规划与执行
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (success)
  {
    RCLCPP_INFO(node->get_logger(), "✅ 规划成功！双臂准备移动到安全位置...");
    move_group.execute(my_plan);
    RCLCPP_INFO(node->get_logger(), "🎉 运动执行完毕！");
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "❌ 规划失败！请检查终端1的报错信息。");
  }

  rclcpp::shutdown();
  return 0;
}