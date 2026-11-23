#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <thread>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("test_moveit");

  // MoveGroupInterface 内部会调用服务（查规划组、取当前状态等），
  // 所以节点必须有一个线程在 spin，否则构造函数就会卡住
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // 第二个参数是 SRDF 里的规划组名，必须和 MoveIt 配置里一致
  moveit::planning_interface::MoveGroupInterface arm(node, "arm");
  arm.setMaxVelocityScalingFactor(0.5);
  arm.setMaxAccelerationScalingFactor(0.5);

  auto pose = arm.getCurrentPose();
  RCLCPP_INFO(node->get_logger(), "当前末端位置: x=%.3f y=%.3f z=%.3f",
              pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

  // 命名目标：SRDF 里的 group_state
  arm.setStartStateToCurrentState();
  arm.setNamedTarget("home");

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (arm.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
  {
    RCLCPP_INFO(node->get_logger(), "规划成功，开始执行");
    arm.execute(plan);
    RCLCPP_INFO(node->get_logger(), "执行完成");
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "规划失败");
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
