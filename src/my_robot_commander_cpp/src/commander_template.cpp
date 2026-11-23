#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <example_interfaces/msg/float64_multi_array.hpp>
#include <example_interfaces/msg/bool.hpp>
#include <my_robot_interfaces/msg/pose_command.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <atomic>
#include <thread>
#include <utility>
#include <vector>

class Commander
{
public:
  explicit Commander(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  , arm_(node, "arm")
  , gripper_(node, "gripper")
  {
    arm_.setMaxVelocityScalingFactor(0.5);
    arm_.setMaxAccelerationScalingFactor(0.5);
    gripper_.setMaxVelocityScalingFactor(0.5);
    gripper_.setMaxAccelerationScalingFactor(0.5);

    joint_sub_ = node_->create_subscription<example_interfaces::msg::Float64MultiArray>(
      "joint_command", 10,
      [this](const example_interfaces::msg::Float64MultiArray::SharedPtr msg) {
        submit("joint_command", [this, msg]() { goToJointTarget(msg->data); });
      });

    gripper_sub_ = node_->create_subscription<example_interfaces::msg::Bool>(
      "open_gripper", 10,
      [this](const example_interfaces::msg::Bool::SharedPtr msg) {
        submit("open_gripper", [this, msg]() { setGripper(msg->data); });
      });

    pose_sub_ = node_->create_subscription<my_robot_interfaces::msg::PoseCommand>(
      "pose_command", 10,
      [this](const my_robot_interfaces::msg::PoseCommand::SharedPtr msg) {
        submit("pose_command", [this, msg]() {
          goToPoseTarget(msg->x, msg->y, msg->z,
                         msg->roll, msg->pitch, msg->yaw,
                         msg->cartesian_path);
        });
      });

    RCLCPP_INFO(node_->get_logger(),
      "Commander 就绪 (异步执行): /joint_command  /open_gripper  /pose_command");
  }

  ~Commander()
  {
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  // 回调只负责"接收并派发"，规划执行放进工作线程，回调立刻返回。
  // 机械臂同一时刻只能跑一条轨迹：busy_ 为真时直接丢弃新指令，
  // 而不是排队——过期的目标延迟执行比不执行更危险。
  template<typename Fn>
  void submit(const char * topic, Fn && fn)
  {
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true)) {
      RCLCPP_WARN(node_->get_logger(), "%s 已忽略：上一条指令还在执行", topic);
      return;
    }

    // 走到这里说明 busy_ 刚从 false 变成 true，上一个工作线程必然已经结束
    if (worker_.joinable()) {
      worker_.join();
    }

    worker_ = std::thread([this, fn = std::forward<Fn>(fn)]() {
        fn();
        busy_ = false;
      });
  }

  void goToJointTarget(const std::vector<double> & joints)
  {
    if (joints.size() != 6) {
      RCLCPP_ERROR(node_->get_logger(), "关节角数量必须是 6，收到 %zu 个", joints.size());
      return;
    }

    RCLCPP_INFO(node_->get_logger(), "收到关节目标，开始规划");
    arm_.setStartStateToCurrentState();
    arm_.setJointValueTarget(joints);
    planAndExecute(arm_);
  }

  void setGripper(bool open)
  {
    RCLCPP_INFO(node_->get_logger(), "收到夹爪指令: %s", open ? "张开" : "闭合");
    gripper_.setStartStateToCurrentState();
    gripper_.setNamedTarget(open ? "gripper_open" : "gripper_closed");
    planAndExecute(gripper_);
  }

  void goToPoseTarget(double x, double y, double z,
                      double roll, double pitch, double yaw,
                      bool cartesian_path)
  {
    // RPY 欧拉角 -> 四元数
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();

    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = arm_.getPlanningFrame();
    target_pose.pose.position.x = x;
    target_pose.pose.position.y = y;
    target_pose.pose.position.z = z;
    target_pose.pose.orientation.x = q.getX();
    target_pose.pose.orientation.y = q.getY();
    target_pose.pose.orientation.z = q.getZ();
    target_pose.pose.orientation.w = q.getW();

    arm_.setStartStateToCurrentState();

    if (!cartesian_path) {
      RCLCPP_INFO(node_->get_logger(), "位姿目标 (普通规划): %.2f %.2f %.2f", x, y, z);
      arm_.setPoseTarget(target_pose);
      planAndExecute(arm_);
      arm_.clearPoseTargets();
      return;
    }

    RCLCPP_INFO(node_->get_logger(), "位姿目标 (笛卡尔直线): %.2f %.2f %.2f", x, y, z);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(target_pose.pose);

    moveit_msgs::msg::RobotTrajectory trajectory;
    moveit_msgs::msg::MoveItErrorCodes error_code;

    double fraction = arm_.computeCartesianPath(
      waypoints, 0.01, 0.0, trajectory, true, &error_code);

    if (fraction >= 0.999) {
      if (arm_.execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "笛卡尔路径执行失败");
      } else {
        RCLCPP_INFO(node_->get_logger(), "完成");
      }
    } else {
      RCLCPP_WARN(node_->get_logger(), "笛卡尔路径只完成 %.1f%%", fraction * 100.0);
    }
  }

  void planAndExecute(moveit::planning_interface::MoveGroupInterface & group)
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    if (group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "规划失败");
      return;
    }

    if (group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "执行失败");
      return;
    }

    RCLCPP_INFO(node_->get_logger(), "完成");
  }

  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface arm_;
  moveit::planning_interface::MoveGroupInterface gripper_;
  rclcpp::Subscription<example_interfaces::msg::Float64MultiArray>::SharedPtr joint_sub_;
  rclcpp::Subscription<example_interfaces::msg::Bool>::SharedPtr gripper_sub_;
  rclcpp::Subscription<my_robot_interfaces::msg::PoseCommand>::SharedPtr pose_sub_;

  std::atomic<bool> busy_{false};
  std::thread worker_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("commander_template");

  // 多线程执行器：某个回调在等待时，其他话题仍能被处理
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  Commander commander(node);

  spinner.join();

  rclcpp::shutdown();
  return 0;
}
