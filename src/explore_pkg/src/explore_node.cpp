#include "explore_pkg/frontier_explorer.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

class ExploreNode : public rclcpp::Node
{
public:
  ExploreNode()
  : Node("explore_node"),
    explorer_(),
    tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
    tf_listener_(*tf_buffer_),
    exploration_active_(false),
    current_goal_handle_(nullptr)
  {
    // 声明参数
    this->declare_parameter<double>("exploration_radius", -1.0);
    this->declare_parameter<double>("frontier_distance_threshold", 0.5);
    this->declare_parameter<int>("min_frontier_size", 20);
    this->declare_parameter<double>("goal_timeout", 60.0);
    this->declare_parameter<double>("update_frequency", 1.0);
    this->declare_parameter<double>("stuck_threshold", 0.1);  // 卡死检测阈值（米）
    this->declare_parameter<double>("stuck_timeout", 30.0);  // 卡死超时时间（秒）
    this->declare_parameter<std::string>("map_topic", "/map");
    this->declare_parameter<std::string>("robot_base_frame", "base_footprint");
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<std::string>("nav_action_name", "navigate_to_pose");

    // 获取参数
    exploration_radius_ = this->get_parameter("exploration_radius").as_double();
    goal_timeout_ = this->get_parameter("goal_timeout").as_double();
    update_frequency_ = this->get_parameter("update_frequency").as_double();
    stuck_threshold_ = this->get_parameter("stuck_threshold").as_double();
    stuck_timeout_ = this->get_parameter("stuck_timeout").as_double();
    map_topic_ = this->get_parameter("map_topic").as_string();
    robot_base_frame_ = this->get_parameter("robot_base_frame").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    nav_action_name_ = this->get_parameter("nav_action_name").as_string();

    RCLCPP_INFO(this->get_logger(), "探索半径: %f (负数表示无限制)", exploration_radius_);
    RCLCPP_INFO(this->get_logger(), "更新频率: %f Hz", update_frequency_);
    RCLCPP_INFO(this->get_logger(), "导航action名称: %s", nav_action_name_.c_str());

    // 订阅地图
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, 10,
      std::bind(&ExploreNode::mapCallback, this, std::placeholders::_1));

    // 创建导航动作客户端
    nav_action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
      this, nav_action_name_);

    // 发布前沿点可视化
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/explore/frontiers", 10);

    // 发布建图完成度
    progress_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "/explore/progress", 10);

    // 创建定时器
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / update_frequency_)),
      std::bind(&ExploreNode::exploreTimerCallback, this));

    // 启动探索（导航服务器将在定时器中异步检查）
    exploration_active_ = true;
    nav_server_ready_ = false;
    last_position_update_time_ = this->now();
    RCLCPP_INFO(this->get_logger(), "探索节点已启动，等待导航服务器就绪...");
    RCLCPP_INFO(this->get_logger(), "卡死检测: 阈值=%.2fm, 超时=%.1fs", stuck_threshold_, stuck_timeout_);
  }

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    current_map_ = *msg;
    map_received_ = true;

    // 计算并发布建图完成度
    double progress = explorer_.calculateExplorationProgress(current_map_);
    std_msgs::msg::Float32 progress_msg;
    progress_msg.data = progress;
    progress_pub_->publish(progress_msg);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "建图完成度: %.2f%%", progress);
  }

  void exploreTimerCallback()
  {
    if (!exploration_active_ || !map_received_) {
      return;
    }

    // 异步检查导航服务器是否就绪（不阻塞）
    if (!nav_server_ready_) {
      // 使用非阻塞方式检查，但给一点时间让nav2启动
      static rclcpp::Time start_wait_time = this->now();
      auto elapsed = (this->now() - start_wait_time).seconds();
      
      // 至少等待5秒让nav2完全启动
      if (elapsed < 5.0) {
        RCLCPP_DEBUG_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "等待nav2启动中... (已等待 %.1f 秒)", elapsed);
        return;
      }
      
      if (nav_action_client_->wait_for_action_server(std::chrono::seconds(0))) {
        nav_server_ready_ = true;
        RCLCPP_INFO(this->get_logger(), "导航服务器已连接，开始探索");
      } else {
        // 检查action server是否存在
        static int check_count = 0;
        check_count++;
        if (check_count % 10 == 0) {  // 每10次检查输出一次详细信息
          RCLCPP_WARN(
            this->get_logger(), 
            "导航服务器未就绪 (已检查 %d 次，已等待 %.1f 秒，尝试连接: %s)。\n"
            "请确认：\n"
            "  1. nav2_bringup 是否已启动？\n"
            "  2. 是否已设置机器人初始位姿（2D Pose Estimate）？\n"
            "  3. 运行: ros2 action list | grep navigate_to_pose 检查action是否存在\n"
            "  4. 运行: ros2 lifecycle get /bt_navigator 检查nav2生命周期状态",
            check_count, elapsed, nav_action_name_.c_str());
        } else {
          RCLCPP_DEBUG_THROTTLE(
            this->get_logger(), *this->get_clock(), 5000,
            "等待导航服务器就绪... (已等待 %.1f 秒)", elapsed);
        }
        return;  // 导航服务器未就绪，不执行探索逻辑
      }
    }

    // 获取机器人当前位置
    geometry_msgs::msg::PoseStamped robot_pose;
    if (!getRobotPose(robot_pose)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "无法获取机器人位置");
      return;
    }

    // 检测前沿点
    std::vector<explore_pkg::Frontier> frontiers = explorer_.detectFrontiers(
      current_map_, robot_pose, exploration_radius_);

    // 可视化前沿点
    visualizeFrontiers(frontiers);

    // 检查机器人是否卡死
    if (current_goal_handle_ && checkRobotStuck(robot_pose)) {
      RCLCPP_WARN(this->get_logger(), "检测到机器人卡死，取消当前目标并选择新目标");
      nav_action_client_->async_cancel_goal(current_goal_handle_);
      current_goal_handle_ = nullptr;
      last_robot_position_ = robot_pose.pose.position;
      last_position_update_time_ = this->now();
    }

    // 如果当前没有目标或目标已完成，选择新目标
    if (!current_goal_handle_ || isGoalCompleted()) {
      if (!frontiers.empty()) {
        geometry_msgs::msg::PoseStamped goal = explorer_.selectBestFrontier(
          frontiers, robot_pose);

        if (goal.header.frame_id != "") {
          sendGoal(goal);
        } else {
          RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "未找到有效的前沿点");
        }
      } else {
        RCLCPP_INFO_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "未发现前沿点，探索可能已完成");
      }
    } else {
      // 更新机器人位置用于卡死检测
      updateRobotPosition(robot_pose);
    }
  }

  bool getRobotPose(geometry_msgs::msg::PoseStamped & pose)
  {
    try {
      // 使用最新的transform，但允许一些延迟
      geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform(
        map_frame_, robot_base_frame_, rclcpp::Time(0), std::chrono::milliseconds(100));

      pose.header.frame_id = map_frame_;
      pose.header.stamp = this->now();
      pose.pose.position.x = transform.transform.translation.x;
      pose.pose.position.y = transform.transform.translation.y;
      pose.pose.position.z = transform.transform.translation.z;
      pose.pose.orientation = transform.transform.rotation;

      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "无法获取机器人位置: %s", ex.what());
      return false;
    }
  }

  void sendGoal(const geometry_msgs::msg::PoseStamped & goal)
  {
    if (!nav_server_ready_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "导航服务器未就绪，无法发送目标");
      return;
    }

    // 如果已有目标，先取消
    if (current_goal_handle_) {
      RCLCPP_INFO(this->get_logger(), "取消旧目标，发送新目标");
      nav_action_client_->async_cancel_goal(current_goal_handle_);
      current_goal_handle_ = nullptr;
    }

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose = goal;

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::
      SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&ExploreNode::goalResponseCallback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(&ExploreNode::feedbackCallback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&ExploreNode::resultCallback, this, std::placeholders::_1);

    RCLCPP_INFO(
      this->get_logger(), "发送探索目标: (%.2f, %.2f)",
      goal.pose.position.x, goal.pose.position.y);

    nav_action_client_->async_send_goal(goal_msg, send_goal_options);
    goal_start_time_ = this->now();
    
    // 初始化卡死检测
    geometry_msgs::msg::PoseStamped robot_pose;
    if (getRobotPose(robot_pose)) {
      last_robot_position_ = robot_pose.pose.position;
      last_position_update_time_ = this->now();
    }
  }

  void goalResponseCallback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_WARN(this->get_logger(), "目标被拒绝");
      return;
    }

    current_goal_handle_ = goal_handle;
    RCLCPP_INFO(this->get_logger(), "目标已接受");
  }

  void feedbackCallback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr &,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> /*feedback*/)
  {
    // 可以在这里处理反馈信息
  }

  void resultCallback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "成功到达目标");
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_WARN(this->get_logger(), "目标被中止");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "目标被取消");
        break;
      default:
        RCLCPP_WARN(this->get_logger(), "未知结果");
        break;
    }
    current_goal_handle_ = nullptr;
  }

  bool isGoalCompleted()
  {
    if (!current_goal_handle_) {
      return true;
    }

    // 检查目标是否超时
    auto elapsed = (this->now() - goal_start_time_).seconds();
    if (elapsed > goal_timeout_) {
      RCLCPP_WARN(this->get_logger(), "目标超时（%.1f秒），取消当前目标并选择新目标", elapsed);
      nav_action_client_->async_cancel_goal(current_goal_handle_);
      current_goal_handle_ = nullptr;
      return true;
    }

    // 检查目标状态
    auto status = current_goal_handle_->get_status();
    if (status == rclcpp_action::GoalStatus::STATUS_EXECUTING) {
      return false;
    }

    // 如果目标被中止或失败，也需要选择新目标
    if (status == rclcpp_action::GoalStatus::STATUS_ABORTED ||
        status == rclcpp_action::GoalStatus::STATUS_CANCELED) {
      RCLCPP_WARN(this->get_logger(), "目标失败或被取消，选择新目标");
      current_goal_handle_ = nullptr;
      return true;
    }

    return true;
  }

  void updateRobotPosition(const geometry_msgs::msg::PoseStamped & robot_pose)
  {
    // 计算位置变化
    double dist = std::sqrt(
      std::pow(robot_pose.pose.position.x - last_robot_position_.x, 2) +
      std::pow(robot_pose.pose.position.y - last_robot_position_.y, 2));

    // 如果位置变化超过阈值，更新位置和时间
    if (dist > stuck_threshold_) {
      last_robot_position_ = robot_pose.pose.position;
      last_position_update_time_ = this->now();
    }
  }

  bool checkRobotStuck(const geometry_msgs::msg::PoseStamped & robot_pose)
  {
    if (!current_goal_handle_) {
      return false;
    }

    // 计算位置变化
    double dist = std::sqrt(
      std::pow(robot_pose.pose.position.x - last_robot_position_.x, 2) +
      std::pow(robot_pose.pose.position.y - last_robot_position_.y, 2));

    // 如果位置变化小于阈值，检查时间
    if (dist < stuck_threshold_) {
      auto stuck_duration = (this->now() - last_position_update_time_).seconds();
      if (stuck_duration > stuck_timeout_) {
        return true;  // 机器人卡死
      }
    } else {
      // 位置有变化，更新位置和时间
      last_robot_position_ = robot_pose.pose.position;
      last_position_update_time_ = this->now();
    }

    return false;
  }

  void visualizeFrontiers(const std::vector<explore_pkg::Frontier> & frontiers)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    for (size_t i = 0; i < frontiers.size(); i++) {
      const auto & frontier = frontiers[i];

      // 前沿点簇
      visualization_msgs::msg::Marker points_marker;
      points_marker.header.frame_id = map_frame_;
      points_marker.header.stamp = this->now();
      points_marker.ns = "frontier_points";
      points_marker.id = i * 2;
      points_marker.type = visualization_msgs::msg::Marker::POINTS;
      points_marker.action = visualization_msgs::msg::Marker::ADD;
      points_marker.points = frontier.points;
      points_marker.scale.x = 0.1;
      points_marker.scale.y = 0.1;
      points_marker.color.r = 1.0;
      points_marker.color.g = 0.0;
      points_marker.color.b = 0.0;
      points_marker.color.a = 0.8;
      marker_array.markers.push_back(points_marker);

      // 前沿点质心
      visualization_msgs::msg::Marker centroid_marker;
      centroid_marker.header.frame_id = map_frame_;
      centroid_marker.header.stamp = this->now();
      centroid_marker.ns = "frontier_centroids";
      centroid_marker.id = i * 2 + 1;
      centroid_marker.type = visualization_msgs::msg::Marker::SPHERE;
      centroid_marker.action = visualization_msgs::msg::Marker::ADD;
      centroid_marker.pose.position = frontier.centroid;
      centroid_marker.pose.orientation.w = 1.0;
      centroid_marker.scale.x = 0.3;
      centroid_marker.scale.y = 0.3;
      centroid_marker.scale.z = 0.3;
      centroid_marker.color.r = 0.0;
      centroid_marker.color.g = 1.0;
      centroid_marker.color.b = 0.0;
      centroid_marker.color.a = 1.0;
      marker_array.markers.push_back(centroid_marker);
    }

    marker_pub_->publish(marker_array);
  }

  explore_pkg::FrontierExplorer explorer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_action_client_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr progress_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  nav_msgs::msg::OccupancyGrid current_map_;
  bool map_received_ = false;
  bool exploration_active_;

  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr current_goal_handle_;
  rclcpp::Time goal_start_time_;

  bool nav_server_ready_;  // 导航服务器是否就绪

  double exploration_radius_;
  double goal_timeout_;
  double update_frequency_;
  double stuck_threshold_;  // 卡死检测阈值
  double stuck_timeout_;    // 卡死超时时间
  std::string map_topic_;
  std::string robot_base_frame_;
  std::string map_frame_;
  std::string nav_action_name_;

  // 卡死检测相关
  geometry_msgs::msg::Point last_robot_position_;
  rclcpp::Time last_position_update_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExploreNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

