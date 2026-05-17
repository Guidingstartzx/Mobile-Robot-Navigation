#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/marker.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/transport/Node.hh>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sdf/Element.hh>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

namespace four_wheel_car
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

double Clamp(const double value, const double lower, const double upper)
{
  return std::max(lower, std::min(upper, value));
}

double WrapToPi(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double ShortestAngleError(const double target, const double actual)
{
  return WrapToPi(target - actual);
}

template<typename T>
T GetSdfValue(
  const std::shared_ptr<const sdf::Element> & sdf,
  const std::string & name,
  const T & fallback)
{
  if (sdf && sdf->HasElement(name)) {
    return sdf->Get<T>(name);
  }
  return fallback;
}

builtin_interfaces::msg::Time RosStampFromSimTime(const std::chrono::steady_clock::duration & sim_time)
{
  const auto ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(sim_time).count();
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(ns / 1000000000);
  stamp.nanosec = static_cast<uint32_t>(ns % 1000000000);
  return stamp;
}
}  // namespace

struct JointHandle
{
  std::string name;
  gz::sim::Entity entity{gz::sim::kNullEntity};
  double target_position{0.0};
  double target_velocity{0.0};
};

struct Command
{
  double linear_x{0.0};
  double angular_z{0.0};
};

class FourWheelCarSystem:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate
{
public:
  FourWheelCarSystem() = default;

  ~FourWheelCarSystem() override
  {
    if (this->executor_ && this->ros_node_) {
      this->executor_->remove_node(this->ros_node_);
    }
  }

  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm,
    gz::sim::EventManager &) override
  {
    this->model_ = gz::sim::Model(entity);
    this->base_link_name_ = GetSdfValue<std::string>(sdf, "base_link", "base_link");
    this->odom_child_frame_ =
      GetSdfValue<std::string>(sdf, "odom_child_frame", this->base_link_name_);
    this->base_link_ = this->model_.LinkByName(ecm, this->base_link_name_);

    this->front_left_steer_.name =
      GetSdfValue<std::string>(sdf, "front_left_steer_joint", "front_left_steer_joint");
    this->front_right_steer_.name =
      GetSdfValue<std::string>(sdf, "front_right_steer_joint", "front_right_steer_joint");
    this->front_left_wheel_.name =
      GetSdfValue<std::string>(sdf, "front_left_wheel_joint", "front_left_wheel_joint");
    this->front_right_wheel_.name =
      GetSdfValue<std::string>(sdf, "front_right_wheel_joint", "front_right_wheel_joint");
    this->rear_left_wheel_.name =
      GetSdfValue<std::string>(sdf, "rear_left_wheel_joint", "rear_left_wheel_joint");
    this->rear_right_wheel_.name =
      GetSdfValue<std::string>(sdf, "rear_right_wheel_joint", "rear_right_wheel_joint");

    this->wheel_radius_ = GetSdfValue<double>(sdf, "wheel_radius", 0.10);
    this->front_x_ = GetSdfValue<double>(sdf, "front_x", 0.28);
    this->front_left_y_ = GetSdfValue<double>(sdf, "front_left_y", 0.23);
    this->front_right_y_ = GetSdfValue<double>(sdf, "front_right_y", -0.23);
    this->steer_limit_ = GetSdfValue<double>(sdf, "steer_limit", 0.75);
    this->drive_speed_limit_ = GetSdfValue<double>(sdf, "drive_speed_limit", 45.0);
    this->steer_kp_ = GetSdfValue<double>(sdf, "steer_kp", 45.0);
    this->steer_kd_ = GetSdfValue<double>(sdf, "steer_kd", 4.5);
    this->drive_kp_ = GetSdfValue<double>(sdf, "drive_kp", 1.8);
    this->drive_kd_ = GetSdfValue<double>(sdf, "drive_kd", 0.35);
    this->max_steer_torque_ = GetSdfValue<double>(sdf, "max_steer_torque", 35.0);
    this->max_drive_torque_ = GetSdfValue<double>(sdf, "max_drive_torque", 12.0);

    this->ResolveJoint(this->front_left_steer_, ecm);
    this->ResolveJoint(this->front_right_steer_, ecm);
    this->ResolveJoint(this->front_left_wheel_, ecm);
    this->ResolveJoint(this->front_right_wheel_, ecm);
    this->ResolveJoint(this->rear_left_wheel_, ecm);
    this->ResolveJoint(this->rear_right_wheel_, ecm);

    const auto cmd_vel_topic = GetSdfValue<std::string>(sdf, "cmd_vel_topic", "/cmd_vel");
    const auto odom_topic = GetSdfValue<std::string>(sdf, "odom_topic", "/odom");
    const auto imu_topic = GetSdfValue<std::string>(sdf, "imu_topic", "/imu/data");
    const auto joint_state_topic =
      GetSdfValue<std::string>(sdf, "joint_state_topic", "/joint_states");
    const auto marker_topic =
      GetSdfValue<std::string>(sdf, "marker_topic", "/com_trajectory_marker");

    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }

    this->ros_node_ = std::make_shared<rclcpp::Node>("four_wheel_car_gazebo_system");
    this->cmd_sub_ = this->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic,
      10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(this->cmd_mutex_);
        this->command_.linear_x = msg->linear.x;
        this->command_.angular_z = msg->angular.z;
      });

    this->joint_state_pub_ =
      this->ros_node_->create_publisher<sensor_msgs::msg::JointState>(joint_state_topic, 10);
    // this->clock_pub_ =
    //   this->ros_node_->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    this->odom_pub_ = this->ros_node_->create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);
    this->imu_pub_ = this->ros_node_->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 10);
    this->rviz_marker_pub_ =
      this->ros_node_->create_publisher<visualization_msgs::msg::Marker>(marker_topic, 1);
    this->tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this->ros_node_);
    this->executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    this->executor_->add_node(this->ros_node_);

    this->gz_marker_pub_ = this->gz_node_.Advertise<gz::msgs::Marker>("/marker");

    RCLCPP_INFO(
      this->ros_node_->get_logger(),
      "four_wheel_car Gazebo system configured. Subscribing to %s",
      cmd_vel_topic.c_str());
  }

  void PreUpdate(
    const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm) override
  {
    if (info.paused) {
      return;
    }

    if (this->executor_) {
      this->executor_->spin_some(std::chrono::milliseconds(0));
    }

    const double dt = std::chrono::duration<double>(info.dt).count();
    if (dt <= 0.0) {
      return;
    }

    Command cmd;
    {
      std::lock_guard<std::mutex> lock(this->cmd_mutex_);
      cmd = this->command_;
    }

    const double steer_target = this->CommonFrontSteerTarget(cmd.linear_x, cmd.angular_z);

    this->front_left_steer_.target_position = steer_target;
    this->front_right_steer_.target_position = steer_target;
    this->front_left_wheel_.target_velocity =
      this->FrontWheelDriveVelocity(cmd.linear_x, cmd.angular_z, this->front_left_y_, steer_target);
    this->front_right_wheel_.target_velocity =
      this->FrontWheelDriveVelocity(cmd.linear_x, cmd.angular_z, this->front_right_y_, steer_target);
    this->front_left_wheel_.target_position =
      WrapToPi(this->front_left_wheel_.target_position + this->front_left_wheel_.target_velocity * dt);
    this->front_right_wheel_.target_position =
      WrapToPi(this->front_right_wheel_.target_position + this->front_right_wheel_.target_velocity * dt);

    this->ApplySteeringPd(this->front_left_steer_, ecm);
    this->ApplySteeringPd(this->front_right_steer_, ecm);
    this->ApplyDrivePd(this->front_left_wheel_, ecm);
    this->ApplyDrivePd(this->front_right_wheel_, ecm);
  }

  void PostUpdate(
    const gz::sim::UpdateInfo & info,
    const gz::sim::EntityComponentManager & ecm) override
  {
    if (info.paused || this->base_link_ == gz::sim::kNullEntity) {
      return;
    }

    const double sim_time = std::chrono::duration<double>(info.simTime).count();
    if (sim_time - this->last_pub_time_ < this->publish_period_) {
      return;
    }
    const double dt = std::max(1.0e-6, sim_time - this->last_pub_time_);
    this->last_pub_time_ = sim_time;

    const auto stamp = RosStampFromSimTime(info.simTime);
    const auto pose = gz::sim::worldPose(this->base_link_, ecm);
    const auto pos = pose.Pos();
    const auto rot = pose.Rot();

    // rosgraph_msgs::msg::Clock clock;
    // clock.clock = stamp;
    // this->clock_pub_->publish(clock);

    gz::math::Vector3d world_linear_velocity{0.0, 0.0, 0.0};
    double yaw_rate = 0.0;
    if (this->have_prev_pose_) {
      world_linear_velocity = (pos - this->prev_position_) / dt;
      yaw_rate = ShortestAngleError(rot.Yaw(), this->prev_yaw_) / dt;
    }
    const auto body_linear_velocity = rot.Inverse().RotateVector(world_linear_velocity);
    const auto body_linear_acceleration = (body_linear_velocity - this->prev_body_velocity_) / dt;

    this->PublishJointStates(stamp, ecm);
    this->PublishOdometry(stamp, pose, body_linear_velocity, yaw_rate);
    this->PublishImu(stamp, rot, body_linear_acceleration, yaw_rate);
    this->PublishTrajectory(stamp, pos);

    this->prev_position_ = pos;
    this->prev_yaw_ = rot.Yaw();
    this->prev_body_velocity_ = body_linear_velocity;
    this->have_prev_pose_ = true;
  }

private:
  void ResolveJoint(JointHandle & handle, gz::sim::EntityComponentManager & ecm)
  {
    handle.entity = this->model_.JointByName(ecm, handle.name);
    if (handle.entity == gz::sim::kNullEntity) {
      if (this->ros_node_) {
        RCLCPP_ERROR(this->ros_node_->get_logger(), "Joint not found: %s", handle.name.c_str());
      }
      return;
    }
    if (!ecm.Component<gz::sim::components::JointPosition>(handle.entity)) {
      ecm.CreateComponent(handle.entity, gz::sim::components::JointPosition());
    }
    if (!ecm.Component<gz::sim::components::JointVelocity>(handle.entity)) {
      ecm.CreateComponent(handle.entity, gz::sim::components::JointVelocity());
    }
  }

  double CommonFrontSteerTarget(
    const double center_vx,
    const double center_wz) const
  {
    constexpr double kMinLinearForCurvature = 0.03;
    constexpr double kMinAngularForSteering = 0.01;

    double steer = 0.0;
    if (std::abs(center_vx) > kMinLinearForCurvature) {
      double wheelbase = 2.0 * this->front_x_;
      steer = std::atan((center_wz * wheelbase) / center_vx);
    } else if (std::abs(center_wz) > kMinAngularForSteering) {
      steer = std::copysign(this->steer_limit_, center_wz);
    }

    return Clamp(steer, -this->steer_limit_, this->steer_limit_);
  }

  double FrontWheelDriveVelocity(
    const double center_vx,
    const double center_wz,
    const double wheel_y,
    const double steer) const
  {
    const double wheel_vx = center_vx - center_wz * wheel_y;
    const double wheel_vy = center_wz * this->front_x_;
    const double wheel_linear_speed =
      wheel_vx * std::cos(steer) + wheel_vy * std::sin(steer);

    return Clamp(
      wheel_linear_speed / this->wheel_radius_,
      -this->drive_speed_limit_,
      this->drive_speed_limit_);
  }

  double JointPosition(const JointHandle & handle, const gz::sim::EntityComponentManager & ecm) const
  {
    const auto comp = ecm.Component<gz::sim::components::JointPosition>(handle.entity);
    if (comp && !comp->Data().empty()) {
      return comp->Data()[0];
    }
    return 0.0;
  }

  double JointVelocity(const JointHandle & handle, const gz::sim::EntityComponentManager & ecm) const
  {
    const auto comp = ecm.Component<gz::sim::components::JointVelocity>(handle.entity);
    if (comp && !comp->Data().empty()) {
      return comp->Data()[0];
    }
    return 0.0;
  }

  void ApplyForce(
    const JointHandle & handle,
    const double torque,
    gz::sim::EntityComponentManager & ecm) const
  {
    if (handle.entity == gz::sim::kNullEntity) {
      return;
    }
    auto force = ecm.Component<gz::sim::components::JointForceCmd>(handle.entity);
    if (!force) {
      ecm.CreateComponent(handle.entity, gz::sim::components::JointForceCmd({torque}));
      return;
    }
    auto & data = force->Data();
    if (data.empty()) {
      data.resize(1, 0.0);
    }
    data[0] = torque;
  }

  void ApplySteeringPd(JointHandle & handle, gz::sim::EntityComponentManager & ecm) const
  {
    const double position = this->JointPosition(handle, ecm);
    const double velocity = this->JointVelocity(handle, ecm);
    const double position_error = ShortestAngleError(handle.target_position, position);
    const double velocity_error = -velocity;
    const double torque = Clamp(
      this->steer_kp_ * position_error + this->steer_kd_ * velocity_error,
      -this->max_steer_torque_,
      this->max_steer_torque_);
    this->ApplyForce(handle, torque, ecm);
  }

  void ApplyDrivePd(JointHandle & handle, gz::sim::EntityComponentManager & ecm) const
  {
    const double position = WrapToPi(this->JointPosition(handle, ecm));
    const double velocity = this->JointVelocity(handle, ecm);
    const double position_error = ShortestAngleError(handle.target_position, position);
    const double velocity_error = handle.target_velocity - velocity;
    const double torque = Clamp(
      this->drive_kp_ * position_error + this->drive_kd_ * velocity_error,
      -this->max_drive_torque_,
      this->max_drive_torque_);
    this->ApplyForce(handle, torque, ecm);
  }

  void PublishJointStates(
    const builtin_interfaces::msg::Time & stamp,
    const gz::sim::EntityComponentManager & ecm) const
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = stamp;
    const std::vector<JointHandle> joints{
      this->front_left_steer_,
      this->front_right_steer_,
      this->front_left_wheel_,
      this->front_right_wheel_,
      this->rear_left_wheel_,
      this->rear_right_wheel_};

    msg.name.reserve(joints.size());
    msg.position.reserve(joints.size());
    msg.velocity.reserve(joints.size());
    for (const auto & joint : joints) {
      msg.name.push_back(joint.name);
      msg.position.push_back(this->JointPosition(joint, ecm));
      msg.velocity.push_back(this->JointVelocity(joint, ecm));
    }
    this->joint_state_pub_->publish(msg);
  }

  void PublishOdometry(
    const builtin_interfaces::msg::Time & stamp,
    const gz::math::Pose3d & pose,
    const gz::math::Vector3d & body_linear_velocity,
    const double yaw_rate) const
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id = this->odom_child_frame_;
    odom.pose.pose.position.x = pose.Pos().X();
    odom.pose.pose.position.y = pose.Pos().Y();
    odom.pose.pose.position.z = pose.Pos().Z();
    odom.pose.pose.orientation.x = pose.Rot().X();
    odom.pose.pose.orientation.y = pose.Rot().Y();
    odom.pose.pose.orientation.z = pose.Rot().Z();
    odom.pose.pose.orientation.w = pose.Rot().W();
    odom.twist.twist.linear.x = body_linear_velocity.X();
    odom.twist.twist.linear.y = body_linear_velocity.Y();
    odom.twist.twist.linear.z = body_linear_velocity.Z();
    odom.twist.twist.angular.z = yaw_rate;
    this->odom_pub_->publish(odom);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = "odom";
    tf.child_frame_id = this->odom_child_frame_;
    tf.transform.translation.x = pose.Pos().X();
    tf.transform.translation.y = pose.Pos().Y();
    tf.transform.translation.z = pose.Pos().Z();
    tf.transform.rotation = odom.pose.pose.orientation;
    this->tf_broadcaster_->sendTransform(tf);
  }

  void PublishImu(
    const builtin_interfaces::msg::Time & stamp,
    const gz::math::Quaterniond & orientation,
    const gz::math::Vector3d & body_linear_acceleration,
    const double yaw_rate) const
  {
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = "imu_link";
    imu.orientation.x = orientation.X();
    imu.orientation.y = orientation.Y();
    imu.orientation.z = orientation.Z();
    imu.orientation.w = orientation.W();
    imu.angular_velocity.z = yaw_rate;
    imu.linear_acceleration.x = body_linear_acceleration.X();
    imu.linear_acceleration.y = body_linear_acceleration.Y();
    imu.linear_acceleration.z = body_linear_acceleration.Z();
    imu.orientation_covariance[0] = 0.01;
    imu.orientation_covariance[4] = 0.01;
    imu.orientation_covariance[8] = 0.01;
    imu.angular_velocity_covariance[0] = 0.02;
    imu.angular_velocity_covariance[4] = 0.02;
    imu.angular_velocity_covariance[8] = 0.02;
    imu.linear_acceleration_covariance[0] = 0.05;
    imu.linear_acceleration_covariance[4] = 0.05;
    imu.linear_acceleration_covariance[8] = 0.05;
    this->imu_pub_->publish(imu);
  }

  void PublishTrajectory(
    const builtin_interfaces::msg::Time & stamp,
    const gz::math::Vector3d & com_position)
  {
    if (this->trajectory_.empty() ||
      this->trajectory_.back().Distance(com_position) >= this->trajectory_min_spacing_)
    {
      this->trajectory_.push_back(com_position);
      if (this->trajectory_.size() > this->max_trajectory_points_) {
        this->trajectory_.erase(this->trajectory_.begin());
      }
    }

    visualization_msgs::msg::Marker ros_marker;
    ros_marker.header.stamp = stamp;
    ros_marker.header.frame_id = "odom";
    ros_marker.ns = "four_wheel_car_com";
    ros_marker.id = 0;
    ros_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    ros_marker.action = visualization_msgs::msg::Marker::ADD;
    ros_marker.scale.x = 0.035;
    ros_marker.color.r = 1.0;
    ros_marker.color.g = 0.35;
    ros_marker.color.b = 0.02;
    ros_marker.color.a = 1.0;
    ros_marker.points.reserve(this->trajectory_.size());

    gz::msgs::Marker gz_marker;
    gz_marker.set_ns("four_wheel_car_com");
    gz_marker.set_id(0);
    gz_marker.set_action(gz::msgs::Marker::ADD_MODIFY);
    gz_marker.set_type(gz::msgs::Marker::LINE_STRIP);
    gz_marker.set_visibility(gz::msgs::Marker::ALL);
    gz_marker.mutable_scale()->set_x(0.035);
    gz_marker.mutable_scale()->set_y(0.035);
    gz_marker.mutable_scale()->set_z(0.035);
    gz_marker.mutable_material()->mutable_diffuse()->set_r(1.0);
    gz_marker.mutable_material()->mutable_diffuse()->set_g(0.35);
    gz_marker.mutable_material()->mutable_diffuse()->set_b(0.02);
    gz_marker.mutable_material()->mutable_diffuse()->set_a(1.0);

    for (const auto & point : this->trajectory_) {
      geometry_msgs::msg::Point ros_point;
      ros_point.x = point.X();
      ros_point.y = point.Y();
      ros_point.z = point.Z();
      ros_marker.points.push_back(ros_point);

      auto * gz_point = gz_marker.add_point();
      gz_point->set_x(point.X());
      gz_point->set_y(point.Y());
      gz_point->set_z(point.Z());
    }

    this->rviz_marker_pub_->publish(ros_marker);
    this->gz_marker_pub_.Publish(gz_marker);
  }

  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Entity base_link_{gz::sim::kNullEntity};
  std::string base_link_name_{"base_link"};
  std::string odom_child_frame_{"base_link"};

  JointHandle front_left_steer_;
  JointHandle front_right_steer_;
  JointHandle front_left_wheel_;
  JointHandle front_right_wheel_;
  JointHandle rear_left_wheel_;
  JointHandle rear_right_wheel_;

  double wheel_radius_{0.10};
  double front_x_{0.28};
  double front_left_y_{0.23};
  double front_right_y_{-0.23};
  double steer_limit_{0.75};
  double drive_speed_limit_{45.0};
  double steer_kp_{45.0};
  double steer_kd_{4.5};
  double drive_kp_{1.8};
  double drive_kd_{0.35};
  double max_steer_torque_{35.0};
  double max_drive_torque_{12.0};

  mutable std::mutex cmd_mutex_;
  Command command_;

  rclcpp::Node::SharedPtr ros_node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_marker_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;

  gz::transport::Node gz_node_;
  gz::transport::Node::Publisher gz_marker_pub_;

  bool have_prev_pose_{false};
  double last_pub_time_{0.0};
  double publish_period_{0.02};
  gz::math::Vector3d prev_position_{0.0, 0.0, 0.0};
  gz::math::Vector3d prev_body_velocity_{0.0, 0.0, 0.0};
  double prev_yaw_{0.0};

  std::vector<gz::math::Vector3d> trajectory_;
  std::size_t max_trajectory_points_{2500};
  double trajectory_min_spacing_{0.02};
};
}  // namespace four_wheel_car

GZ_ADD_PLUGIN(
  four_wheel_car::FourWheelCarSystem,
  gz::sim::System,
  four_wheel_car::FourWheelCarSystem::ISystemConfigure,
  four_wheel_car::FourWheelCarSystem::ISystemPreUpdate,
  four_wheel_car::FourWheelCarSystem::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
  four_wheel_car::FourWheelCarSystem,
  "four_wheel_car::FourWheelCarSystem")
