#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <chrono>
#include <iostream>
#include <sstream>
#include <cstdlib>

using namespace std::chrono;
using namespace std::chrono_literals;

class StarlingInterface : public rclcpp::Node
{
public:
    StarlingInterface() : Node("starling_interface")
    {
        // Input Parameters
        this->declare_parameter<std::string>("namespace", "rX");
        std::string ns;
        this->get_parameter("namespace", ns);

        // QoS
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        // Velocity Translation (TwistStamped [GNN] to TrajectorySetPoint [PX4])
        gnn_vel_subscription = this->create_subscription<geometry_msgs::msg::TwistStamped>(
                ns + "/cmd_vel", qos, std::bind(&StarlingInterface::publish_trajectory_setpoint, this, std::placeholders::_1));
        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetPoint>(ns + "/fmu/in/trajectory_setpoint", qos);

        // Position Translation (VehicleLocalPosition [PX4] to PoseStamped [GNN])
        //vehicle_local_pos_subscription_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        //        ns + "/fmu/out/vehicle_local_position", qos, std::bind(&StarlingInterface::publish_pose, this, std::placeholders::_1));

        // Synchronize two topics with message message
        // Required to construct PoseStamped message from local position + attitude
        vehicle_local_pos_sub_.subscribe(this, ns + "/fmu/out/vehicle_local_position");
        vehicle_attitude_sub_.subscribe(this, ns + "/fmu/out/vehicle_attitude");

        sync_.reset(new Sync(SyncPolicy(10), twist_sub_, imu_sub_));
        sync_->registerCallback(std::bind(&StarlingInterface::publish_pose, this, std::placeholders::_1, std::placeholders::_2));

        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(ns + "/pose", qos);
    }

private:
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr gnn_vel_subscription_;
        rclcpp::Publisher<px4_msgs::msg::TrajectorySetPoint>::SharedPtr trajectory_setpoint_publisher_;
        rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr vehicle_local_position_subscription_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;

        message_filters::Subscriber<px4_msg::msg::VehicleLocalPosition> vehicle_local_pos_sub_;
        message_filters::Subscriber<px4_msg::msg::VehicleAttitude> vehicle_attitude_sub_;

        typedef message_filters::sync_policies::ApproximateTime<> SyncPolicy;
        typedef message_filters::Synchronizer<SyncPolicy> Sync;
        std::shared_ptr<Sync> sync_;

        std::atomic<uint64_t> timestamp_;

        float scale_;

        void publish_trajectory_setpoint(const geometry_msgs::msg::TwistStamped &gnn_cmd_vel);
        void publish_pose(const px4_msgs::msg::VehicleLocalPosition &vehicle_local_position);
};

void StarlingInterface::publish_trajectory_setpoint(const geometry_msgs::msg::TwistStamped &gnn_cmd_vel)
{
    px4::msg::TrajectorySetPoint px4_msg{};
	msg.position = {std::nanf(""), std::nanf(""), std::nanf("")}; // required for vel control in px4
	msg.velocity = {gnn_cmd_vel.vx, gnn_cmd_vel.vy, gnn_cmd_vel.vz};
	msg.yaw = -3.14; // [-PI:PI] //TODO
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

void StarlingInterface::publish_pose(const px4_msgs::msg::VehicleLocalPosition &vehicle_local_position, const px4_msgs::msg::VehicleAttitude &vehicle_attitude)
{
    geometry_msgs::msg::PoseStamped gnn_pose;
    gnn_pose.header.stamp = this->get_clock()->now();
    gnn_pose.pose.position.x = vehicle_local_position.x;
    gnn_pose.pose.position.y = vehicle_local_position.y;
    gnn_pose.pose.position.z = vehicle_local_position.z;
    gnn_pose.pose.orientation.x = vehicle_attitude.q[1];
    gnn_pose.pose.orientation.y = vehicle_attitude.q[2];
    gnn_pose.pose.orientation.z = vehicle_attitude.q[3];
    gnn_pose.pose.orientation.w = vehicle_attitude.q[0];

    pose_publisher_->publish(gnn_pose);
}


