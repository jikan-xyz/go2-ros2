#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"

class Go2TF_Odom : public rclcpp::Node
{
public:
    Go2TF_Odom()
    : Node("go2_tf_odom")
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/go2/odom", 10);
        subscription_ = this->create_subscription<unitree_go::msg::SportModeState>(
            "/sportmodestate", 10,
            std::bind(&Go2TF_Odom::imu_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "TF et Odom actifs.");
    }

private:
    void imu_callback(const unitree_go::msg::SportModeState::SharedPtr msg)
    {
        // === Quaternion conversion from Go2 to ROS (Z up, X forward) ===
        double qx = msg->imu_state.quaternion[2];   // ROS x ← Go2 z
        double qy = -msg->imu_state.quaternion[1];  // ROS y ← -Go2 y
        double qz = -msg->imu_state.quaternion[0];  // ROS z ← -Go2 x
        double qw = msg->imu_state.quaternion[3];   // w ok

        // === TF transform odom → go2 ===
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = this->get_clock()->now();
        tf_msg.header.frame_id = "odom";
        tf_msg.child_frame_id = "go2";

        tf_msg.transform.translation.x = - msg->position[0];  // Si X inversé en Go2 → mettre un "-"
        tf_msg.transform.translation.y = msg->position[1];
        tf_msg.transform.translation.z = msg->body_height;

        tf_msg.transform.rotation.x = qx;
        tf_msg.transform.rotation.y = qy;
        tf_msg.transform.rotation.z = qz;
        tf_msg.transform.rotation.w = qw;

        tf_broadcaster_->sendTransform(tf_msg);

        // === Odométrie ===
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = tf_msg.header.stamp;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "go2";

        odom_msg.pose.pose.position.x = - msg->position[0];  // Même ici → inverser si nécessaire
        odom_msg.pose.pose.position.y = msg->position[1];
        odom_msg.pose.pose.position.z = 0.0;

        odom_msg.pose.pose.orientation.x = qx;
        odom_msg.pose.pose.orientation.y = qy;
        odom_msg.pose.pose.orientation.z = qz;
        odom_msg.pose.pose.orientation.w = qw;

        odom_msg.twist.twist.linear.x = msg->velocity[0];
        odom_msg.twist.twist.linear.y = msg->velocity[1];
        odom_msg.twist.twist.linear.z = msg->velocity[2];
        odom_msg.twist.twist.angular.z = msg->yaw_speed;

        odom_publisher_->publish(odom_msg);
    }

    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Go2TF_Odom>());
    rclcpp::shutdown();
    return 0;
}
