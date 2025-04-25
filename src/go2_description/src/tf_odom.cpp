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

        // === TF transform odom → base_footprint (2D) ===
        geometry_msgs::msg::TransformStamped tf_base_footprint;
        tf_base_footprint.header.stamp = this->get_clock()->now();
        tf_base_footprint.header.frame_id = "odom";
        tf_base_footprint.child_frame_id = "base_footprint";

        // Position 2D : on ignore la composante Z pour une transformation 2D.
        tf_base_footprint.transform.translation.x = -msg->position[0];  // Position X (inversée selon ta convention)
        tf_base_footprint.transform.translation.y = - msg->position[1];   // Position Y (pas de changement ici)
        tf_base_footprint.transform.translation.z = 0.0;  // Pas de hauteur en 2D

        // Rotation 2D : seule la rotation autour de Z est utilisée.
        tf_base_footprint.transform.rotation.x = 0.0;  // Pas de rotation autour de l'axe X pour base_footprint 2D
        tf_base_footprint.transform.rotation.y = 0.0;  // Pas de rotation autour de l'axe Y pour base_footprint 2D
        tf_base_footprint.transform.rotation.z = qz;   // Rotation autour de Z (quaternion simplifié pour 2D)
        tf_base_footprint.transform.rotation.w = qw;   // Rotation w du quaternion

        tf_broadcaster_->sendTransform(tf_base_footprint);


        // === TF transform base_footprint → base_link ===
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = this->get_clock()->now();
        tf_msg.header.frame_id = "base_footprint";
        tf_msg.child_frame_id = "base_link";

        tf_msg.transform.translation.z = msg->body_height;

        tf_msg.transform.rotation.x = qx;
        tf_msg.transform.rotation.y = qy;

        tf_broadcaster_->sendTransform(tf_msg);


        // === Odométrie mise à jour avec base_footprint ===
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = tf_base_footprint.header.stamp;  // On utilise le timestamp de la transformation base_footprint
        odom_msg.header.frame_id = "odom";  // Le frame_id reste "odom"
        odom_msg.child_frame_id = "base_footprint";  // Maintenant base_footprint est le repère enfant de l'odométrie

        // Position 2D dans le repère odom (base_footprint)
        odom_msg.pose.pose.position.x = -msg->position[0];  // Position X (inversée selon ta convention)
        odom_msg.pose.pose.position.y = -msg->position[1];   // Position Y (pas de changement ici)
        odom_msg.pose.pose.position.z = 0.0;  // Pas de hauteur en 2D

        // Orientation 2D dans le repère odom (seulement rotation autour de Z)
        odom_msg.pose.pose.orientation.x = 0.0;  // Pas de rotation autour de X pour base_footprint 2D
        odom_msg.pose.pose.orientation.y = 0.0;  // Pas de rotation autour de Y pour base_footprint 2D
        odom_msg.pose.pose.orientation.z = qz;   // Rotation autour de Z (quaternion simplifié pour 2D)
        odom_msg.pose.pose.orientation.w = qw;   // Rotation w du quaternion

        // Vitesse linéaire et angulaire
        odom_msg.twist.twist.linear.x = msg->velocity[0];  // Vitesse linéaire en X (dans le repère base_footprint)
        odom_msg.twist.twist.linear.y = - msg->velocity[1];  // Vitesse linéaire en Y
        odom_msg.twist.twist.linear.z = 0.0;               // Pas de vitesse en Z en 2D
        odom_msg.twist.twist.angular.z = msg->yaw_speed;    // Vitesse angulaire autour de Z

        // Publier l'odométrie
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