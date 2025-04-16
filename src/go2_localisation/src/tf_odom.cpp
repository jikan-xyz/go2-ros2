#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"

// Définition de la classe Go2TF_Odom qui gère les transformations TF2 et l'odométrie
class Go2TF_Odom : public rclcpp::Node
{
public:
    // Constructeur du noeud, initialisation des composants nécessaires
    Go2TF_Odom()
    : Node("go2_tf_odom")
    {
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);  // Initialisation du broadcaster TF2
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/go2/odom", 10);  // Création du publisher d'odométrie
        subscription_ = this->create_subscription<unitree_go::msg::SportModeState>(
            "/sportmodestate", 10, std::bind(&Go2TF_Odom::imu_callback, this, std::placeholders::_1));  // Abonnement au topic IMU
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "TF et Odom actifs.");  // Confirmation que le noeud est lancé
    }

private:
    // Callback appelée à chaque réception d'un message sur le topic /sportmodestate
    void imu_callback(const unitree_go::msg::SportModeState::SharedPtr msg)
    {
        // Création et envoi de la transformation pour le robot
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = this->get_clock()->now();
        tf_msg.header.frame_id = "map";
        tf_msg.child_frame_id = "go2";
        
        tf_msg.transform.translation.x = -msg->position[0];  // Inversion de la position X pour la transformation
        tf_msg.transform.translation.y = -msg->position[1];  // Inversion de la position Y pour la transformation
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation.x = msg->imu_state.quaternion[2];
        tf_msg.transform.rotation.y = -msg->imu_state.quaternion[1];
        tf_msg.transform.rotation.z = -msg->imu_state.quaternion[0];
        tf_msg.transform.rotation.w = msg->imu_state.quaternion[3];

        tf_broadcaster_->sendTransform(tf_msg);  // Envoi de la transformation via TF2

        // Création du message d'odométrie avec les informations du robot
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = this->get_clock()->now();
        odom_msg.header.frame_id = "odom";  // Frame de référence pour l'odométrie
        odom_msg.child_frame_id = "go2";    // Frame du robot

        odom_msg.pose.pose.position.x = msg->position[0];  // Position normale en X
        odom_msg.pose.pose.position.y = msg->position[1];  // Position normale en Y
        odom_msg.pose.pose.position.z = 0.0;

        // Orientation basée sur le quaternion de l'IMU
        odom_msg.pose.pose.orientation.x = msg->imu_state.quaternion[2];
        odom_msg.pose.pose.orientation.y = -msg->imu_state.quaternion[1];
        odom_msg.pose.pose.orientation.z = -msg->imu_state.quaternion[0];
        odom_msg.pose.pose.orientation.w = msg->imu_state.quaternion[3];

        // Vitesse du robot en X, Y et Z, basée sur l'IMU
        odom_msg.twist.twist.linear.x = msg->velocity[0];  // Vitesse en X
        odom_msg.twist.twist.linear.y = msg->velocity[1];  // Vitesse en Y
        odom_msg.twist.twist.linear.z = msg->velocity[2];

        odom_msg.twist.twist.angular.z = msg->yaw_speed;  // Vitesse angulaire (lacet)

        odom_publisher_->publish(odom_msg);  // Publication du message d'odométrie

        x_ = msg->position[0];  // Mise à jour de la position X
        y_ = msg->position[1];  // Mise à jour de la position Y
    }

    // Déclaration des membres pour l'abonnement, le broadcaster et le publisher
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr subscription_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    double x_, y_, theta_;  // Variables pour stocker la position et l'orientation du robot
};

// Fonction principale pour initialiser le noeud et lancer le spin
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);  // Initialisation de ROS2
    rclcpp::spin(std::make_shared<Go2TF_Odom>());  // Lancement du spin pour attendre les messages
    rclcpp::shutdown();  // Fermeture propre de ROS2
    return 0;
}
