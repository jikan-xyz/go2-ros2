#include "rclcpp/rclcpp.hpp"                    // ROS2 Core
#include "unitree_go/msg/sport_mode_state.hpp"    // Pour accéder aux messages du Go2 (SportModeState)
#include "geometry_msgs/msg/transform_stamped.hpp"  // Message pour la publication TF2
#include "tf2_ros/transform_broadcaster.h"           // Librairie pour broadcaster TF2

// Définition de la classe Go2TFBroadcaster, héritant de la classe Node de ROS2
class Go2TFBroadcaster : public rclcpp::Node
{
public:
    // Constructeur du noeud Go2TFBroadcaster
    Go2TFBroadcaster()
    : Node("go2_tf_broadcaster")  // Nom du noeud ROS2
    {
        // Création du broadcaster TF2 (qui permet de publier des transformations)
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Souscription au topic /sportmodestate qui nous enverra les données IMU du robot
        subscription_ = this->create_subscription<unitree_go::msg::SportModeState>(
            "/sportmodestate",  // Nom du topic qui contient les données IMU
            10,                  // Taille du buffer (10 messages dans la file d'attente)
            std::bind(&Go2TFBroadcaster::imu_callback, this, std::placeholders::_1) // Callback quand on reçoit un message
        );

        // Message de log pour confirmer que le broadcaster est actif
        RCLCPP_INFO(this->get_logger(), "TF2 broadcaster actif. Abonnement au topic /sportmodestate...");
    }

private:
    // Callback appelée chaque fois qu'un message IMU est reçu sur le topic /sportmodestate
    void imu_callback(const unitree_go::msg::SportModeState::SharedPtr msg)
    {
        // Log pour vérifier que le message a bien été reçu
        RCLCPP_INFO(this->get_logger(), "Message IMU reçu. Traitement de l'orientation...");

        // Création d'un message TransformStamped, qui est la structure utilisée pour publier des transformations dans TF2
        geometry_msgs::msg::TransformStamped tf_msg;

        // Nous affectons l'horodatage (timestamp) à la transformation
        tf_msg.header.stamp = this->get_clock()->now();
        
        // Log pour vérifier le timestamp (en secondes et nanosecondes)
        RCLCPP_INFO(this->get_logger(), "Timestamp du message : %d.%09u",
                    tf_msg.header.stamp.sec, tf_msg.header.stamp.nanosec);

        // La frame de référence parent (ici, c'est "map", mais tu peux choisir d'autres frames comme "odom" ou "world")
        tf_msg.header.frame_id = "map";  // C'est la frame de la carte de ton système de navigation

        // La frame enfant (ici, c'est le robot Go2, qui va tourner dans RViz)
        tf_msg.child_frame_id = "go2";   // La frame que l'on va publier (le robot)

        // Log pour afficher les frames de référence
        RCLCPP_INFO(this->get_logger(), "Publication de la transformation : %s -> %s", tf_msg.header.frame_id.c_str(), tf_msg.child_frame_id.c_str());

        // La position du robot, ici fixée à zéro, car on ne s'intéresse qu'à l'orientation
        tf_msg.transform.translation.x = 0.0;  // X
        tf_msg.transform.translation.y = 0.0;  // Y
        tf_msg.transform.translation.z = 0.0;  // Z

        // Conversion des axes : on échange les composantes du quaternion pour adapter l'orientation
        tf_msg.transform.rotation.x = msg->imu_state.quaternion[2];       // X
        tf_msg.transform.rotation.y = -msg->imu_state.quaternion[1];      // inverser Y
        tf_msg.transform.rotation.z = -msg->imu_state.quaternion[0];      // inverser Z
        tf_msg.transform.rotation.w = msg->imu_state.quaternion[3];       // W inchangé

        // Log pour afficher les valeurs du quaternion
        RCLCPP_INFO(this->get_logger(), "Quaternion du robot: [%f, %f, %f, %f]",
                    tf_msg.transform.rotation.x, tf_msg.transform.rotation.y,
                    tf_msg.transform.rotation.z, tf_msg.transform.rotation.w);

        // Enfin, on envoie cette transformation via le broadcaster
        tf_broadcaster_->sendTransform(tf_msg);

        // Log pour confirmer que la transformation a bien été envoyée
        RCLCPP_INFO(this->get_logger(), "Transformation envoyée avec succès.");
    }

    // Abonnement au topic SportModeState
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr subscription_;

    // Le broadcaster qui envoie les transformations TF2
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

// Le point d'entrée de l'application (la fonction main)
int main(int argc, char * argv[])
{
    // Initialisation de ROS2
    rclcpp::init(argc, argv);

    // Création du noeud Go2TFBroadcaster et lancement du spin (qui boucle en attendant les messages)
    rclcpp::spin(std::make_shared<Go2TFBroadcaster>());

    // Fermeture propre de ROS2 après avoir quitté le spin
    rclcpp::shutdown();
    return 0;
}
