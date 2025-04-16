#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <cmath>
#include "unitree_go/msg/sport_mode_state.hpp"    // Pour accéder à SportModeState

// Inclusion du bon header, évite l'avertissement de dépréciation
#include <tf2_eigen/tf2_eigen.hpp>   // Remplace l'ancien header tf2_eigen.h par celui-ci

// Définition de la classe LidarFixer qui est un noeud ROS2
class LidarFixer : public rclcpp::Node {
public:
    LidarFixer() 
        : Node("go2_lidar_fixer"), 
          tf_buffer_(this->get_clock()),  // Initialisation de tf_buffer
          body_height_(0.32)  // Initialisation de la hauteur du corps du robot
    {
        RCLCPP_INFO(this->get_logger(), "Initialisation du noeud LidarFixer...");

        // Souscription au LiDAR pour recevoir les données du nuage de points
        sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/utlidar/cloud", 10,
            std::bind(&LidarFixer::cloud_callback, this, std::placeholders::_1));

        // Souscription à l'état SportMode pour récupérer la hauteur du corps
        sub_state_ = this->create_subscription<unitree_go::msg::SportModeState>(
            "/sportmodestate", 10,
            std::bind(&LidarFixer::state_callback, this, std::placeholders::_1));

        // Publisher pour envoyer les nuages de points corrigés
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/go2/lidar", 10);

        // Log pour vérifier la souscription
        RCLCPP_INFO(this->get_logger(), "Souscrit à /utlidar/cloud et /sportmodestate");
    }

private:
    // Callback qui sera appelée pour mettre à jour la hauteur du corps (SportModeState)
    void state_callback(const unitree_go::msg::SportModeState::SharedPtr msg) {
        body_height_ = msg->body_height;  // Mise à jour de la hauteur du corps
        RCLCPP_DEBUG(this->get_logger(), "Body height mise à jour : %.3f m", body_height_);
    }

    // Callback qui sera appelée pour traiter le nuage de points LiDAR
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "PointCloud reçu");

        // Conversion du message ROS en format PCL
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *cloud);

        // Calibration du LiDAR (rotation fixe selon l'axe Y)
        float correction_deg_y = 165.0f; // Correction de l'axe Y

        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        float correction_rad_y = correction_deg_y * M_PI / 180.0f;
        transform.rotate(Eigen::AngleAxisf(correction_rad_y, Eigen::Vector3f::UnitY()));

        // Application d'une translation verticale selon la hauteur du corps du robot
        transform.translation() << 0.0, 0.0, body_height_;

        // Transformation du nuage de points
        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

        // Filtrage du nuage de points pour créer un "cube" de points utiles
        float offset_x = -0.2;
        float min_x = -0.4 + offset_x, max_x = 0.4 + offset_x;
        float min_y = -0.2, max_y = 0.2;
        float min_z = 0, max_z = body_height_;

        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        for (const auto& point : transformed_cloud->points) {
            if (point.x < min_x || point.x > max_x || 
                point.y < min_y || point.y > max_y || 
                point.z < min_z || point.z > max_z) {
                filtered_cloud->points.push_back(point);
            }
        }

        // Publication du nuage filtré
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*filtered_cloud, output);
        output.header.stamp = this->now();
        output.header.frame_id = "go2";  // frame_id correspond à la frame du robot

        pub_->publish(output);
        RCLCPP_DEBUG(this->get_logger(), "Nuage publié : %zu points | Body height utilisée : %.3f m",
                    filtered_cloud->points.size(), body_height_);
    }

    // Déclaration des souscriptions et publication
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sub_state_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

    // Déclaration du tf_buffer et de la hauteur du corps
    tf2_ros::Buffer tf_buffer_;   // Buffers pour les transformations
    float body_height_;           // Hauteur du corps du robot
};

// Le point d'entrée principal du programme
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv); // Initialisation de ROS2

    // Création du noeud et appel du spin pour le traiter
    rclcpp::spin(std::make_shared<LidarFixer>());

    rclcpp::shutdown(); // Fermeture propre de ROS2
    return 0;
}
