#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <cmath>
#include "unitree_go/msg/sport_mode_state.hpp"    // Pour accéder à SportModeState

class LidarFixer : public rclcpp::Node {
public:
    LidarFixer() : Node("go2_lidar_fixer"), body_height_(0.32) {
        RCLCPP_INFO(this->get_logger(), "Initialisation du noeud LidarFixer...");

        // Souscription au LiDAR
        sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/utlidar/cloud", 10,
            std::bind(&LidarFixer::cloud_callback, this, std::placeholders::_1));

        // Souscription à l'état SportMode (pour récupérer la hauteur du corps)
        sub_state_ = this->create_subscription<unitree_go::msg::SportModeState>(
            "/sportmodestate", 10,
            std::bind(&LidarFixer::state_callback, this, std::placeholders::_1));

        // Publisher du LiDAR corrigé
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/go2/lidar", 10);

        RCLCPP_INFO(this->get_logger(), "Souscrit à /utlidar/cloud et /sportmodestate");
    }

private:
    void state_callback(const unitree_go::msg::SportModeState::SharedPtr msg) {
        body_height_ = msg->body_height;  // Stockage de la hauteur
        RCLCPP_DEBUG(this->get_logger(), "Body height mise à jour : %.3f m", body_height_);
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "PointCloud reçu");

        // Conversion ROS -> PCL
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *cloud);

        // Calibration du LiDAR (rotations fixes)
        float correction_deg_y = 193.0f; //degrés
        float correction_deg_z = 171.5f; //degrés

        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        float correction_rad_y = correction_deg_y * M_PI / 180.0f;
        transform.rotate(Eigen::AngleAxisf(correction_rad_y, Eigen::Vector3f::UnitY()));
        float correction_rad_z = correction_deg_z * M_PI / 180.0f;
        transform.rotate(Eigen::AngleAxisf(correction_rad_z, Eigen::Vector3f::UnitZ()));
        // Translation verticale selon body_height
        transform.translation() << 0.0, 0.0, body_height_;

        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

        // Filtrage d’un cube central
        float offset_x = 0.2;
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

        // Publication
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*filtered_cloud, output);
        output.header.stamp = this->now();
        output.header.frame_id = "map";

        pub_->publish(output);
        RCLCPP_INFO(this->get_logger(), "Nuage publié : %zu points | Body height utilisée : %.3f m",
                    filtered_cloud->points.size(), body_height_);
    }

    // Souscriptions et publication
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr sub_state_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

    // Variable de hauteur du corps (en mètres)
    float body_height_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarFixer>());
    rclcpp::shutdown();
    return 0;
}
