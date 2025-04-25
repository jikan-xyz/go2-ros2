#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>

class LidarFixer : public rclcpp::Node {
public:
    LidarFixer() 
        : Node("go2_lidar_fixer")
    {
        RCLCPP_INFO(this->get_logger(), "Initialisation du noeud LidarFixer...");

        sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/utlidar/cloud", 10,
            std::bind(&LidarFixer::cloud_callback, this, std::placeholders::_1));

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/go2/lidar", 10);

        RCLCPP_INFO(this->get_logger(), "Souscrit à /utlidar/cloud");
    }

private:
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_DEBUG(this->get_logger(), "PointCloud reçu");

        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *cloud);

        // === Correction basée sur l'URDF ===
        float correction_rad_y = 2.8782f;  // Rotation autour de Y (URDF)

        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate(Eigen::AngleAxisf(correction_rad_y, Eigen::Vector3f::UnitY()));

        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*cloud, *transformed_cloud, transform);

        // === Filtrage basique pour virer les pattes ===
        float offset_x = -0.3;
        float min_x = -0.3f + offset_x, max_x = 0.5f + offset_x;
        float min_y = -0.2f, max_y = 0.2f;
        float min_z = -0.3f, max_z = 0.05f;

        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        for (const auto& point : transformed_cloud->points) {
            if (point.x < min_x || point.x > max_x || 
                point.y < min_y || point.y > max_y || 
                point.z < min_z || point.z > max_z) {
                filtered_cloud->points.push_back(point);
            }
        }

        // === Conversion ROS + publication ===
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*filtered_cloud, output);
        output.header.stamp = this->now();
        output.header.frame_id = "lidar";  // repère global du robot

        pub_->publish(output);
        RCLCPP_DEBUG(this->get_logger(), "Nuage publié : %zu points", filtered_cloud->points.size());
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarFixer>());
    rclcpp::shutdown();
    return 0;
}
