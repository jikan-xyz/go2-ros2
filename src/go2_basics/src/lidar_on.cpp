#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SwitchPublisherNode : public rclcpp::Node
{
public:
    SwitchPublisherNode() : Node("go2_lidarr_on"), timer_count_(0)
    {
        // Création du publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>("/utlidar/switch", 10);

        // Timer pour envoyer le message une seule fois après un court délai
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&SwitchPublisherNode::publish_once, this));
    }

private:
    void publish_once()
    {
        // Création du message
        std_msgs::msg::String msg;
        msg.data = "ON";

        // Publication du message
        publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Message publié : '%s'", msg.data.c_str());

        // Arrêter le timer et fermer le nœud
        timer_->cancel();
        rclcpp::shutdown();
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int timer_count_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SwitchPublisherNode>();
    rclcpp::spin(node);
    return 0;
}
