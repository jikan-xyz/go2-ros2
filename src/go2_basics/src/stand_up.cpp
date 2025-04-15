#include <rclcpp/rclcpp.hpp>
#include "unitree_api/msg/request.hpp"
#include "ros2_sport_client.h"

class RobotNode : public rclcpp::Node
{
public:
    RobotNode() : Node("go2_stand_up"), timer_count_(0)
    {
        // Création du publisher pour envoyer la commande Request
        req_puber_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

        // Initialisation du client Sport
        sport_req_ = std::make_shared<SportClient>();

        // Créer un timer qui envoie la commande StandUp après 1 seconde
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&RobotNode::send_request, this));
    }

private:
    void send_request()
    {
        // Créer la requête
        unitree_api::msg::Request req;
        sport_req_->StandUp(req);

        // Publier la requête 
        req_puber_->publish(req);
        RCLCPP_INFO(this->get_logger(), "Commande 'StandUp' envoyée au robot.");

        // Incrémenter le compteur du timer
        timer_count_++;

        // Lancer le timer pour envoyer la commande BalanceStand après 0.5 seconde
        if (timer_count_ == 1)
        {
            // Créer un nouveau timer pour envoyer la commande BalanceStand après 0.5 seconde
            balance_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(500),
                std::bind(&RobotNode::send_balance_stand_request, this));
        }

        // Si c'est la première exécution, arrêter le timer initial et fermer proprement le nœud
        if (timer_count_ == 1)
        {
            timer_->cancel();  // Annuler le timer après l'envoi de StandUp
        }
    }

    // Callback pour envoyer la commande BalanceStand après 0.5 seconde
    void send_balance_stand_request()
    {
        // Créer la requête BalanceStand
        unitree_api::msg::Request req;
        sport_req_->BalanceStand(req);

        // Publier la requête BalanceStand pour stabiliser le robot
        req_puber_->publish(req);
        RCLCPP_INFO(this->get_logger(), "Commande 'BalanceStand' envoyée au robot.");

        // Arrêter le nœud après l'envoi de BalanceStand
        rclcpp::shutdown();
    }

    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber_;  // Publisher pour envoyer la requête au robot
    std::shared_ptr<SportClient> sport_req_;  // Client pour envoyer des commandes
    rclcpp::TimerBase::SharedPtr timer_;      // Timer pour envoyer la commande StandUp après 1 seconde
    rclcpp::TimerBase::SharedPtr balance_timer_; // Timer pour envoyer BalanceStand après 0.5 seconde
    int timer_count_;  // Compteur pour gérer l'arrêt après une seule exécution
};

int main(int argc, char **argv)
{
    // Initialisation de ROS2
    rclcpp::init(argc, argv);

    // Créer une instance du nœud RobotNode
    auto node = std::make_shared<RobotNode>();

    // Spin pour le nœud (il attend jusqu'à ce que le timer s'arrête et le nœud ferme)
    rclcpp::spin(node);

    // Arrêt de ROS2 quand le nœud termine
    rclcpp::shutdown();
    return 0;
}