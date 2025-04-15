#include <rclcpp/rclcpp.hpp>
#include "unitree_api/msg/request.hpp"
#include "ros2_sport_client.h"

class RobotNode : public rclcpp::Node
{
public:
    RobotNode() : Node("go2_scrape"), timer_count_(0)
    {
        // Création du publisher pour envoyer la commande
        req_puber_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

        // Initialisation du client Sport
        sport_req_ = std::make_shared<SportClient>();

        // Créer un timer qui envoie la commande une seule fois après 1 seconde
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&RobotNode::send_request, this));
    }

private:
    void send_request()
    {
        // Créer la requête
        unitree_api::msg::Request req;
        sport_req_->Scrape(req);

        // Publier la requête
        req_puber_->publish(req);
        RCLCPP_INFO(this->get_logger(), "Commande 'Scrape' envoyée au robot.");

        // Incrémenter le compteur du timer
        timer_count_++;

        // Si c'est la première exécution, arrêter le timer et fermer proprement le nœud
        if (timer_count_ == 1)
        {
            timer_->cancel();  // Annuler le timer après 1 exécution
            rclcpp::shutdown(); // Arrêter le nœud
        }
    }

    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber_;
    std::shared_ptr<SportClient> sport_req_;  // Client pour envoyer des commandes
    rclcpp::TimerBase::SharedPtr timer_;      // Timer pour envoyer la commande périodiquement
    int timer_count_;  // Compteur pour gérer l'arrêt après une seule exécution
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Créer une instance du nœud et envoyer la commande
    auto node = std::make_shared<RobotNode>();

    // Spin pour le nœud (il attend jusqu'à ce que le timer s'arrête et le nœud ferme)
    rclcpp::spin(node);

    // Shutdown une fois que le nœud a terminé
    rclcpp::shutdown();
    return 0;
}