#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "unitree_api/msg/request.hpp"
#include "ros2_sport_client.h"

class MoveRobotNode : public rclcpp::Node
{
public:
    // Constructeur du nœud
    MoveRobotNode() : Node("go2_move")
    {
        // Création du publisher pour envoyer la commande Request (Move ou StopMove) au robot.
        req_puber_ = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

        // Initialisation du client Sport (permet d'envoyer les commandes de mouvement)
        sport_req_ = std::make_shared<SportClient>();

        // Création d'un subscriber pour recevoir les messages Twist depuis /go2/move
        move_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/go2/move", 10, std::bind(&MoveRobotNode::move_callback, this, std::placeholders::_1));

        // Affichage d'un message d'information pour indiquer que le nœud est prêt à recevoir des commandes
        RCLCPP_INFO(this->get_logger(), "Abonné au topic /go2/move et en attente de commandes...");
    }

private:
    // Callback qui est appelé à chaque réception d'un message Twist sur /go2/move
    void move_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // Récupération des vitesses de mouvement du message reçu
        float vx = msg->linear.x;   // Vitesse linéaire en X (avant/arrière)
        float vy = msg->linear.y;   // Vitesse linéaire en Y (gauche/droite)
        float vz = msg->linear.z; // Vitesse de rotation autour de Z ("yaw")

        // Récupération des angles roll, pitch à partir du message Twist
        float roll = msg->angular.x;  // Rotation autour de l'axe X (roll)
        float pitch = msg->angular.y; // Rotation autour de l'axe Y (pitch)
        float yaw = msg->angular.z; // Rotation autour de l'axe Z (yaw)

        // Création d'une nouvelle requête pour envoyer au robot
        unitree_api::msg::Request req;

        // Vérification si toutes les vitesses sont à zéro (arrêt complet demandé)
        if (vx == 0.0f && vy == 0.0f && vz == 0.0f) {
            // Si toutes les vitesses sont nulles, envoyer la commande 'stopmove'
            RCLCPP_INFO(this->get_logger(), "Toutes les vitesses sont à zéro, envoi de la commande 'stopmove'");
            sport_req_->StopMove(req);  // Utilisation de la fonction StopMove pour arrêter le robot
            
            // Envoi de la requête StopMove
            req_puber_->publish(req);
            
            // Attente ou publication supplémentaire pour être sûr que l'arrêt est pris en compte
            RCLCPP_INFO(this->get_logger(), "Commande 'StopMove' envoyée, robot devrait s'arrêter.");
        } else {
            // Sinon, envoyer la commande 'Move' avec les vitesses actuelles
            sport_req_->Move(req, vx, vy, vz);
            RCLCPP_INFO(this->get_logger(), "Commande 'Move' => vx: %.2f | vy: %.2f | vz: %.2f", vx, vy, vz);

            // Publier la requête Move pour que le robot effectue le mouvement
            req_puber_->publish(req);
        }

        // Envoyer la commande Euler avec les valeurs roll, pitch, et yaw (yaw = 0)
        RCLCPP_INFO(this->get_logger(), "Commande 'Euler' => Roll: %.2f | Pitch: %.2f | Yaw: %.2f", roll, pitch, yaw);
        sport_req_->Euler(req, roll, pitch, yaw); // Envoi des angles de posture (Euler)

        // Publier la requête Euler pour que le robot effectue le mouvement
        req_puber_->publish(req);
    }

    // Déclaration des interfaces ROS2
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber_;  // Publisher pour envoyer la requête au robot
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr move_subscriber_;  // Subscriber pour recevoir les commandes de mouvement
    std::shared_ptr<SportClient> sport_req_;  // Client pour envoyer des commandes au robot via l'API
};

int main(int argc, char **argv)
{
    // Initialisation de ROS2
    rclcpp::init(argc, argv);

    // Création d'une instance du nœud MoveRobotNode
    auto node = std::make_shared<MoveRobotNode>();

    // Boucle principale du nœud (attente des messages /go2/move)
    rclcpp::spin(node);

    // Arrêt de ROS2 quand le nœud termine
    rclcpp::shutdown();
    return 0;
}
