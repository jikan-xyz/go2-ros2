#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <cmath>  // Pour std::abs

class MoveRobotNode : public rclcpp::Node
{
public:
    MoveRobotNode() : Node("go2_teleop")
    {
        // Initialisation du publisher pour envoyer des commandes de mouvement
        move_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/go2/move", 10);

        // Initialisation du subscriber pour écouter les entrées de la manette
        joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&MoveRobotNode::input_callback, this, std::placeholders::_1));
    }

private:
    bool is_lock_ = false;  // Variable pour savoir si le robot est verrouillé
    
    // Fonction de callback qui traite les données de la manette
    void input_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        const float dead_zone = 0.08f;  // Zone morte pour ignorer les petits mouvements parasites

        // Lecture des axes de la manette
        float left_stick_x = msg->axes[0];  // Axe horizontal du stick gauche
        float left_stick_y = msg->axes[1];  // Axe vertical du stick gauche
        float right_stick_x = msg->axes[2]; // Axe horizontal du stick droit
        float right_stick_y = msg->axes[3]; // Axe vertical du stick droit
        float dpad_vertical = msg->axes[5];  // Axe vertical de la croix directionnelle (haut/bas)

        // Appliquer la zone morte
        if (std::abs(left_stick_x) < dead_zone) left_stick_x = 0.0f;
        if (std::abs(left_stick_y) < dead_zone) left_stick_y = 0.0f;
        if (std::abs(right_stick_x) < dead_zone) right_stick_x = 0.0f;
        if (std::abs(right_stick_y) < dead_zone) right_stick_y = 0.0f;

        // Mapping des axes de la manette dans une plage adaptée pour le contrôle
        float mapped_left_y = map_range(left_stick_y, -1.0f, 1.0f, -2.5f, 2.5f) / 2.0f;  // Déplacement avant/arrière
        float mapped_right_x = map_range(right_stick_x, -1.0f, 1.0f, -4.0f, 4.0f) / 2.0f;  // Rotation gauche/droite
        float mapped_right_y = map_range(right_stick_y, -1.0f, 1.0f, -0.75f, 0.75f) / 2.0f;  // Rotation du pitch (inclinaison)

        // Mode Roll/Pitch/Yaw lorsque LB est pressé
        if (msg->buttons[4] == 1)  // LB
        {
            // Contrôles de l'orientation du robot
            last_roll_ = map_range(right_stick_x, -1.0f, 1.0f, -0.75f, 0.75f);  // Contrôle du roll (axe droit X)
            last_pitch_ = mapped_right_y;  // Contrôle du pitch (axe droit Y)
            last_yaw_ = map_range(left_stick_x, -1.0f, 1.0f, -0.6f, 0.6f);  // Contrôle du yaw (axe gauche X)
            last_vx_ = last_vy_ = last_vz_ = 0.0f;  // Pas de mouvement linéaire en mode orientation
        }
        else  // Mode déplacement
        {
            last_vx_ = mapped_left_y;  // Déplacement avant/arrière
            last_vy_ = left_stick_x;   // Déplacement gauche/droite
            last_vz_ = mapped_right_x; // Rotation gauche/droite
            last_pitch_ = mapped_right_y;  // Pitch déjà mappé
            last_roll_ = last_yaw_ = 0.0f; // Pas de rotation supplémentaire
        }

        // Mise à jour des boutons pressés
        lt_pressed_ = (msg->buttons[6] == 1);
        rt_pressed_ = (msg->buttons[7] == 1);
        lb_pressed_ = (msg->buttons[4] == 1);
        rb_pressed_ = (msg->buttons[5] == 1);
        x_pressed_ = (msg->buttons[0] == 1);
        a_pressed_ = (msg->buttons[1] == 1);
        b_pressed_ = (msg->buttons[2] == 1);
        y_pressed_ = (msg->buttons[3] == 1);
        select_pressed_ = (msg->buttons[8] == 1);
        start_pressed_ = (msg->buttons[9] == 1);
        left_stick_pressed_ = (msg->buttons[10] == 1);
        right_stick_pressed_ = (msg->buttons[11] == 1);

        // Envoi de la commande
        send_command(last_vx_, last_vy_, last_vz_, last_pitch_, last_roll_, last_yaw_);

        // Verrouillage ou déverrouillage du robot si LB et RB sont pressés simultanément
        if (lb_pressed_ && rb_pressed_) {
            is_lock_ = !is_lock_;  // Inverse l'état de verrouillage à chaque pression simultanée
            if (is_lock_) {
                RCLCPP_INFO(this->get_logger(), "Robot verrouillé.");
                system("ros2 run go2_basics lock");  // Exécution de la commande
            } else {
                RCLCPP_INFO(this->get_logger(), "Robot déverrouillé.");
                system("ros2 run go2_basics unlock");  // Exécution de la commande
            }
        }

        // Gestion des combos sans délai
        handle_combo(lb_pressed_ && dpad_vertical == 1, "ros2 run go2_basics stand_up", "Stand Up !");
        handle_combo(lb_pressed_ && dpad_vertical == -1, "ros2 run go2_basics stand_down", "Stand Down !");
        handle_combo(lb_pressed_ && b_pressed_, "ros2 run go2_basics scrape", "Scrape !");
        handle_combo(lb_pressed_ && x_pressed_, "ros2 run go2_basics hello", "Hello !");
        handle_combo(lb_pressed_ && y_pressed_, "ros2 run go2_basics jump", "Jump !");
        handle_combo(lb_pressed_ && a_pressed_, "ros2 run go2_basics pounce", "Pounce !");        
        handle_combo(rb_pressed_ && x_pressed_, "ros2 run go2_basics lidar_on", "Lidar On !");        
        handle_combo(rb_pressed_ && y_pressed_, "ros2 run go2_basics lidar_off", "Lidar OFF !");        
    }

    // Fonction pour envoyer la commande de mouvement au robot
    void send_command(float vx, float vy, float vz, float pitch, float roll, float yaw)
    {
        // ARRET D'URGENCE si LT et RT sont tous les deux pressés
        if (lt_pressed_ && rt_pressed_)
        {
            RCLCPP_WARN(this->get_logger(), "ARRET D'URGENCE - envoi de la commande ZERO");
            geometry_msgs::msg::Twist stop_msg;
            move_publisher_->publish(stop_msg);  // Envoi d'une commande de stop (vitesse et rotation nulles)
            return;
        }

        // Si aucune commande de mouvement n'est donnée, on envoie une commande "zéro"
        if (vx == 0.0f && vy == 0.0f && vz == 0.0f && pitch == 0.0f && roll == 0.0f && yaw == 0.0f)
        {
            if (!last_movement_was_zero_) {
                RCLCPP_INFO(this->get_logger(), "Robot stoppé - envoi de la commande zéro.");
                geometry_msgs::msg::Twist msg;
                move_publisher_->publish(msg);  // Envoi de la commande "zéro"
            }
            last_movement_was_zero_ = true;
        }
        else  // Si un mouvement est demandé, envoi de la commande avec les vitesses
        {
            if (!is_lock_)
            {
                geometry_msgs::msg::Twist msg;
                msg.linear.x = vx;
                msg.linear.y = vy;
                msg.linear.z = vz;
                msg.angular.x = roll;  // Contrôle du roll
                msg.angular.y = pitch; // Contrôle du pitch
                msg.angular.z = yaw;   // Contrôle du yaw
                move_publisher_->publish(msg);  // Envoi de la commande

                // Affichage de l'état des commandes
                RCLCPP_INFO(this->get_logger(), "Commande => vx: %.2f | vy: %.2f | vz: %.2f | pitch: %.2f | roll: %.2f | yaw: %.2f", vx, vy, vz, pitch, roll, yaw);
                last_movement_was_zero_ = false;
            } else {
                RCLCPP_WARN(this->get_logger(), "ROBOT VEROUILLE");
            }
        }
    }

    // Fonction pour gérer l'exécution des combos (actions simultanées sur plusieurs boutons)
    void handle_combo(bool condition, const std::string &command, const std::string &description)
    {
        // Si la condition de combo est vraie, on déclenche l'action
        if (condition)
        {
            if (!is_lock_)
            {
            RCLCPP_INFO(this->get_logger(), "Action combo : %s", description.c_str());
            system(command.c_str());  // Exécution de la commande (ex. : ros2 run go2_basics stand_up)
            } else {
                RCLCPP_WARN(this->get_logger(), "ROBOT VEROUILLE");
            }
        }
    }

    // Fonction de mappage d'une valeur d'un intervalle à un autre
    float map_range(float value, float from_min, float from_max, float to_min, float to_max)
    {
        return (value - from_min) / (from_max - from_min) * (to_max - to_min) + to_min;
    }

    // === Interfaces ROS 2 ===
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr move_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

    // === Variables internes ===
    float last_vx_ = 0.0f, last_vy_ = 0.0f, last_vz_ = 0.0f;
    float last_pitch_ = 0.0f, last_roll_ = 0.0f, last_yaw_ = 0.0f;
    bool last_movement_was_zero_ = false;

    // === Variables des boutons ===
    bool lt_pressed_ = false, rt_pressed_ = false;
    bool lb_pressed_ = false, rb_pressed_ = false;
    bool x_pressed_ = false, a_pressed_ = false, b_pressed_ = false, y_pressed_ = false;
    bool select_pressed_ = false, start_pressed_ = false;
    bool left_stick_pressed_ = false, right_stick_pressed_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);  // Initialisation de ROS 2
    auto node = std::make_shared<MoveRobotNode>();  // Création du noeud
    rclcpp::spin(node);  // Lancement de l'exécution du noeud
    rclcpp::shutdown();  // Arrêt de ROS 2
    return 0;
}
