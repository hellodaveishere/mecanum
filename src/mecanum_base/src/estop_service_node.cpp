// ==============================
// 📦 Inclusione delle librerie necessarie
// ==============================

#include "rclcpp/rclcpp.hpp"                  // Base per creare nodi ROS 2
#include "std_srvs/srv/set_bool.hpp"          // Servizio standard ROS 2 con un campo booleano
#include "mecanum_system.hpp"                 // Include la tua interfaccia hardware con metodi statici

// ==============================
// 🧠 Definizione del nodo ROS 2
// ==============================

class EstopServiceNode : public rclcpp::Node {
public:
    // Costruttore del nodo
    EstopServiceNode() : Node("estopservicenode") {
        // Creazione del servizio ROS 2 /emergency_stop
        estop_service_ = this->create_service<std_srvs::srv::SetBool>(
            "emergency_stop",  // Nome del servizio
            std::bind(&EstopServiceNode::handle_estop, this, std::placeholders::_1, std::placeholders::_2)
        );

        // Messaggio di log per confermare che il servizio è attivo
        RCLCPP_INFO(this->get_logger(), "✅ Servizio /emergency_stop inizializzato correttamente");
    }

private:
    // Puntatore al servizio ROS 2
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr estop_service_;

    // ==============================
    // 🔁 Callback del servizio
    // ==============================
    void handle_estop(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        // Se il client invia "true", attiva l'emergency stop
        if (request->data) {
            MecanumSystem::activateEmergencyStopGlobal();
            response->message = "Emergency stop attivato";
            RCLCPP_WARN(this->get_logger(), "🛑 Emergency stop attivato da servizio");
        } else {
            // Se invia "false", lo disattiva
            MecanumSystem::clearEmergencyStopGlobal();
            response->message = "Emergency stop disattivato";
            RCLCPP_INFO(this->get_logger(), "🟢 Emergency stop disattivato da servizio");
        }

        // Conferma che l'operazione è andata a buon fine
        response->success = true;
    }
};

// ==============================
// 🚀 Funzione main
// ==============================

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);                                      // Inizializza ROS 2
    rclcpp::spin(std::make_shared<EstopServiceNode>());            // Avvia il nodo e lo mantiene attivo
    rclcpp::shutdown();                                            // Chiude ROS 2 in modo pulito
    return 0;
}
