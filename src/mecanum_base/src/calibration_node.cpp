// === Inclusioni ROS 2 ===
#include <rclcpp/rclcpp.hpp>                    // Base per nodi ROS 2
#include <std_msgs/msg/float64_multi_array.hpp> // Per inviare comandi ai motori
#include <std_msgs/msg/string.hpp>              // Per pubblicare messaggi di stato
#include <sensor_msgs/msg/joint_state.hpp>      // Per ricevere velocità delle ruote

// === Inclusione del servizio definito nel pacchetto mecanum_base ===
#include "mecanum_base/srv/calibration_input.hpp"

// === Librerie standard C++ ===
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <numeric>
#include <cmath>
#include <algorithm>

#include <filesystem>           // Per gestire directory e file system
namespace fs = std::filesystem; // Alias per semplificare l'uso

class CalibrationNode : public rclcpp::Node
{
public:
  CalibrationNode() : Node("calibration_node")
  {
    // === Parametri robot letti da file di configurazione o launch ===
    r_ = declare_parameter<double>("wheel_radius", 0.05);
    L_ = declare_parameter<double>("L", 0.15);
    W_ = declare_parameter<double>("W", 0.15);
    controller_topic_ = declare_parameter<std::string>("controller_cmd_topic", "/mecanum_velocity_controller/commands");

    // === Inizializzazione dei vettori di stato ===
    wheel_offset_ = {0.0, 0.0, 0.0, 0.0};
    last_wheel_velocity_ = {0.0, 0.0, 0.0, 0.0};

    // === Carica correzioni precedenti da file YAML ===
    // loadCorrectionFromFile();

    // === Publisher per comandi ai motori e stato della calibrazione ===
    pub_cmd_ = create_publisher<std_msgs::msg::Float64MultiArray>(controller_topic_, 10);
    calib_pub_ = create_publisher<std_msgs::msg::String>("/calibration_status", 10);

    // === Subscriber per ricevere velocità delle ruote ===
    sub_joint_states_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&CalibrationNode::jointStatesCb, this, std::placeholders::_1));

    // === Servizio per avviare test di calibrazione ===
    calib_srv_ = create_service<mecanum_base::srv::CalibrationInput>(
        "/run_calibration_test",
        std::bind(&CalibrationNode::handleCalibrationRequest, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "✅ Nodo di calibrazione avviato.");
  }

private:
  // === Modalità di calibrazione disponibili ===
  enum class CalibrationMode
  {
    MIN_PWM,
    MAX_PWM,
    MAP_PWM
  };

  // === Callback per ricevere velocità delle ruote dal topic /joint_states ===
  void jointStatesCb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::map<std::string, double> joint_vel_map;
    for (size_t i = 0; i < msg->name.size(); ++i)
      joint_vel_map[msg->name[i]] = msg->velocity[i];

    last_wheel_velocity_ = {
        joint_vel_map["wheel_fl_joint"],
        joint_vel_map["wheel_fr_joint"],
        joint_vel_map["wheel_rl_joint"],
        joint_vel_map["wheel_rr_joint"]};
  }

  // === Esegue test di calibrazione PWM e raccoglie velocità medie delle ruote ===
  void runCalibrationTest(CalibrationMode mode, double step, std::vector<double> &result)
  {
    result.clear();
    std::ostringstream log;
    log << "🧪 Avvio test: ";
    if (mode == CalibrationMode::MIN_PWM)
      log << "Impulso minimo\n";
    else if (mode == CalibrationMode::MAX_PWM)
      log << "Impulso massimo\n";
    else
      log << "Mappatura impulso-velocità\n";

    calib_pub_->publish(std_msgs::msg::String().set__data(log.str()));

    for (int pwm = 50; pwm <= 255; pwm += static_cast<int>(step))
    {
      std_msgs::msg::Float64MultiArray cmd;
      cmd.data = {static_cast<double>(pwm), static_cast<double>(pwm),
                  static_cast<double>(pwm), static_cast<double>(pwm)};

      pub_cmd_->publish(cmd);
      rclcpp::sleep_for(std::chrono::milliseconds(300));

      double avg = std::accumulate(last_wheel_velocity_.begin(), last_wheel_velocity_.end(), 0.0) / 4.0;
      result.push_back(avg);

      std::ostringstream step_log;
      step_log << "PWM: " << pwm << " → Velocità media: " << avg << " rad/s\n";
      calib_pub_->publish(std_msgs::msg::String().set__data(step_log.str()));
    }

    calib_pub_->publish(std_msgs::msg::String().set__data("✅ Test completato.\n"));
  }

  // === Calcola vettore di correzione cinematico in base all’errore misurato ===
  std::vector<double> computeCorrectionVector(double vx, double vy, double wz, double ex, double ey, double ew)
  {
    const double a = L_ + W_;
    std::vector<std::vector<double>> M = {
        {1, -1, -a},
        {1, 1, a},
        {1, 1, -a},
        {1, -1, a}};

    double total = std::abs(vx) + std::abs(vy) + std::abs(wz);
    double wx = total > 0 ? std::abs(vx) / total : 0;
    double wy = total > 0 ? std::abs(vy) / total : 0;
    double ww = total > 0 ? std::abs(wz) / total : 0;

    std::vector<double> e = {wx * ex, wy * ey, ww * ew};
    std::vector<double> correction(4, 0.0);
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 3; ++j)
        correction[i] += M[i][j] * e[j];

    return correction;
  }

  // === Gestisce la richiesta del servizio di calibrazione ===
  void handleCalibrationRequest(
      const std::shared_ptr<mecanum_base::srv::CalibrationInput::Request> req,
      std::shared_ptr<mecanum_base::srv::CalibrationInput::Response> res)
  {
    std::string tipo = req->test_type;
    double valore = req->error_value;
    std::vector<double> result;
    CalibrationMode mode;

    // === Modalità PWM ===
    if (tipo == "min_pwm")
    {
      mode = CalibrationMode::MIN_PWM;
    }
    else if (tipo == "max_pwm")
    {
      mode = CalibrationMode::MAX_PWM;
    }
    else if (tipo == "map_pwm")
    {
      mode = CalibrationMode::MAP_PWM;
    }
    // === Modalità cinematiche ===
    else if (tipo == "rettilineo" || tipo == "strafe" || tipo == "rotazione")
    {
      double ex = 0.0, ey = 0.0, ew = 0.0;

      if (tipo == "rettilineo")
        ex = valore;
      else if (tipo == "strafe")
        ey = valore;
      else if (tipo == "rotazione")
        ew = valore;

      double vx = (tipo == "rettilineo") ? 0.5 : 0.0;
      double vy = (tipo == "strafe") ? 0.5 : 0.0;
      double wz = (tipo == "rotazione") ? 1.0 : 0.0;

      auto correction = computeCorrectionVector(vx, vy, wz, ex, ey, ew);

      // Salva il file con nome specifico per tipo
      saveCorrectionToFile(correction, tipo); // <-- nuovo parametro

      std::stringstream ss;
      ss << "🧪 Test cinematico: " << tipo << "\n";
      ss << "Errore: ex=" << ex << ", ey=" << ey << ", ew=" << ew << "\n";
      ss << "Correzione ruote:\n";
      ss << "FL: " << correction[0] << "\n";
      ss << "FR: " << correction[1] << "\n";
      ss << "RL: " << correction[2] << "\n";
      ss << "RR: " << correction[3] << "\n";

      calib_pub_->publish(std_msgs::msg::String().set__data(ss.str()));
      res->result = ss.str();
      return;
    }
    // === Tipo non valido ===
    else
    {
      res->result = "❌ Tipo di test non riconosciuto.";
      calib_pub_->publish(std_msgs::msg::String().set__data(res->result));
      return;
    }

    // === Esecuzione test PWM ===
    runCalibrationTest(mode, valore, result);

    // === Costruzione del messaggio di risposta con i risultati del test ===
    std::stringstream ss;
    ss << "📊 Risultati test " << tipo << ":\n";
    for (size_t i = 0; i < result.size(); ++i)
      ss << "Step " << i << ": " << result[i] << " rad/s\n";

    // Pubblica i risultati sul topic di stato e li restituisce nella risposta del servizio
    calib_pub_->publish(std_msgs::msg::String().set__data(ss.str()));
    res->result = ss.str();
  }

  // Metodo per salvare il vettore di correzione su file YAML
  void saveCorrectionToFile(const std::vector<double> &correction, const std::string &tipo)
  {
    const std::string dir = "config";
    const std::string filepath = dir + "/correction_" + tipo + ".yaml"; // es. correction_rettilineo.yaml

    if (!fs::exists(dir))
    {
      try
      {
        fs::create_directories(dir);
      }
      catch (const fs::filesystem_error &e)
      {
        RCLCPP_ERROR(get_logger(), "❌ Errore nella creazione della cartella '%s': %s", dir.c_str(), e.what());
        return;
      }
    }

    std::ofstream out(filepath);
    if (out.is_open())
    {
      out << "correction_" << tipo << ": ["
          << correction[0] << ", "
          << correction[1] << ", "
          << correction[2] << ", "
          << correction[3] << "]\n";
      out.close();
      RCLCPP_INFO(get_logger(), "💾 Correzione '%s' salvata in %s", tipo.c_str(), filepath.c_str());
    }
    else
    {
      RCLCPP_WARN(get_logger(), "⚠️ Impossibile scrivere su %s", filepath.c_str());
    }
  }

  // Apertura del file in scrittura
  std::ofstream out(filepath);
  if (out.is_open())
  {
    // Scrittura del vettore nel formato YAML
    out << "wheel_correction: ["
        << correction[0] << ", "
        << correction[1] << ", "
        << correction[2] << ", "
        << correction[3] << "]\n";
    out.close(); // Chiude il file dopo la scrittura

    // Log di conferma
    RCLCPP_INFO(get_logger(), "💾 Correzione salvata in %s", filepath.c_str());
  }
  else
  {
    // Log di avviso se il file non può essere aperto
    RCLCPP_WARN(get_logger(), "⚠️ Impossibile scrivere su %s", filepath.c_str());
  }
}

// === Caricamento del vettore di correzione da file YAML ===
void loadCorrectionFromFile()
{
  // 🔧 Tipi di test cinematici da combinare
  const std::vector<std::string> types = {"rettilineo", "strafe", "rotazione"};

  // ⚖️ Pesi assegnati a ciascun test (equilibrati: somma = 1.0)
  // Puoi modificarli per dare più importanza a un test specifico
  const std::vector<double> weights = {0.33, 0.33, 0.34};

  // 🧮 Vettori parziali per ciascun test
  std::vector<std::vector<double>> partials(3, std::vector<double>(4, 0.0));
  bool loaded_any = false;

  // 📂 Carica ciascun file YAML separato
  for (size_t t = 0; t < types.size(); ++t)
  {
    std::string filepath = "config/correction_" + types[t] + ".yaml";
    std::ifstream in(filepath);

    if (in.is_open())
    {
      std::string line;
      while (std::getline(in, line))
      {
        // Cerca la riga con il nome del vettore
        if (line.find("correction_" + types[t]) != std::string::npos)
        {
          size_t start = line.find("[");
          size_t end = line.find("]");
          if (start != std::string::npos && end != std::string::npos && end > start)
          {
            // Estrae i valori tra parentesi quadre
            std::string values = line.substr(start + 1, end - start - 1);
            std::stringstream ss(values);
            std::string val;
            int i = 0;

            // Converte i valori in double e li salva nel vettore parziale
            while (std::getline(ss, val, ',') && i < 4)
            {
              try
              {
                partials[t][i] = std::stod(val);
              }
              catch (...)
              {
                partials[t][i] = 0.0; // In caso di errore, imposta a zero
              }
              ++i;
            }

            loaded_any = true;
            RCLCPP_INFO(get_logger(), "📂 Correzione '%s' caricata da %s", types[t].c_str(), filepath.c_str());
            break;
          }
        }
      }
      in.close();
    }
    else
    {
      RCLCPP_WARN(get_logger(), "⚠️ File %s non trovato. Correzione '%s' impostata a zero.", filepath.c_str(), types[t].c_str());
    }
  }

  // ➕ Somma pesata dei tre vettori per ottenere la correzione finale
  wheel_offset_.resize(4, 0.0);
  for (int i = 0; i < 4; ++i)
    wheel_offset_[i] = weights[0] * partials[0][i] + weights[1] * partials[1][i] + weights[2] * partials[2][i];

  // ✅ Log finale
  if (loaded_any)
  {
    RCLCPP_INFO(get_logger(), "✅ Correzione totale combinata con pesi: rettilineo=%.2f, strafe=%.2f, rotazione=%.2f",
                weights[0], weights[1], weights[2]);

    // ✨ Esempio di output
    RCLCPP_INFO(get_logger(), "🔎 Offset ruote finali:");
    RCLCPP_INFO(get_logger(), "FL: %.4f", wheel_offset_[0]);
    RCLCPP_INFO(get_logger(), "FR: %.4f", wheel_offset_[1]);
    RCLCPP_INFO(get_logger(), "RL: %.4f", wheel_offset_[2]);
    RCLCPP_INFO(get_logger(), "RR: %.4f", wheel_offset_[3]);
  }
  else
  {
    RCLCPP_WARN(get_logger(), "⚠️ Nessuna correzione caricata. Tutti gli offset impostati a zero.");
  }
}

// === Membri ROS ===
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr calib_pub_;
rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_states_;
rclcpp::Service<mecanum_base::srv::CalibrationInput>::SharedPtr calib_srv_;

// === Parametri robot ===
double r_, L_, W_;
std::string controller_topic_;
std::vector<double> wheel_offset_;
std::vector<double> last_wheel_velocity_;
}
;

// === Funzione main: avvia il nodo ROS 2 ===
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CalibrationNode>());
  rclcpp::shutdown();
  return 0;
}
