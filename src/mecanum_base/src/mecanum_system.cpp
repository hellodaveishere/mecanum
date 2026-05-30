#include "mecanum_hardware/mecanum_system.hpp"
#include "pluginlib/class_list_macros.hpp"

#include <cstdio>

#include <sstream>

#include <iostream> // std::cerr

#include <rclcpp/rclcpp.hpp>
#include <fcntl.h>     // open()
#include <unistd.h>    // close(), read(), write()
#include <termios.h>   // struct termios, tcgetattr, tcsetattr
#include <sys/ioctl.h> // ioctl()
#include <cerrno>      // errno
#include <cstring>     // strerror()

#include <optional>
#include <string>
#include <unistd.h> // read()
#include <mutex>
#include <poll.h> // <-- necessario per poll(), struct pollfd, POLLOUT

namespace mecanum_hardware
{

  // =============================
  // Apertura della porta seriale
  // =============================
  bool MecanumSystem::open_serial()
  {
    std::lock_guard<std::mutex> lock(serial_mutex_); // protezione accesso concorrente

    // 1. Apri la porta seriale
    serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Errore apertura seriale %s: %s",
                   serial_port_.c_str(), std::strerror(errno));
      return false;
    }

    // 2. Recupera le impostazioni correnti
    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Errore tcgetattr: %s", std::strerror(errno));
      ::close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }

    // 3. Imposta baud rate
    speed_t baud;
    switch (baudrate_)
    {
    case 9600:
      baud = B9600;
      break;
    case 19200:
      baud = B19200;
      break;
    case 57600:
      baud = B57600;
      break;
    case 115200:
      baud = B115200;
      break;
    case 1000000:
      baud = B1000000;
      break;

    default:
      RCLCPP_WARN(this->get_logger(),
                  "Baudrate %d non standard, uso 115200", baudrate_);
      baud = B115200;
    }
    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    // 4. Configura 8N1
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    // 5. Modalità raw
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;

    // 6. Timeout lettura
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    // 7. Applica le impostazioni
    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Errore tcsetattr: %s", std::strerror(errno));
      ::close(serial_fd_);
      serial_fd_ = -1;
      return false;
    }

    RCLCPP_INFO(this->get_logger(),
                "Porta seriale %s aperta a %d baud",
                serial_port_.c_str(), baudrate_);
    return true;
  }

  // =============================
  // Chiusura della porta seriale
  // =============================
  void MecanumSystem::close_serial()
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);

    if (serial_fd_ >= 0)
    {
      ::close(serial_fd_);
      RCLCPP_INFO(this->get_logger(),
                  "Porta seriale %s chiusa", serial_port_.c_str());
      serial_fd_ = -1;
    }
    else
    {
      RCLCPP_WARN(this->get_logger(),
                  "close_serial chiamato ma la porta non era aperta");
    }
  }

  // =============================
  // Loop di scrittura
  // =============================
  void MecanumSystem::writerloop()
  {
    while (running_writer_.load())
    {
      std::string cmd;
      {
        std::unique_lock<std::mutex> lk(txmutex);
        wxcv_.wait(lk, [&]
                   { return !txqueue.empty() || !running_writer_.load(); });
        if (!running_writer_.load() && txqueue.empty())
          break;
        cmd = std::move(txqueue.front());
        txqueue.pop();
      }
      send_command_(cmd);
    }
  }

  // =============================
  // Loop di lettura
  // =============================
  void MecanumSystem::readerloop()
  {
    RCLCPP_INFO(this->get_logger(), "📡 Reader thread avviato");

    char c;
    std::string buffer;
    bool inside = false;

    // Per limitare log "coda vuota"
    auto last_empty_log = std::chrono::steady_clock::now();

    while (running_reader_.load())
    {
      ssize_t n = ::read(serial_fd_, &c, 1);

      if (n > 0)
      {
        // --- Byte valido ---
        if (c == '^')
        {
          buffer.clear();
          inside = true;
        }
        else if (c == '$' && inside)
        {
          // Messaggio completo → push + log
          {
            std::lock_guard<std::mutex> lk(rxmutex);
            rxqueue.push(buffer);
            rxcv_.notify_one();
          }

          //RCLCPP_INFO(this->get_logger(), "📨 Messaggio ricevuto: '%s'", buffer.c_str());
          inside = false;
        }
        else if (inside)
        {
          buffer.push_back(c);
        }
        else
        {
          RCLCPP_WARN(this->get_logger(),
                      "⚠️ Carattere fuori messaggio ignorato: 0x%02X",
                      (unsigned char)c);
        }
      }
      else if (n == 0)
      {
        RCLCPP_WARN(this->get_logger(), "🔌 EOF sulla seriale: dispositivo chiuso");
        break;
      }
      else // n == -1
      {
        if (errno == EAGAIN || errno == EWOULDBLOCK)
        {
          // Nessun dato → log limitato a 1 Hz
          //auto now = std::chrono::steady_clock::now();
          //if (now - last_empty_log > std::chrono::seconds(1))
          //{
          //  RCLCPP_INFO(this->get_logger(), "⏳ Nessun dato disponibile (EAGAIN)");
          //  last_empty_log = now;
          //}

          std::this_thread::sleep_for(std::chrono::milliseconds(2));
          continue;
        }

        if (errno == EINTR)
        {
          RCLCPP_WARN(this->get_logger(), "⏸️ read() interrotta da EINTR → retry");
          continue;
        }

        RCLCPP_WARN(this->get_logger(),
                    "❌ Errore read(): errno=%d (%s)",
                    errno, strerror(errno));
        break;
      }
    }

    if (inside)
    {
      RCLCPP_WARN(this->get_logger(),
                  "⚠️ Thread terminato con messaggio parziale scartato: '%s'",
                  buffer.c_str());
    }

    RCLCPP_INFO(this->get_logger(), "🛑 Reader thread terminato");
  }

  std::optional<std::string> MecanumSystem::read_buffer_()
  {
    std::lock_guard<std::mutex> lk(rxmutex);

    if (rxqueue.empty())
      return std::nullopt;

    auto msg = rxqueue.front();
    rxqueue.pop();

    //RCLCPP_INFO(this->get_logger(), "📬 Messaggio estratto dalla coda: '%s'", msg.c_str());
    return msg;
  }

  // ==========================
  // Funzione send_command_()
  // ==========================

  bool MecanumSystem::send_command_(const std::string &cmd)
  {
    // Protegge l’accesso concorrente alla seriale
    std::lock_guard<std::mutex> lock(serial_mutex_);

    if (serial_fd_ < 0)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "send_command_: porta seriale non aperta");
      return false;
    }

    // Assicura terminatore newline
    std::string payload = cmd;
    if (payload.empty() || payload.back() != '\n')
    {
      payload.push_back('\n');
    }

    const char *buf = payload.c_str();
    size_t to_write = payload.size();
    ssize_t written_total = 0;
    int eagain_retries = 0;

    // Scrittura robusta con poll() per EAGAIN/EWOULDBLOCK
    while (to_write > 0)
    {
      ssize_t n = ::write(serial_fd_, buf + written_total, to_write);
      if (n > 0)
      {
        written_total += n;
        to_write -= static_cast<size_t>(n);
        eagain_retries = 0; // reset su successo
      }
      else if (n < 0 && errno == EINTR)
      {
        continue; // interrotto da segnale, ritenta
      }
      else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
      {
        if (++eagain_retries > 10)
        {
          RCLCPP_ERROR(this->get_logger(),
                       "send_command_: troppi EAGAIN consecutivi");
          return false;
        }
        // Usa poll per attendere disponibilità del buffer di trasmissione
        struct pollfd pfd;
        pfd.fd = serial_fd_;
        pfd.events = POLLOUT;
        int pret = ::poll(&pfd, 1, 5); // timeout 5 ms
        if (pret <= 0)
        {
          // timeout o errore → piccolo sleep per non saturare CPU
          ::usleep(1000); // 1 ms
        }
        continue;
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(),
                     "send_command_: errore write(): %s", std::strerror(errno));
        return false;
      }
    }

    // tcdrain opzionale (configurabile)
    if (enable_tcdrain_)
    {
      if (::tcdrain(serial_fd_) != 0)
      {
        RCLCPP_WARN(this->get_logger(),
                    "send_command_: tcdrain errore: %s", std::strerror(errno));
      }
    }

    RCLCPP_DEBUG(this->get_logger(),
                 "send_command_: scritto '%s' (%zd byte)",
                 payload.c_str(), static_cast<ssize_t>(written_total));
    return true;
  }

  // ================== INIT ==================
  //
  // Inizializza parametri hardware leggendo dal file URDF/Xacro
  // e prepara le strutture dati dei giunti.
  //
  hardware_interface::CallbackReturn MecanumSystem::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    info_ = info;

    last_stamp_.clear();
    first_stamp_.clear();

    // Helper per leggere parametri dal file URDF
    auto getP = [&](const char *key, auto &var)
    {
      auto it = info_.hardware_parameters.find(key);
      if (it != info_.hardware_parameters.end())
      {
        using T = std::decay_t<decltype(var)>;
        const auto &val = it->second;
        if constexpr (std::is_same_v<T, double>)
          var = std::stod(val);
        else if constexpr (std::is_same_v<T, int>)
          var = std::stoi(val);
        else if constexpr (std::is_same_v<T, bool>)
        {
          std::string v = val;
          std::transform(v.begin(), v.end(), v.begin(), ::tolower);
          var = (v == "true" || v == "1");
        }
        else if constexpr (std::is_same_v<T, std::string>)
          var = val;
      }
    };

    try
    {
      getP("wheel_radius", wheel_radius_);
      getP("L", L_);
      getP("W", W_);
      getP("mock", mock_);
      getP("accel_limit", accel_limit_);
      getP("ticks_per_rev", ticks_per_rev_);
      getP("gear_ratio", gear_ratio_);
      getP("invert_fl", inv_fl_);
      getP("invert_fr", inv_fr_);
      getP("invert_rl", inv_rl_);
      getP("invert_rr", inv_rr_);
      getP("serial_port", serial_port_);
      getP("baudrate", baudrate_);
    }
    catch (...)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Errore parsing parametri hardware");
      return hardware_interface::CallbackReturn::ERROR;
    }

    ticks_per_wheel_rev_ = static_cast<double>(ticks_per_rev_) * gear_ratio_;

    // Verifica che ci siano esattamente 4 giunti
    if (info_.joints.size() != 4)
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Attesi 4 giunti, trovati %zu", info_.joints.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Inizializza i giunti
    joint_names_.clear();
    joints_.assign(info_.joints.size(), JointState{});

    for (const auto &j : info_.joints)
    {
      joint_names_.push_back(j.name);
      bool ok_pos = false, ok_vel = false, ok_cmd = false;

      for (const auto &si : j.state_interfaces)
      {
        if (si.name == hardware_interface::HW_IF_POSITION)
          ok_pos = true;
        if (si.name == hardware_interface::HW_IF_VELOCITY)
          ok_vel = true;
      }
      for (const auto &ci : j.command_interfaces)
      {
        if (ci.name == hardware_interface::HW_IF_VELOCITY)
          ok_cmd = true;
      }

      if (!ok_pos || !ok_vel || !ok_cmd)
      {
        RCLCPP_ERROR(this->get_logger(),
                     "Giunto %s: servono state(position,velocity) + command(velocity)",
                     j.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    RCLCPP_INFO(this->get_logger(),
                "Init OK (mock=%s, serial=%s, baud=%d)",
                mock_ ? "true" : "false",
                serial_port_.c_str(), baudrate_);

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // ================== EXPORT INTERFACES ==================

  std::vector<hardware_interface::StateInterface> MecanumSystem::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> out;
    out.reserve(joints_.size() * 2);

    for (size_t i = 0; i < joints_.size(); ++i)
    {
      out.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &joints_[i].pos_rad);
      out.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &joints_[i].vel_rad_s);
    }

    // Stato IMU (orientamento, velocità angolare, accelerazione lineare)
    out.emplace_back("imu", "orientation.x", &imu_state_.orientation[0]);
    out.emplace_back("imu", "orientation.y", &imu_state_.orientation[1]);
    out.emplace_back("imu", "orientation.z", &imu_state_.orientation[2]);
    out.emplace_back("imu", "orientation.w", &imu_state_.orientation[3]);

    out.emplace_back("imu", "angular_velocity.x", &imu_state_.angular_vel[0]);
    out.emplace_back("imu", "angular_velocity.y", &imu_state_.angular_vel[1]);
    out.emplace_back("imu", "angular_velocity.z", &imu_state_.angular_vel[2]);

    out.emplace_back("imu", "linear_acceleration.x", &imu_state_.linear_accel[0]);
    out.emplace_back("imu", "linear_acceleration.y", &imu_state_.linear_accel[1]);
    out.emplace_back("imu", "linear_acceleration.z", &imu_state_.linear_accel[2]);

    // Stato dei sensori IR frontali
    out.emplace_back("ir_front_left", "range", &ir_state_.ir_front_left);
    out.emplace_back("ir_front_center", "range", &ir_state_.ir_front_center);
    out.emplace_back("ir_front_right", "range", &ir_state_.ir_front_right);

    // ================== Interfacce hardware per i servomotori pan e tilt ==================

    // 📤 Interfacce di stato: esportano la posizione attuale dei giunti (letti dal sensore o simulati)
    // Queste vengono utilizzate dal joint_state_broadcaster per pubblicare su /joint_states
    out.emplace_back(hardware_interface::StateInterface(
        "servo_pan_joint", "position", &servo_state_.pan_position)); // Posizione attuale del servo pan
    out.emplace_back(hardware_interface::StateInterface(
        "servo_tilt_joint", "position", &servo_state_.tilt_position)); // Posizione attuale del servo tilt

    out.emplace_back(hardware_interface::StateInterface("battery_state", "voltage", &battery_state_.voltage));
    // out.emplace_back(hardware_interface::StateInterface("battery_state", "current", &battery_state_.current));
    // out.emplace_back(hardware_interface::StateInterface("battery_state", "temperature", &battery_state_.temperature));
    // out.emplace_back(hardware_interface::StateInterface("battery_state", "charge", &battery_state_.charge));
    // out.emplace_back(hardware_interface::StateInterface("battery_state", "capacity", &battery_state_.capacity));
    out.emplace_back(hardware_interface::StateInterface("battery_state", "percentage", &battery_state_.percentage));

    // Definisci la state interface "mecanum_base/estop_active"
    out.emplace_back(
        hardware_interface::StateInterface(
            "estop_sensor",      // deve coincidere con il nome nel URDF
            "estop_active",      // deve coincidere con il nome nel URDF
            &estop_active_state_ // variabile double che contiene lo stato
            ));

    RCLCPP_DEBUG(rclcpp::get_logger("BatteryDebug"), "Exporting interface: percentage @ %p", &battery_state_.percentage);

    return out;
  }

  std::vector<hardware_interface::CommandInterface> MecanumSystem::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> out;
    out.reserve(joints_.size() + 2); // 4 ruote + 2 servomotori

    // 🎯 Interfacce di comando per le ruote (velocity)
    for (size_t i = 0; i < joints_.size(); ++i)
    {
      out.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &joints_[i].cmd_vel);
    }

    // 🎯 Interfacce di comando per i servomotori pan e tilt (position)
    out.emplace_back(hardware_interface::CommandInterface(
        "servo_pan_joint", "position", &servo_command_.pan_position)); // Comando di posizione per il servo pan

    out.emplace_back(hardware_interface::CommandInterface(
        "servo_tilt_joint", "position", &servo_command_.tilt_position)); // Comando di posizione per il servo tilt

    return out;
  }

  // ================== LIFECYCLE ==================

  hardware_interface::CallbackReturn MecanumSystem::on_activate(const rclcpp_lifecycle::State &)
  {
    // Reset stato giunti
    for (auto &j : joints_)
    {
      j.pos_rad = 0.0;
      j.vel_rad_s = 0.0;
      j.cmd_vel = 0.0;
    }

    if (!mock_)
    {
      if (!open_serial())
      {
        RCLCPP_ERROR(rclcpp::get_logger("MecanumSystem"),
                     "Impossibile aprire la porta seriale %s", serial_port_.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    // Avvia thread di lettura
    running_reader_.store(true);
    readerthread = std::thread(&MecanumSystem::readerloop, this);

    // Avvia thread di scrittura
    running_writer_.store(true);
    writerthread = std::thread(&MecanumSystem::writerloop, this);

    RCLCPP_INFO(rclcpp::get_logger("MecanumSystem"),
                "MecanumSystem attivato: thread reader+writer avviati");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn MecanumSystem::on_deactivate(const rclcpp_lifecycle::State &)
  {
    // Ferma thread di lettura
    running_reader_.store(false);
    if (readerthread.joinable())
    {
      readerthread.join();
    }

    // Ferma thread di scrittura
    running_writer_.store(false);
    wxcv_.notify_all(); // sveglia writerloop se in attesa
    if (writerthread.joinable())
    {
      writerthread.join();
    }

    if (!mock_)
    {
      close_serial();
    }

    RCLCPP_INFO(rclcpp::get_logger("MecanumSystem"),
                "MecanumSystem disattivato: thread reader+writer chiusi");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  
  // Definizione delle delle tre funzioni helper per monitorare il possibile timout dell'arrivo dei dati dei sensori
  std::string MecanumSystem::sensor_type_to_string(SensorType type) const
  {
      switch (type)
      {
          case SensorType::UART_GLOBAL: return "UART_GLOBAL";
          case SensorType::ENC:         return "ENC";
          case SensorType::IMU:         return "IMU";
          case SensorType::IRS:         return "IRS";
          case SensorType::SERVO:       return "SERVO";
          case SensorType::BAT:         return "BAT";
          case SensorType::EMR:         return "EMR";
          default:                      return "UNKNOWN";
      }
  }

  void MecanumSystem::update_sensor_timestamp(SensorType type, const rclcpp::Time& t)
  {
      // Se il sensore era in timeout → ora è ripristinato
      if (warned_[type])
      {
          RCLCPP_INFO(
              this->get_logger(),
              "Data flow restored for sensor: %s",
              sensor_type_to_string(type).c_str()
          );
      }

      // Aggiorna timestamp e resetta warning
      last_update_[type] = t;
      warned_[type] = false;
  }

  void MecanumSystem::register_framing_error(const rclcpp::Time& t)
  {
      // Se prima c’erano errori e ora non più → flusso ripristinato
      if (framing_warned_)
      {
          RCLCPP_INFO(
              this->get_logger(),
              "UART framing stabilized: data flow restored"
          );
      }

      framing_errors_++;
      last_framing_error_ = t;
      framing_warned_ = false;   // reset anti-spam
  }

  void MecanumSystem::check_sensor_timeouts(const rclcpp::Time& now)
  {
      // --- Timeout sensori ---
      for (const auto& kv : timeout_sec_)
      {
          SensorType type = kv.first;
          double timeout = kv.second;

          if (last_update_.find(type) == last_update_.end())
              continue; // mai ricevuto nulla

          double silence = (now - last_update_[type]).seconds();

          if (silence > timeout && !warned_[type])
          {
              switch (type)
              {
                  case SensorType::UART_GLOBAL:
                      RCLCPP_WARN(this->get_logger(),
                          "No sensor data published! Missing UART connectivity?");
                      break;

                  case SensorType::ENC:
                      RCLCPP_WARN(this->get_logger(),
                          "ENC timeout: no encoder data for %.2f s", silence);
                      break;

                  case SensorType::IMU:
                      RCLCPP_WARN(this->get_logger(),
                          "IMU timeout: no IMU data for %.2f s", silence);
                      break;

                  case SensorType::IRS:
                      RCLCPP_WARN(this->get_logger(),
                          "IRS timeout: no IR sensor data for %.2f s", silence);
                      break;

                  case SensorType::SERVO:
                      RCLCPP_WARN(this->get_logger(),
                          "SERVO timeout: no servo feedback for %.2f s", silence);
                      break;

                  case SensorType::BAT:
                      RCLCPP_WARN(this->get_logger(),
                          "BAT timeout: no battery telemetry for %.2f s", silence);
                      break;

                  case SensorType::EMR:
                      RCLCPP_WARN(this->get_logger(),
                          "EMR timeout: no emergency-stop updates for %.2f s", silence);
                      break;
              }

              warned_[type] = true;
          }
      }

      // --- Timeout errori di framing UART ---
      if (framing_errors_ > 10 && !framing_warned_)
      {
          double dt = (now - last_framing_error_).seconds();
          if (dt < framing_window_sec_)
          {
              RCLCPP_WARN(this->get_logger(),
                  "High UART framing error rate: %d errors in last %.1f s. Noise or wrong baud?",
                  framing_errors_, framing_window_sec_);
              framing_warned_ = true;
          }
      }

      // Reset finestra errori
      if ((now - last_framing_error_).seconds() > framing_window_sec_)
      {
          framing_errors_ = 0;
          framing_warned_ = false;
      }
  }

  // ================== READ ==================
  //
  // Legge i pacchetti dalla seriale e aggiorna encoder + IMU.
  //
  hardware_interface::return_type MecanumSystem::read(
      const rclcpp::Time &time, const rclcpp::Duration &period)
  {
    // 1) Tempo trascorso dall’ultimo ciclo (fornito da ros2_control).
    //    Serve per calcolare la velocità media come dpos/dt.
    const double dt = period.seconds();

    // controllo jitter/overload
    if (dt > dt_error_threshold_ && !dt_error_warned_) {
        RCLCPP_ERROR(this->get_logger(),
            "ros2_control loop too slow: dt=%.3f s (CPU overload?)", dt);
        dt_error_warned_ = true;
    }
    else if (dt > dt_warn_threshold_ && !dt_warned_) {
        RCLCPP_WARN(this->get_logger(),
            "ros2_control loop jitter: dt=%.3f s", dt);
        dt_warned_ = true;
    }

    if (dt <= dt_warn_threshold_) {
        dt_warned_ = false;
        dt_error_warned_ = false;
    }
    ////////////////

    // 1.1) Sanity check: evita divisioni per zero o valori non sensati.
    if (dt <= 0.0)
    {
      RCLCPP_WARN(this->get_logger(),
                  "Periodo dt non valido (dt=%.6f).", dt);
    }

    // 2) Modalità mock: salta la seriale e simula la dinamica.
    if (mock_)
    {
      apply_mock_dynamics_(dt);
      RCLCPP_DEBUG(this->get_logger(),
                   "Mock dynamics applied (dt=%.3f)", dt);
      return hardware_interface::return_type::OK;
    }

    // 3) Legge un buffer dalla seriale (può contenere più pacchetti separati da '\n').
    auto buffer = read_buffer_();

    // 3.1) Se non arriva nulla in questo ciclo, controlla il timeout UART e poi ritorna OK.
    //      (Può capitare per jitter o rate diversi tra MCU e PC)
    if (!buffer || buffer->empty())
    {
        check_sensor_timeouts(time);
        return hardware_interface::return_type::OK;
    }

    // 3.2) Suddividi il buffer in righe
    std::stringstream ss(buffer.value()); // estrai la stringa dall’optional

    update_sensor_timestamp(SensorType::UART_GLOBAL, time);

    std::string line;
    while (std::getline(ss, line, '\n'))
    {
      if (line.empty())
        continue; // ignora righe vuote

      // 3.3) Log della riga grezza ricevuta (utile per diagnosi di framing/formato).

      long ms = time.nanoseconds() / 1000000;

      // Extract message type (IMU, ENC, IRS, SER, BAT, EMR)
      std::string msg_type = line.substr(0, line.find(','));

      // Initialize first time
      if (first_stamp_.find(msg_type) == first_stamp_.end())
      {
        first_stamp_[msg_type] = true;
      }

      if (!first_stamp_[msg_type])
      {
        double dt = (time - last_stamp_[msg_type]).seconds();
        double dt_ms = dt * 1000.0;
        double freq = 1.0 / dt;

        //RCLCPP_INFO(
        //    this->get_logger(),
        //    "[%ld ms] UART %s   dt=%.2f ms   f=%.2f Hz",
        //    ms,
        //    msg_type.c_str(),
        //    dt_ms,
        //    freq);
      }
      else
      {
        first_stamp_[msg_type] = false;
      }

      // Update timestamp for this message type
      last_stamp_[msg_type] = time;

      // 4) Dispatch in base al prefisso del pacchetto.
      //    Usiamo rfind(...,0) per verificare che la stringa inizi con il prefisso.
      //
      if (line.rfind("ENC", 0) == 0) // rfind("ENC", 0) == 0 → la stringa inizia con "ENC"
      {
        try
        {
          update_sensor_timestamp(SensorType::ENC, time);

          // Delego il parsing alla funzione dedicata.
          // Se il pacchetto è ben formato, aggiornerà:
          //   joints_[i].pos_rad
          //   joints_[i].vel_rad_s
          parse_encoder_packet_(line);
        }
        catch (const std::exception &e)
        {
          register_framing_error(time);

          // In caso di eccezioni non gestite (es. std::bad_alloc, ecc.),
          // logghiamo un warning per non far crashare il nodo.
          // logger_ NON è accessibile → usare get_logger()
          RCLCPP_WARN(rclcpp::get_logger("MecanumSystem"), "Eccezione durante il parsing ENC: %s", e.what());
        }
      }

      else if (line.rfind("IMU", 0) == 0)
      {
        // --- gestione pacchetto IMU ---
        try
        {
          update_sensor_timestamp(SensorType::IMU, time);

          parse_imu_packet_(line);
        }
        catch (const std::exception &e)
        {
          register_framing_error(time);

          RCLCPP_WARN(this->get_logger(),
                      "Pacchetto IMU malformato, scartato. Errore: %s | Riga: '%s'",
                      e.what(), line.c_str());
        }
      }
      else if (line.rfind("IRS", 0) == 0)
      {
        // 📦 Parsing dei dati IR: formato atteso "IRS,ir_left,ir_center,ir_right"
        try
        {
          update_sensor_timestamp(SensorType::IRS, time);

          std::vector<std::string> tokens;
          std::stringstream ss_ir(line);
          std::string item;
          while (std::getline(ss_ir, item, ','))
            tokens.push_back(item);

          if (tokens.size() != 4)
            throw std::runtime_error("Pacchetto IR malformato: numero errato di campi");

          auto parse_ir_value = [](const std::string &s, const std::string &label) -> double
          {
            if (s == "None")
            {
              RCLCPP_WARN(rclcpp::get_logger("MecanumSystem"),
                          "Sensore IR '%s' fuori copertura o troppo vicino: valore 'None' ricevuto", label.c_str());
              return -1.0;
            }
            return std::stod(s);
          };

          ir_state_.ir_front_left = parse_ir_value(tokens[1], "front_left");
          ir_state_.ir_front_center = parse_ir_value(tokens[2], "front_center");
          ir_state_.ir_front_right = parse_ir_value(tokens[3], "front_right");
        }
        catch (const std::exception &e)
        {
          register_framing_error(time);

          RCLCPP_WARN(rclcpp::get_logger("MecanumSystem"),
                      "Pacchetto IR SENSORS malformato, scartato. Errore: %s | Riga: '%s'",
                      e.what(), line.c_str());
        }
      }
      else if (line.rfind("SER", 0) == 0) // 🔍 Riconoscimento del pacchetto servo: inizia con "SER"
      {
        try
        {
          update_sensor_timestamp(SensorType::SERVO, time);

          std::vector<std::string> tokens;
          std::stringstream ss_ser(line);
          std::string item;
          while (std::getline(ss_ser, item, ','))
            tokens.push_back(item);

          if (tokens.size() != 3)
            throw std::runtime_error("Pacchetto SERVO malformato: numero errato di campi");

          auto parse_servo_value = [](const std::string &s, const std::string &label) -> double
          {
            if (s == "None")
            {
              RCLCPP_WARN(rclcpp::get_logger("MecanumSystem"),
                          "Servo '%s' ha restituito 'None': posizione non disponibile", label.c_str());
              return -1.0;
            }
            return std::stod(s);
          };

          servo_state_.pan_position = parse_servo_value(tokens[1], "pan");
          servo_state_.tilt_position = parse_servo_value(tokens[2], "tilt");
        }
        catch (const std::exception &e)
        {
          register_framing_error(time);

          RCLCPP_WARN(rclcpp::get_logger("MecanumSystem"),
                      "Pacchetto SERVO malformato, scartato. Errore: %s | Riga: '%s'",
                      e.what(), line.c_str());
        }
      }
      else if (line.rfind("BAT", 0) == 0)
      {
        try
        {
          update_sensor_timestamp(SensorType::BAT, time);

          std::istringstream ss_bat(line);
          std::string token;
          std::vector<std::string> fields;
          while (std::getline(ss_bat, token, ','))
            fields.push_back(token);

          if (fields.size() != 3)
            throw std::runtime_error("Formato BAT non valido");

          float voltage = std::stof(fields[1]); // Converte la tensione da stringa a float
          float percent = std::stof(fields[2]); // Converte la percentuale da stringa a float

          // Aggiorna lo stato della batteria
          battery_state_.voltage = static_cast<double>(voltage);
          battery_state_.percentage = static_cast<double>(percent) / 100.0;
        }
        catch (const std::exception &e)
        {
          register_framing_error(time);

          RCLCPP_WARN(this->get_logger(),
                      "Pacchetto BAT malformato, scartato. Errore: %s | Riga: '%s'",
                      e.what(), line.c_str());
        }
      }
      else if (line.rfind("LOG", 0) == 0)
      {
        RCLCPP_INFO(this->get_logger(),
                    "Pico log: %s", line.c_str());
      }
      // Verifica se la riga ricevuta inizia con il prefisso "EMR:" (Emergency Stop)
      else if (line.rfind("EMR", 0) == 0)
      {
        try{
          update_sensor_timestamp(SensorType::EMR, time);

          std::string value = line.substr(4);
          value.erase(std::remove_if(value.begin(), value.end(), ::isspace), value.end());

          bool new_state = (value == "true");
          estop_active_state_ = new_state ? 1.0 : 0.0;

          // Stampa log solo se cambia stato
          static bool last_state = false;
          if (new_state != last_state)
          {
            if (new_state)
            {
              RCLCPP_WARN_STREAM(rclcpp::get_logger("MecanumSystem"),
                                "Emergency stop ATTIVO (EMR: " << value << ")");
            }
            else
            {
              RCLCPP_INFO_STREAM(rclcpp::get_logger("MecanumSystem"),
                                "Emergency stop DISATTIVO (EMR: " << value << ")");
            }
            last_state = new_state;
          }
        }
        catch (const std::exception &e)
        {
          register_framing_error(time);

          RCLCPP_WARN(this->get_logger(),
                      "Pacchetto EMR malformato, scartato. Errore: %s | Riga: '%s'",
                      e.what(), line.c_str());
        }
      }
      else
      {
        // 6) Prefisso sconosciuto: il pacchetto non appartiene ai formati attesi.
        //    Lo segnaliamo ma non è un errore critico (potrebbe essere rumore o debug lato MCU).
        register_framing_error(time);

        RCLCPP_WARN(this->get_logger(),
                    "Pacchetto con prefisso sconosciuto: %s", line.c_str());
      }
    } // fine while getline

    check_sensor_timeouts(time);

    // 7) Concludi regolarmente: anche se non è arrivato nulla o si è scartato un pacchetto,
    //    il ciclo di controllo continua senza disattivare l'hardware.
    return hardware_interface::return_type::OK;
  }

  // ================== WRITE ==================
  //
  // Invia i comandi motori e servomotori in formato CSV.
  // Formato: "CMD,FL,FR,RL,RR,PAN,TILT"
  //
  // ==========================
  // Funzione write()
  // ==========================

  hardware_interface::return_type MecanumSystem::write(
      const rclcpp::Time &, const rclcpp::Duration &)
  {
    // 🔒 Sicurezza aggiuntiva:
    // In caso di E‑STOP (emergency stop) si potrebbero azzerare i comandi ai motori.
    // Questo blocco è già gestito da EstopManagerNode, quindi qui è commentato.
    /*
    if (estop_active_state_ > 0.5)
    {
        for (auto &ci : command_interfaces_)
        {
            ci.set_value(0.0);
        }
        RCLCPP_WARN(rclcpp::get_logger("MecanumSystem"), "Comandi azzerati per E-STOP");
        return hardware_interface::return_type::OK;
    }
    */

    // 1️⃣ Caso simulazione (mock):
    // Se mock_ è true, significa che stiamo simulando il sistema hardware.
    // In questo caso non inviamo nulla sulla seriale, ma ritorniamo OK.
    if (mock_)
    {
      return hardware_interface::return_type::OK;
    }

    // 2️⃣ Validazione dei valori:
    // Evitiamo di inviare valori non validi (NaN/Inf) al firmware.
    for (auto &j : joints_)
    {
      if (!std::isfinite(j.cmd_vel))
      {
        RCLCPP_ERROR(this->get_logger(), "write(): comando non valido (NaN/Inf)");
        return hardware_interface::return_type::ERROR;
      }
    }
    if (!std::isfinite(servo_command_.pan_position) ||
        !std::isfinite(servo_command_.tilt_position))
    {
      RCLCPP_ERROR(this->get_logger(), "write(): comando servo non valido (NaN/Inf)");
      return hardware_interface::return_type::ERROR;
    }

    // 3️⃣ Costruzione del comando CSV:
    // Formattiamo i valori dei giunti e dei servo in una stringa CSV.
    // Usiamo 4 decimali per coerenza con il logging lato firmware.
    std::string csv;
    csv.reserve(64);
    csv = "CMD," +
          fmt::format("{:.4f},{:.4f},{:.4f},{:.4f},{:.4f},{:.4f}",
                      joints_[0].cmd_vel, joints_[1].cmd_vel,
                      joints_[2].cmd_vel, joints_[3].cmd_vel,
                      servo_command_.pan_position, servo_command_.tilt_position);

    // 4️⃣ Frequenza di invio: riduci la chiamata effettiva a 10 Hz (ogni 5 cicli)
    static int cycle_counter = 0;
    cycle_counter = (cycle_counter + 1) % 5; // contatore ciclico 0–4

    if (cycle_counter == 0)
    {
      // Capacità massima della coda di trasmissione.
      constexpr size_t MAX_TXQUEUE_SIZE = 10;

      // Inserisci il comando nella txqueue in modo NON BLOCCANTE per il ciclo real-time.
      {
        std::unique_lock<std::mutex> lk(txmutex);

        if (txqueue.size() >= MAX_TXQUEUE_SIZE)
        {
          // Politica: scarta il più vecchio per fare spazio al comando più recente.
          txqueue.pop();
          RCLCPP_WARN(this->get_logger(),
                      "txqueue piena (>%zu): scartato comando più vecchio per inserire il nuovo",
                      MAX_TXQUEUE_SIZE);
        }

        txqueue.push(csv);
      } // rilascio txmutex qui

      // Notifica il writer thread (non-blocking, veloce)
      wxcv_.notify_one();
    }

    // 5️⃣ Logging (opzionale):
    // Possiamo loggare il comando inviato solo se è diverso dall’ultimo loggato.
    // Questo riduce il rumore nei log. Attualmente è commentato.
    /*
    static std::string last_logged_csv;
    if (last_logged_csv != csv)
    {
        RCLCPP_INFO(this->get_logger(),
                    "write() cmd: FL=%.3f FR=%.3f RL=%.3f RR=%.3f PAN=%.3f TILT=%.3f",
                    joints_[0].cmd_vel, joints_[1].cmd_vel,
                    joints_[2].cmd_vel, joints_[3].cmd_vel,
                    servo_command_.pan_position, servo_command_.tilt_position);
        last_logged_csv = csv;
    }
    */

    // ✅ Se tutto è andato bene, ritorniamo OK.
    return hardware_interface::return_type::OK;
  }

  // ================== MOCK DYNAMICS ==================
  //
  // Simula la dinamica delle ruote in assenza di hardware reale.
  // Applica un modello di primo ordine con limite di accelerazione.
  //
  void MecanumSystem::apply_mock_dynamics_(double dt)
  {
    // Calcola la variazione massima di velocità consentita in questo step
    const double dv_max = accel_limit_ * dt;

    // Itera su ciascun giunto (ruota)
    for (auto &j : joints_)
    {
      // Differenza tra comando desiderato e velocità attuale
      double dv = j.cmd_vel - j.vel_rad_s;

      // Limita la variazione di velocità per simulare inerzia/accelerazione massima
      if (dv > dv_max)
        dv = dv_max;
      else if (dv < -dv_max)
        dv = -dv_max;

      // Aggiorna la velocità simulata
      j.vel_rad_s += dv;

      // Aggiorna la posizione integrando la velocità
      j.pos_rad += j.vel_rad_s * dt;
    }
  }

  // ================== PARSING PACCHETTI ==================
  //
  /**
   * @brief Parsing del pacchetto encoder proveniente dal microcontrollore.
   *
   * Formato atteso della stringa "line":
   *
   *   ENC,pos_fl,pos_fr,pos_rl,pos_rr,vel_fl,vel_fr,vel_rl,vel_rr
   *
   * Dove:
   *   - pos_xx = posizione cumulativa encoder (in tick, lato microcontrollore)
   *   - vel_xx = velocità filtrata (in rad/s, lato microcontrollore)
   *
   * Obiettivo:
   *   - Aggiornare joints_[i].pos_rad e joints_[i].vel_rad_s
   *     in modo coerente con ros2_control:
   *       pos_rad   → posizione cumulativa in radianti
   *       vel_rad_s → velocità in radianti/secondo
   *
   * NOTE:
   *   - Se in futuro il microcontrollore invierà posizioni già in radianti,
   *     basterà impostare k = 1.0 invece di 2π / ticks_per_wheel_rev_.
   */
  void MecanumSystem::parse_encoder_packet_(const std::string &line)
  {
    // Creiamo uno stringstream per "spezzare" la riga sugli ','.
    std::stringstream ss(line);
    std::string token;

    // 1) Scartiamo il prefisso "ENC"
    //    Esempio: "ENC,123,124,..." → qui leggiamo "ENC" e lo ignoriamo.
    if (!std::getline(ss, token, ','))
    {
      RCLCPP_WARN(rclcpp::get_logger("MecanumSystem"), "Pacchetto ENC vuoto o malformato (manca prefisso)");
      return;
    }

    double pos[4];
    double vel[4];

    // 2) Parsing delle POSIZIONI (pos_fl, pos_fr, pos_rl, pos_rr)
    for (int i = 0; i < 4; ++i)
    {
      if (!std::getline(ss, token, ','))
      {
        // Se non riusciamo a leggere abbastanza campi, il pacchetto è incompleto.
        RCLCPP_WARN(rclcpp::get_logger("MecanumSystem"), "Pacchetto ENC incompleto (posizione %d)", i);

        return;
      }

      try
      {
        pos[i] = std::stod(token); // converte stringa → double
      }
      catch (const std::exception &e)
      {
        RCLCPP_WARN(rclcpp::get_logger("MecanumSystem"), "Errore conversione posizione ENC[%d]: '%s'", i, token.c_str());
        return;
      }
    }

    // 3) Parsing delle VELOCITÀ (vel_fl, vel_fr, vel_rl, vel_rr)
    for (int i = 0; i < 4; ++i)
    {
      if (!std::getline(ss, token, ','))
      {
        RCLCPP_WARN(rclcpp::get_logger("MecanumSystem"), "Pacchetto ENC incompleto (velocità %d)", i);
        return;
      }

      try
      {
        vel[i] = std::stod(token); // velocità già in rad/s
      }
      catch (const std::exception &e)
      {
        RCLCPP_WARN(rclcpp::get_logger("MecanumSystem"), "Errore conversione velocità ENC[%d]: '%s'", i, token.c_str());
        return;
      }
    }

    // 4) Applichiamo eventuali inversioni di segno
    //    (utile se i motori sono montati in verso opposto o gli encoder contano al contrario).
    pos[0] *= inv_fl_;
    vel[0] *= inv_fl_;
    pos[1] *= inv_fr_;
    vel[1] *= inv_fr_;
    pos[2] *= inv_rl_;
    vel[2] *= inv_rl_;
    pos[3] *= inv_rr_;
    vel[3] *= inv_rr_;

    // 5) Conversione da tick → radianti.
    //
    //    ticks_per_wheel_rev_ = tick per un giro completo di ruota.
    //    2π rad = 1 giro → fattore di conversione:
    //
    //      rad = tick * (2π / ticks_per_wheel_rev_)
    //
    //    Se il microcontrollore invia già posizioni in radianti,
    //    imposta semplicemente:
    //
    //      const double k = 1.0;
    //
    const double k = 2.0 * M_PI / ticks_per_wheel_rev_;

    // 6) Aggiorniamo lo stato dei giunti usato da ros2_control
    for (int i = 0; i < 4; ++i)
    {
      // Posizione cumulativa in radianti
      joints_[i].pos_rad = pos[i] * k;

      // Velocità in rad/s (già filtrata dal microcontrollore)
      joints_[i].vel_rad_s = vel[i];
    }
  }

  //
  // Parsing pacchetto IMU: "IMU,qx,qy,qz,qw,gx,gy,gz,ax,ay,az"
  //
  void MecanumSystem::parse_imu_packet_(const std::string &line)
  {
    std::stringstream ss(line);
    std::string token;

    // Scarta il prefisso "IMU"
    std::getline(ss, token, ',');

    double values[10] = {0.0};
    for (int i = 0; i < 10 && std::getline(ss, token, ','); ++i)
    {
      values[i] = std::stod(token);
    }

    // Aggiorna stato IMU
    imu_state_.orientation[0] = values[0];
    imu_state_.orientation[1] = values[1];
    imu_state_.orientation[2] = values[2];
    imu_state_.orientation[3] = values[3];

    imu_state_.angular_vel[0] = values[4];
    imu_state_.angular_vel[1] = values[5];
    imu_state_.angular_vel[2] = values[6];

    imu_state_.linear_accel[0] = values[7];
    imu_state_.linear_accel[1] = values[8];
    imu_state_.linear_accel[2] = values[9];
  }
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    mecanum_hardware::MecanumSystem,
    hardware_interface::SystemInterface)
