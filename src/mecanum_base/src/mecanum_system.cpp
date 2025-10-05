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

  std::optional<std::string> MecanumSystem::read_line_()
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);

    if (serial_fd_ < 0)
    {
      return std::nullopt;
    }

    std::string line;
    char c;
    while (true)
    {
      ssize_t n = ::read(serial_fd_, &c, 1);
      if (n > 0)
      {
        if (c == '\n')
        {
          break;
        }
        line.push_back(c);
      }
      else
      {
        // nessun dato disponibile
        break;
      }
    }

    if (line.empty())
    {
      return std::nullopt;
    }
    return line;
  }

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

    // Molti firmware si aspettano terminazione a nuova linea.
    // Se il tuo protocollo usa diverso terminatore, modifica qui.
    std::string payload = cmd;
    if (payload.empty() || payload.back() != '\n')
    {
      payload.push_back('\n');
    }

    const char *buf = payload.c_str();
    size_t to_write = payload.size();
    ssize_t written_total = 0;

    // Scrittura robusta: gestisce EAGAIN/EINTR e short write
    while (to_write > 0)
    {
      ssize_t n = ::write(serial_fd_, buf + written_total, to_write);
      if (n > 0)
      {
        written_total += n;
        to_write -= static_cast<size_t>(n);
      }
      else
      {
        if (n < 0 && (errno == EINTR))
        {
          // Interrotto da segnale: ritenta
          continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
        {
          // Non-blocking: nessun buffer disponibile ora, piccola attesa e retry
          // Nota: evita usleep lunghi nell’hot path; regola se serve.
          ::usleep(1000); // 1 ms
          continue;
        }
        RCLCPP_ERROR(this->get_logger(),
                     "send_command_: errore write(): %s", std::strerror(errno));
        return false;
      }
    }

    // Assicura lo svuotamento del buffer di trasmissione prima di proseguire
    if (::tcdrain(serial_fd_) != 0)
    {
      RCLCPP_WARN(this->get_logger(),
                  "send_command_: tcdrain ha riportato errore: %s",
                  std::strerror(errno));
      // Non consideriamo questo un errore fatale per default
    }

    //RCLCPP_DEGUG(rclcpp::get_logger("MecanumSystem"),
    //            "send_command_: scritto '%s' (%zd byte)",
    //            cmd.c_str(), static_cast<ssize_t>(written_total));
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

  // 🔧 Helper per leggere parametri hardware dal file URDF/Xacro
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

  // 🔄 Lettura dei parametri hardware
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
    RCLCPP_ERROR(this->get_logger(), "Errore parsing parametri hardware");
    return hardware_interface::CallbackReturn::ERROR;
  }

  ticks_per_wheel_rev_ = static_cast<double>(ticks_per_rev_) * gear_ratio_;

  // 🔩 Inizializza giunti ruota e servo
  joint_names_.clear();
  joints_.clear();
  servo_names_.clear();
  servos_.clear();

  for (const auto &j : info_.joints)
  {
    bool has_pos = false, has_vel = false;
    bool has_cmd_vel = false, has_cmd_pos = false;

    for (const auto &si : j.state_interfaces)
    {
      if (si.name == hardware_interface::HW_IF_POSITION)
        has_pos = true;
      if (si.name == hardware_interface::HW_IF_VELOCITY)
        has_vel = true;
    }

    for (const auto &ci : j.command_interfaces)
    {
      if (ci.name == hardware_interface::HW_IF_VELOCITY)
        has_cmd_vel = true;
      if (ci.name == hardware_interface::HW_IF_POSITION)
        has_cmd_pos = true;
    }

    // 🚗 Giunti ruota: position + velocity + velocity command
    if (has_pos && has_vel && has_cmd_vel)
    {
      joint_names_.push_back(j.name);
      joints_.emplace_back();
    }
    // 🎯 Giunti servo: position + position command
    else if (has_pos && has_cmd_pos)
    {
      servo_names_.push_back(j.name);
      servos_.emplace_back();
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Giunto %s: interfacce non valide", j.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // 📡 Inizializza sensori sonar (definiti come <sensor> nel Xacro)
  sonar_names_.clear();
  sonars_.clear();

  for (const auto &s : info_.sensors)
  {
    if (s.state_interfaces.size() == 1 && s.state_interfaces[0].name == "range")
    {
      sonar_names_.push_back(s.name);
      sonars_.emplace_back();
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Sensore %s: interfaccia 'range' mancante", s.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(this->get_logger(),
              "Init OK (mock=%s, serial=%s, baud=%d) — Giunti ruota: %zu, servo: %zu, sonar: %zu",
              mock_ ? "true" : "false",
              serial_port_.c_str(), baudrate_,
              joints_.size(), servos_.size(), sonars_.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

  // ================== EXPORT INTERFACES ==================

  std::vector<hardware_interface::StateInterface> MecanumSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;

  // 🔩 Stato delle ruote: posizione e velocità
  out.reserve(joints_.size() * 2);
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    out.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &joints_[i].pos_rad);
    out.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &joints_[i].vel_rad_s);
  }

  // 🎯 Stato dei servomotori: posizione attuale
  for (size_t i = 0; i < servos_.size(); ++i)
  {
    out.emplace_back(servo_names_[i], hardware_interface::HW_IF_POSITION, &servos_[i].position);
  }

  // 📡 Stato dei sonar: distanza misurata (range)
  for (size_t i = 0; i < sonars_.size(); ++i)
  {
    out.emplace_back(sonar_names_[i], "range", &sonars_[i].range_m);
  }

  // 📦 Stato IMU: orientamento (quaternion), velocità angolare, accelerazione lineare
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

  return out;
}
  std::vector<hardware_interface::CommandInterface> MecanumSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;

  // 🚗 Comandi alle ruote: velocità
  out.reserve(joints_.size() + servos_.size());
  for (size_t i = 0; i < joints_.size(); ++i)
  {
    out.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &joints_[i].cmd_vel);
  }

  // 🎯 Comandi ai servomotori: posizione desiderata
  for (size_t i = 0; i < servos_.size(); ++i)
  {
    out.emplace_back(servo_names_[i], hardware_interface::HW_IF_POSITION, &servos_[i].command);
  }

  return out;
}

  // ================== LIFECYCLE ==================

  hardware_interface::CallbackReturn MecanumSystem::on_activate(const rclcpp_lifecycle::State &)
  {
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
        RCLCPP_ERROR(this->get_logger(),
                     "Impossibile aprire la porta seriale %s", serial_port_.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn MecanumSystem::on_deactivate(const rclcpp_lifecycle::State &)
  {
    if (!mock_)
      close_serial();
    return hardware_interface::CallbackReturn::SUCCESS;
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

    // 3) Legge UNA riga completa dalla seriale.
    //    Il formato atteso è CSV con prefisso e terminatore '\n', es.:
    //    - "ENC,dl_fl,dl_fr,dl_rl,dl_rr\n"
    //    - "IMU,qx,qy,qz,qw,gx,gy,gz,ax,ay,az\n"
    auto line = read_line_();

    // 3.1) Se non arriva nulla in questo ciclo, semplicemente ritorna OK.
    //      (Può capitare per jitter o rate diversi tra MCU e PC)
    if (!line || line->empty())
    {
      return hardware_interface::return_type::OK;
    }

    // 3.2) Log della riga grezza ricevuta (utile per diagnosi di framing/formato).
    RCLCPP_DEBUG(this->get_logger(),
                 "Linea seriale ricevuta: %s", line->c_str());

    // 4) Dispatch in base al prefisso del pacchetto.
    //    Usiamo rfind(...,0) per verificare che la stringa inizi con il prefisso.
    if (line->rfind("ENC", 0) == 0)
    {
      // 4.1) Salva le posizioni attuali per ricavare il delta di questo ciclo.
      double prev_pos[4];
      for (int i = 0; i < 4; ++i)
      {
        prev_pos[i] = joints_[i].pos_rad;
      }

      // 4.2) Parsing robusto del pacchetto ENC.
      //      - Qualsiasi eccezione (token non numerici, campi mancanti) viene catturata.
      //      - In caso di errore, si scarta il pacchetto e si continua senza fermare l’hardware.
      try
      {
        // parse_encoder_packet_:
        // - legge i delta tick (4 valori interi dopo "ENC")
        // - applica inversioni
        // - converte i tick in radianti
        // - incrementa joints_[i].pos_rad (posizione cumulativa)
        parse_encoder_packet_(*line);

        // 4.3) Calcolo della velocità media del ciclo (solo se dt valido).
        if (dt > 0.0)
        {
          for (int i = 0; i < 4; ++i)
          {
            const double dpos = joints_[i].pos_rad - prev_pos[i];
            joints_[i].vel_rad_s = dpos / dt;
          }
        }
        else
        {
          RCLCPP_WARN(this->get_logger(),
                      "Velocità non aggiornata: dt=%.6f", dt);
        }

        // 4.4) Log immediato dei giunti aggiornati (utile in debug).
        for (size_t i = 0; i < joints_.size(); ++i)
        {
          RCLCPP_INFO(this->get_logger(),
                      "Joint %s: pos=%.3f rad, vel=%.3f rad/s",
                      joint_names_[i].c_str(),
                      joints_[i].pos_rad,
                      joints_[i].vel_rad_s);
        }
      }
      catch (const std::exception &e)
      {
        // 4.5) Pacchetto malformato: scartalo e prosegui senza interrompere il ciclo di controllo.
        RCLCPP_WARN(this->get_logger(),
                    "Pacchetto ENC malformato, scartato. Errore: %s | Riga: '%s'",
                    e.what(), line->c_str());
      }
    }
    else if (line->rfind("IMU", 0) == 0)
    {
      // 5) Pacchetto IMU: parsing robusto con gestione errori.
      try
      {
        // parse_imu_packet_ aggiorna imu_state_ (orientazione, velocità angolare, accelerazione).
        parse_imu_packet_(*line);

        // 5.1) Log dei valori IMU aggiornati (utile in debug di integrazione).
        //RCLCPP_INFO(this->get_logger(),
        RCLCPP_INFO(this->get_logger(),
                    "IMU orient=(%.3f, %.3f, %.3f, %.3f) "
                    "ang_vel=(%.3f, %.3f, %.3f) "
                    "lin_acc=(%.3f, %.3f, %.3f)",
                    imu_state_.orientation[0], imu_state_.orientation[1],
                    imu_state_.orientation[2], imu_state_.orientation[3],
                    imu_state_.angular_vel[0], imu_state_.angular_vel[1],
                    imu_state_.angular_vel[2],
                    imu_state_.linear_accel[0], imu_state_.linear_accel[1],
                    imu_state_.linear_accel[2]);
      }
      catch (const std::exception &e)
      {
        // 5.2) Pacchetto IMU malformato: scarta e continua.
        RCLCPP_WARN(this->get_logger(),
                    "Pacchetto IMU malformato, scartato. Errore: %s | Riga: '%s'",
                    e.what(), line->c_str());
      }
    }
    else if (line->rfind("SON", 0) == 0){
      // TODO
    }
    else if (line->rfind("LOG", 0) == 0){
      RCLCPP_INFO(this->get_logger(),
                  "Pico log: %s", line->c_str());
    }
    else
    {
      // 6) Prefisso sconosciuto: il pacchetto non appartiene ai formati attesi.
      //    Lo segnaliamo ma non è un errore critico (potrebbe essere rumore o debug lato MCU).
      RCLCPP_WARN(this->get_logger(),
                  "Pacchetto con prefisso sconosciuto: %s", line->c_str());
    }

    // 7) Concludi regolarmente: anche se non è arrivato nulla o si è scartato un pacchetto,
    //    il ciclo di controllo continua senza disattivare l'hardware.
    return hardware_interface::return_type::OK;
  }

  // ================== WRITE ==================
  //
  // Invia i comandi motori in formato CSV.
  //
  hardware_interface::return_type MecanumSystem::write(
      const rclcpp::Time &, const rclcpp::Duration &)
  {
    // 1) Bypass in mock
    if (mock_)
    {
      return hardware_interface::return_type::OK;
    }

    // 2) Costruzione CSV con 4 decimali (stessa formattazione lato log)
    std::ostringstream ss;
    ss << "CMD," << std::fixed << std::setprecision(4)
       << joints_[0].cmd_vel << ","
       << joints_[1].cmd_vel << ","
       << joints_[2].cmd_vel << ","
       << joints_[3].cmd_vel;
    const std::string csv = ss.str();

    // 3) Invio sempre (la riduzione riguarda solo il logging)
    if (!send_command_(csv + "\n"))
    {
      RCLCPP_ERROR(this->get_logger(),
                   "Errore invio comando seriale");
      return hardware_interface::return_type::ERROR;
    }

    // 4) Log solo se il comando (formattato) è diverso dall’ultimo loggato
    //    - Stato minimo: una variabile statica locale con l’ultima stringa loggata.
    //    - Vantaggio: nessuna modifica all'header, nessun array da mantenere.
    static std::string last_logged_csv; // vuota al primo giro → farà il primo log
    if (last_logged_csv != csv)
    {
      RCLCPP_INFO(this->get_logger(),
                  "write() cmd: FL=%.3f FR=%.3f RL=%.3f RR=%.3f",
                  joints_[0].cmd_vel, joints_[1].cmd_vel,
                  joints_[2].cmd_vel, joints_[3].cmd_vel);
      last_logged_csv = csv; // aggiorna “istantanea” del comando loggato
    }

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
  // Parsing pacchetto encoder: "ENC,dl_fl,dl_fr,dl_rl,dl_rr"
  //
  void MecanumSystem::parse_encoder_packet_(const std::string &line)
  {
    std::stringstream ss(line);
    std::string token;

    // Scarta il prefisso "ENC"
    std::getline(ss, token, ',');

    int deltas[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4 && std::getline(ss, token, ','); ++i)
    {
      deltas[i] = std::stoi(token);
    }

    // Applica inversioni
    deltas[0] *= inv_fl_;
    deltas[1] *= inv_fr_;
    deltas[2] *= inv_rl_;
    deltas[3] *= inv_rr_;

    // Tick → radianti
    const double k = 2.0 * M_PI / ticks_per_wheel_rev_;
    double dth[4];
    for (int i = 0; i < 4; ++i)
      dth[i] = k * deltas[i];

    // Aggiorna posizione e velocità (vel calcolata nel read con dt)
    for (int i = 0; i < 4; ++i)
      joints_[i].pos_rad += dth[i];
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
