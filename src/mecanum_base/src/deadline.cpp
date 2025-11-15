#include <optional>
#include <string>
#include <mutex>
#include <chrono>
#include <unistd.h>   // ::read
#include <rclcpp/rclcpp.hpp>

class MecanumSystem {
public:
  // Legge un messaggio completo delimitato da ^ e $; ritorna nullopt su timeout/errore.
  std::optional<std::string> read_message_();

private:
  int serial_fd_{-1};        // File descriptor della seriale (aperto altrove; -1 indica non disponibile)
  std::mutex serial_mutex_;  // Mutex per garantire accesso thread-safe al fd
};

std::optional<std::string> MecanumSystem::read_message_()
{
  // Protegge tutta la funzione da accessi concorrenti
  std::lock_guard<std::mutex> lock(serial_mutex_);

  // Se la porta non è aperta, non possiamo leggere
  if (serial_fd_ < 0) {
    return std::nullopt;
  }

  std::string buffer;        // Accumula i byte del messaggio tra ^ e $
  char c;                    // Byte corrente letto dalla seriale
  bool inside_message = false; // Stato: true quando siamo dentro un messaggio iniziato da ^

  // Timeout per evitare attese infinite quando i dati non arrivano
  constexpr int timeout_ms = 200;
  auto last_byte_time = std::chrono::steady_clock::now(); // Timestamp dell’ultimo byte ricevuto

  // Loop di lettura: legge byte finché non trova il delimitatore di fine ($) o scatta il timeout
  while (true) {
    // Legge un singolo byte dal file descriptor della seriale
    ssize_t n = ::read(serial_fd_, &c, 1);

    if (n > 0) {
      // Aggiorna il momento dell’ultimo byte ricevuto
      last_byte_time = std::chrono::steady_clock::now();

      // Rileva l’inizio del messaggio: quando arriva '^' resetta il buffer e entra in stato di "inside_message"
      if (c == '^') {
        buffer.clear();
        inside_message = true;
        continue; // Passa al prossimo byte senza aggiungere '^' al contenuto
      }

      // Rileva la fine del messaggio: quando arriva '$' e siamo dentro un messaggio, ritorna il buffer completo
      if (c == '$' && inside_message) {
        return buffer;
      }

      // Se siamo dentro al messaggio, accumula il byte nel buffer (contenuto utile)
      if (inside_message) {
        buffer.push_back(c);
      }

      // Se non siamo dentro al messaggio, i byte che arrivano prima di '^' vengono ignorati (rumore/prefisso)
    } else {
      // Nessun byte disponibile al momento (read non-blocking o empty). Controlla il timeout.
      auto now = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_byte_time).count();

      // Se è passato troppo tempo dall’ultimo byte, interrompi
      if (elapsed > timeout_ms) {
        // Se eravamo dentro un messaggio, segnala che è incompleto
        if (inside_message) {
          RCLCPP_WARN(rclcpp::get_logger("UART"), "Timeout: messaggio incompleto scartato");
        }
        // In ogni caso, ritorna nullopt per indicare che non abbiamo un messaggio valido
        return std::nullopt;
      }

      // Se non è scattato il timeout, continua a looping (attesa di nuovi byte)
    }
  }
}