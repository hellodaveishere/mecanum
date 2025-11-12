std::optional<std::string> MecanumSystem::read_line_()
{
  std::lock_guard<std::mutex> lock(serial_mutex_);

  if (serial_fd_ < 0)
  {
    return std::nullopt;
  }

  std::string buffer;
  std::string chunk;
  char c;
  bool inside_chunk = false;

  constexpr int timeout_ms = 50;
  auto last_byte_time = std::chrono::steady_clock::now();

  while (true)
  {
    ssize_t n = ::read(serial_fd_, &c, 1);
    if (n > 0)
    {
      last_byte_time = std::chrono::steady_clock::now();
      chunk.push_back(c);

      // Inizio chunk
      if (!inside_chunk && chunk.size() >= 7 &&
          chunk.substr(chunk.size() - 7) == "<CHUNK>")
      {
        chunk.clear();
        inside_chunk = true;
        continue;
      }

      // Fine chunk
      if (inside_chunk && chunk.size() >= 10 &&
          chunk.substr(chunk.size() - 10) == "<ENDCHUNK>")
      {
        chunk.erase(chunk.size() - 10);
        buffer += chunk;
        return buffer;  // Messaggio completo
      }
    }
    else
    {
      // Timeout se <ENDCHUNK> non arriva
      auto now = std::chrono::steady_clock::now();
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_byte_time).count();

      if (elapsed > timeout_ms)
      {
        RCLCPP_WARN(rclcpp::get_logger("UART"), "Timeout: messaggio incompleto scartato");
        return std::nullopt;
      }
    }
  }
}