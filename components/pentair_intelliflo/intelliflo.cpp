#include "intelliflo.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace intelliflo {

static const char *const TAG = "intelliflo";

void Intelliflo::setup() {}

void Intelliflo::dump_config() { ESP_LOGCONFIG(TAG, "  Pentair Intelliflo"); }

void Intelliflo::update() {
  // Polling update loop ONLY requests pump status
  this->requestPumpStatus();
}

void Intelliflo::loop() {
  uint32_t now = millis();

  // Clear stale RX buffer if line was quiet for > 100ms mid-packet
  if (!this->rx_buffer.empty() && (now - this->last_received_byte_millis > 100)) {
    ESP_LOGV(TAG, "RX inter-byte timeout, resetting buffer");
    this->rx_buffer.clear();
  }

  while (this->available() > 0) {
    uint8_t c;
    this->read_byte(&c);
    this->last_received_byte_millis = now;
    this->handle_received_byte(c);
  }

  // Send next command in queue if at least 150ms passed since last transmission
  if (now - this->last_tx_millis_ > 150 && !this->tx_buffer.empty()) {
    const auto &packet = this->tx_buffer.front();
    this->send_array_cmd(packet.data(), packet.size());
    this->tx_buffer.pop();
    this->last_tx_millis_ = now;
  }
}

void Intelliflo::handle_received_byte(uint8_t c) {
  this->rx_buffer.push_back(c);
  if (!this->validate_received_message()) {
    this->rx_buffer.clear();
  }
}

bool Intelliflo::validate_received_message() {
  if (this->rx_buffer.empty()) return false;

  uint32_t at = this->rx_buffer.size() - 1;
  uint8_t *data = this->rx_buffer.data();

  // Validate preamble sequence: FF 00 FF A5
  if (at == 0) return data[0] == 0xFF;
  if (at == 1) return data[1] == 0x00;
  if (at == 2) return data[2] == 0xFF;
  if (at == 3) return data[3] == 0xA5;

  if (at <= 8) return true;

  uint8_t packet_size = data[8];
  uint32_t expected_length = packet_size + 11; // 3 preamble + 5 frame + 1 len + payload + 2 checksum

  // Prevent runaway buffer allocation from noise-corrupted length byte
  if (expected_length > 64) {
    ESP_LOGW(TAG, "Corrupted packet length: %u, resetting buffer", expected_length);
    return false;
  }

  if (this->rx_buffer.size() < expected_length) return true;

  // Validate packet checksum
  uint16_t checksum = 0;
  for (size_t j = 3; j < 3 + packet_size + 6; j++) {
    checksum += data[j];
  }

  uint16_t packet_checksum = (data[3 + 6 + packet_size] << 8) | data[3 + 7 + packet_size];
  if (checksum != packet_checksum) {
    ESP_LOGW(TAG, "Checksum mismatch: calc 0x%04X vs packet 0x%04X", checksum, packet_checksum);
    return false; // reset buffer on checksum error
  }

  // Remove preamble FF 00 FF to get clean A5 packet
  std::vector<uint8_t> clean_packet(this->rx_buffer.begin() + 3, this->rx_buffer.end());

  std::string pretty_cmd = format_hex_pretty(clean_packet);
  ESP_LOGI(TAG, "Package received: %s", pretty_cmd.c_str());

  this->parse_packet(clean_packet);

  return false; // reset buffer after processing complete packet
}

void Intelliflo::parse_packet(const std::vector<uint8_t> &data) {
  if (data.size() < 7) return;

  // Check for Remote/Local control ack packet (data[3]==0x60 && data[4]==0x04)
  if (data[3] == 0x60 && data[4] == 0x04 && data.size() >= 7) {
    if (data[6] == 0xFF) {
      ESP_LOGI(TAG, "Pump is in REMOTE control mode");
    } else if (data[6] == 0x00) {
      ESP_LOGI(TAG, "Pump is in LOCAL control mode");
    }
  } 
  // Check for Status Response packet (data[3]==0x60 && data[4]==0x07)
  else if (data[3] == 0x60 && data[4] == 0x07 && data.size() >= 14) {
    ESP_LOGI(TAG, "Parsing Pump Status Response");

    if (this->running_ != nullptr) {
      if (data[6] == RUNNING || data[6] == 0x0A) {
        this->running_->publish_state(true);
      } else {
        this->running_->publish_state(false);
      }
    }

    if (this->program_ != nullptr) {
      switch (data[7]) {
        case NO_PROG: this->program_->publish_state("Stopped"); break;
        case LOCAL1: this->program_->publish_state("Local 1"); break;
        case LOCAL2: this->program_->publish_state("Local 2"); break;
        case LOCAL3: this->program_->publish_state("Local 3"); break;
        case LOCAL4: this->program_->publish_state("Local 4"); break;
        case EXT1: this->program_->publish_state("External 1"); break;
        case EXT2: this->program_->publish_state("External 2"); break;
        case EXT3: this->program_->publish_state("External 3"); break;
        case EXT4: this->program_->publish_state("External 4"); break;
        case TIMEOUT: this->program_->publish_state("Time Out"); break;
        case PRIMING: this->program_->publish_state("Priming"); break;
        case QUICKCLEAN: this->program_->publish_state("Quick Clean"); break;
        default: this->program_->publish_state("Running"); break;
      }
    }

    if (this->power_ != nullptr)
      this->power_->publish_state((data[8] * 256) + data[9]);
    if (this->rpm_ != nullptr)
      this->rpm_->publish_state((data[10] * 256) + data[11]);
    if (this->flow_ != nullptr)
      this->flow_->publish_state(data[12] * 0.227);
    if (this->pressure_ != nullptr)
      this->pressure_->publish_state(data[13] / 14.504);
  }
}

void Intelliflo::send_array_cmd(const uint8_t *data, size_t len) {
  this->flush();
  this->write_array(&data[0], len);
  std::string pretty_cmd = format_hex_pretty((uint8_t *) &data[0], len);
  ESP_LOGI(TAG, "Sent: %s", pretty_cmd.c_str());
}

void Intelliflo::requestPumpStatus() {
  ESP_LOGI(TAG, "Requesting pump status");
  uint8_t statusPacket[] = {0xA5, 0x00, 0x60, 0x10, 0x07, 0x00};
  QueuePacket(statusPacket, 6);
}

void Intelliflo::pumpToLocalControl() {
  ESP_LOGI(TAG, "Requesting local control");
  uint8_t localControlPacket[] = {0xA5, 0x00, 0x60, 0x10, 0x04, 0x01, 0x00};
  QueuePacket(localControlPacket, 7);
}

void Intelliflo::pumpToRemoteControl() {
  ESP_LOGI(TAG, "Requesting remote control");
  uint8_t remoteControlPacket[] = {0xA5, 0x00, 0x60, 0x10, 0x04, 0x01, 0xFF};
  QueuePacket(remoteControlPacket, 7);
}

void Intelliflo::run() {
  ESP_LOGI(TAG, "Run Pump");
  uint8_t pumpPowerPacket[] = {0xA5, 0x00, 0x60, 0x10, 0x06, 0x01, 0x0A};
  QueuePacket(pumpPowerPacket, 7);
}

void Intelliflo::stop() {
  ESP_LOGI(TAG, "Stop Pump");
  uint8_t pumpPowerPacket[] = {0xA5, 0x00, 0x60, 0x10, 0x06, 0x01, 0x04};
  QueuePacket(pumpPowerPacket, 7);
}

void Intelliflo::commandLocalProgram(int prog) {
  ESP_LOGI(TAG, "Command local program %d", prog);
  uint8_t pumpPowerPacket[] = {0xA5, 0x00, 0x60, 0x10, 0x05, 0x01, 0};
  pumpPowerPacket[6] = prog + 1;
  QueuePacket(pumpPowerPacket, 7);
}

void Intelliflo::commandExternalProgram(int prog) {
  ESP_LOGI(TAG, "Command external program %d", prog);
  uint8_t pumpPowerPacket[] = {0xA5, 0x00, 0x60, 0x10, 0x01, 0x04, 0x03, 0x21, 0x00, 0x00};
  pumpPowerPacket[9] = (prog * 8) & 0xFF;
  QueuePacket(pumpPowerPacket, 10);
}

void Intelliflo::saveValueForProgram(int prog, int value) {
  ESP_LOGI(TAG, "saveValueForProgram %d: %d", prog, value);
  uint8_t pumpPowerPacket[] = {0xA5, 0x00, 0x60, 0x10, 0x01, 0x04, 0x03, 0, 0, 0};
  pumpPowerPacket[7] = 0x26 + prog;
  pumpPowerPacket[8] = (value >> 8) & 0xFF;
  pumpPowerPacket[9] = value & 0xFF;
  QueuePacket(pumpPowerPacket, 10);
}

void Intelliflo::commandRPM(int rpm) {
  ESP_LOGI(TAG, "Command RPM: %d rpm", rpm);
  uint8_t pumpPowerPacket[] = {0xA5, 0x00, 0x60, 0x10, 0x0A, 0x04, 0x02, 0xC4, 0, 0};
  pumpPowerPacket[8] = (rpm >> 8) & 0xFF;
  pumpPowerPacket[9] = rpm & 0xFF;
  QueuePacket(pumpPowerPacket, 10);
}

void Intelliflo::commandFlow(int flow) {
  ESP_LOGI(TAG, "Command Flow: %.1f m3/h", ((double) flow) / 10.0);
  uint8_t pumpPowerPacket[] = {0xA5, 0x00, 0x60, 0x10, 0x09, 0x04, 0x02, 0xC4, 0, 0};
  pumpPowerPacket[8] = (flow >> 8) & 0xFF;
  pumpPowerPacket[9] = flow & 0xFF;
  QueuePacket(pumpPowerPacket, 10);
}

void Intelliflo::QueuePacket(uint8_t message[], int messageLength) {
  if (this->tx_buffer.size() >= 15) {
    ESP_LOGW(TAG, "TX Queue full (15 items), dropping command to prevent heap bloat");
    return;
  }

  int checksum = 0;
  for (int j = 0; j < messageLength; j++) {
    checksum += message[j];
  }

  std::vector<uint8_t> packet = {0xFF, 0x00, 0xFF};
  packet.insert(packet.end(), message, message + messageLength);
  packet.push_back((checksum >> 8) & 0xFF);
  packet.push_back(checksum & 0xFF);

  this->tx_buffer.push(std::move(packet));
}

}  // namespace intelliflo
}  // namespace esphome
