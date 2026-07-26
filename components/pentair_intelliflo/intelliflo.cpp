#include "intelliflo.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <cmath>

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
  while (this->available() > 0) {
    uint8_t c;
    this->read_byte(&c);
    this->handle_received_byte(c);
  }

  // Send next command in queue if at least 150ms passed since last transmission
  if (millis() - this->last_tx_millis_ > 150 && !this->tx_buffer.empty()) {
    auto packet = this->tx_buffer.front();
    this->send_array_cmd(packet.data(), packet.size());
    this->tx_buffer.pop();
    this->last_tx_millis_ = millis();
  }
}

void Intelliflo::handle_received_byte(uint8_t c) {
  this->rx_buffer.push_back(c);
  if (!this->validate_received_message()) {
    this->rx_buffer.clear();
  }
}

bool Intelliflo::validate_received_message() {
  uint32_t at = this->rx_buffer.size() - 1;
  uint8_t *data = &this->rx_buffer[0];

  if (at == 0) return data[0] == 0xFF;
  if (at == 1) return data[1] == 0x00;
  if (at == 2) return data[2] == 0xFF;
  if (at == 3) return data[3] == 0xA5;

  if (at <= 8) return true;
  uint8_t packet_size = data[8];
  uint8_t length = (packet_size + 10);

  if (at < length) return true;

  // Complete packet arrived - parse it
  this->parse_packet(this->rx_buffer);
  return false; // reset buffer after complete packet
}

void Intelliflo::parse_packet(const std::vector<uint8_t> &data) {
  if (data.size() < 10) return;

  // Check for Remote/Local ack packet
  if (data[3] == 0xA5 && data[4] == 0x00 && data[5] == 0x10 && data[6] == 0x60) {
    if (data[7] == 0x04 && data.size() >= 10) {
      if (data[9] == 0xFF) {
        ESP_LOGI(TAG, "Pump is in REMOTE control mode");
      } else if (data[9] == 0x00) {
        ESP_LOGI(TAG, "Pump is in LOCAL control mode");
      }
    }
  } 
  // Check for Status Response packet (data[3]=0xA5, data[4]=0x60, data[5]=0x10, data[6]=0x07)
  else if (data[3] == 0xA5 && data[4] == 0x60 && data[5] == 0x10 && data[6] == 0x07 && data.size() >= 16) {
    ESP_LOGI(TAG, "Received Pump Status Response");

    if (this->running_ != nullptr) {
      if (data[8] == RUNNING || data[8] == 0x0A) {
        this->running_->publish_state(true);
      } else {
        this->running_->publish_state(false);
      }
    }

    if (this->program_ != nullptr) {
      switch (data[9]) {
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
      this->power_->publish_state((data[10] * 256) + data[11]);
    if (this->rpm_ != nullptr)
      this->rpm_->publish_state((data[12] * 256) + data[13]);
    if (this->flow_ != nullptr)
      this->flow_->publish_state(data[14] * 0.227);
    if (this->pressure_ != nullptr)
      this->pressure_->publish_state(data[15] / 14.504);
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
  pumpPowerPacket[9] = prog * 8;
  QueuePacket(pumpPowerPacket, 10);
}

void Intelliflo::saveValueForProgram(int prog, int value) {
  ESP_LOGI(TAG, "saveValueForProgram %d: %d", prog, value);
  uint8_t pumpPowerPacket[] = {0xA5, 0x00, 0x60, 0x10, 0x01, 0x04, 0x03, 0, 0, 0};
  pumpPowerPacket[7] = 0x26 + prog;
  pumpPowerPacket[8] = std::floor(value / 256.0);
  pumpPowerPacket[9] = value % 256;
  QueuePacket(pumpPowerPacket, 10);
}

void Intelliflo::commandRPM(int rpm) {
  ESP_LOGI(TAG, "Command RPM: %d rpm", rpm);
  uint8_t pumpPowerPacket[] = {0xA5, 0x00, 0x60, 0x10, 0x0A, 0x04, 0x02, 0xC4, 0, 0};
  pumpPowerPacket[8] = std::floor(rpm / 256.0);
  pumpPowerPacket[9] = rpm % 256;
  QueuePacket(pumpPowerPacket, 10);
}

void Intelliflo::commandFlow(int flow) {
  ESP_LOGI(TAG, "Command Flow: %.1f m3/h", ((double) flow) / 10);
  uint8_t pumpPowerPacket[] = {0xA5, 0x00, 0x60, 0x10, 0x09, 0x04, 0x02, 0xC4, 0x00, 0};
  pumpPowerPacket[9] = flow;
  QueuePacket(pumpPowerPacket, 10);
}

void Intelliflo::QueuePacket(uint8_t message[], int messageLength) {
  int checksum = 0;
  for (int j = 0; j < messageLength; j++) {
    checksum += message[j];
  }

  std::vector<uint8_t> packet = {0xFF, 0x00, 0xFF};
  packet.insert(packet.end(), message, message + messageLength);
  packet.push_back(checksum >> 8);
  packet.push_back(checksum & 0xFF);

  tx_buffer.push(packet);
}

}  // namespace intelliflo
}  // namespace esphome
