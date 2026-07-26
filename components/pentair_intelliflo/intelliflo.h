#pragma once

#include "esphome/core/component.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"
#include <queue>
#include <vector>

namespace esphome {
namespace intelliflo {

enum running : uint8_t {
  STOPPED = 0x04,
  RUNNING = 0x0A,
};

enum program : uint8_t {
  NO_PROG = 0x00,
  LOCAL1 = 0x01,
  LOCAL2 = 0x02,
  LOCAL3 = 0x03,
  LOCAL4 = 0x04,
  EXT1 = 0x09,
  EXT2 = 0x0A,
  EXT3 = 0x0B,
  EXT4 = 0x0C,
  TIMEOUT = 0x0E,
  PRIMING = 0x11,
  QUICKCLEAN = 0x0D,
  UNKNOWN = 0xFF,
};

class Intelliflo : public uart::UARTDevice, public PollingComponent {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  void update() override;

  void set_power(sensor::Sensor *sensor) { power_ = sensor; }
  void set_rpm(sensor::Sensor *sensor) { rpm_ = sensor; }
  void set_flow(sensor::Sensor *sensor) { flow_ = sensor; }
  void set_pressure(sensor::Sensor *sensor) { pressure_ = sensor; }
  void set_running(binary_sensor::BinarySensor *sensor) { running_ = sensor; }
  void set_program(text_sensor::TextSensor *sensor) { program_ = sensor; }

  void requestPumpStatus();
  void run();
  void stop();
  void commandLocalProgram(int prog);
  void commandExternalProgram(int prog);
  void saveValueForProgram(int prog, int value);
  void commandRPM(int rpm);
  void commandFlow(int flow);
  void pumpToLocalControl();
  void pumpToRemoteControl();

 protected:
  uint32_t last_tx_millis_{0};
  uint32_t last_received_byte_millis{0};

  void send_array_cmd(const uint8_t *data, size_t len);
  void parse_packet(const std::vector<uint8_t> &data);
  void handle_received_byte(uint8_t c);
  bool validate_received_message();
  void QueuePacket(uint8_t message[], int messageLength);

  std::vector<uint8_t> rx_buffer;
  std::queue<std::vector<uint8_t>> tx_buffer;

  sensor::Sensor *power_{nullptr};
  sensor::Sensor *rpm_{nullptr};
  sensor::Sensor *flow_{nullptr};
  sensor::Sensor *pressure_{nullptr};
  binary_sensor::BinarySensor *running_{nullptr};
  text_sensor::TextSensor *program_{nullptr};
};

}  // namespace intelliflo
}  // namespace esphome
