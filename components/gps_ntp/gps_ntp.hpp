#ifndef NTP_H
#define NTP_H

#include "esphome/core/component.h"
#include "esphome/components/gps/gps.h"

namespace esphome {
namespace gps {

class GPS_NTP_Server : public Component, GPSListener {
public:
  void loop() override;
  void on_update(TinyGPSPlus &tiny_gps) override {
    if (!this->has_time_)
      this->from_tiny_gps_(tiny_gps);
  }

protected:
  void from_tiny_gps_(TinyGPSPlus &tiny_gps);
  bool has_time_{false};
};

} // namespace gps
} // namespace esphome

#endif