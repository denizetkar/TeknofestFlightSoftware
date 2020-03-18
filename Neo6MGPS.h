#ifndef NEO6MGPS_H
#define NEO6MGPS_H

#include <Arduino.h>
#ifndef STM32_CORE_VERSION
#include <NeoSWSerial.h>
#endif
#include <TinyGPS++.h>

void lat_lon_to_x_y_mm(float, float, int64_t&, int64_t&);

class Neo6MGPS {
  public:
#ifndef STM32_CORE_VERSION
    NeoSWSerial ss;
#else
    HardwareSerial ss;
#endif
    TinyGPSPlus gps;

    Neo6MGPS(int, int);

    void begin(uint16_t);
    bool try_read_gps(int64_t&, int64_t&, int64_t&, uint8_t num_gps=4);
};

#endif
