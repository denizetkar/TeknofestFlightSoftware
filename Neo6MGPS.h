#ifndef NEO6MGPS_H
#define NEO6MGPS_H

#include <Arduino.h>
#ifndef STM32_CORE_VERSION
#include <NeoSWSerial.h>
#endif
#include <TinyGPS++.h>

#ifndef STM32_CORE_VERSION
void lat_lon_to_x_y_mm(double, double, int64_t&, int64_t&);
#else
void lat_lon_to_x_y_m(double, double, double&, double&);
#endif

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
#ifndef STM32_CORE_VERSION
    bool try_read_gps(int64_t&, int64_t&, int64_t&, uint8_t num_gps=4);
#else
    bool try_read_gps(double&, double&, double&, uint8_t num_gps=4);
#endif
};

#endif
