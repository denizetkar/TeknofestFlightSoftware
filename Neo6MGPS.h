#ifndef NEO6MGPS_H
#define NEO6MGPS_H

#include <Arduino.h>
#include <NeoSWSerial.h>
#include <TinyGPS++.h>

void lat_lon_to_x_y_mm(float, float, int64_t&, int64_t&);

class Neo6MGPS {
  public:
    NeoSWSerial ss;
    TinyGPSPlus gps;

    Neo6MGPS(int, int);

    void begin(uint16_t);
    bool try_read_gps(int64_t&, int64_t&, int64_t&);
};

#endif
