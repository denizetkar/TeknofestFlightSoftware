#include "Neo6MGPS.h"

const unsigned char UBLOX_INIT[] PROGMEM = {
  // Disable specific NMEA sentences
  //0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x24, // GxGGA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x2B, // GxGLL off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x32, // GxGSA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x39, // GxGSV off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40, // GxRMC off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x01,0x05,0x47, // GxVTG off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x08,0x00,0x00,0x00,0x00,0x00,0x01,0x08,0x5C, // GxZDA off

  // Rate (pick one)
  0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, //(10Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A, //(5Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, //(1Hz)
};


// latitude & longitude in degrees into x,y
// coordinates in millimeters on the surface of earth
void lat_lon_to_x_y_mm(float lat, float lon, int64_t &x, int64_t &y)
{
  float lat_rad = radians(lat);
  x = static_cast<int64_t>( 111132952.548 * lat );
  y = static_cast<int64_t>( (((111412.84 * cos(lat_rad) - 93.5 * cos(3.0 * lat_rad)) + 0.118 * cos(5.0 * lat_rad)) * lon) * 1000.0 );
}


Neo6MGPS::Neo6MGPS(int GPS_TX_PIN, int GPS_RX_PIN)
  : ss{ GPS_TX_PIN, GPS_RX_PIN }
{
}

void Neo6MGPS::begin(uint16_t GPS_BAUD_RATE)
{
  ss.begin(GPS_BAUD_RATE);
  // configure the ublox gps module
  for (size_t i = 0; i < sizeof(UBLOX_INIT); i++) {
    ss.write( pgm_read_byte(UBLOX_INIT+i) );
  };
}

// This function attempts to read GPS module stream and search for 4 GPS satellites.
// If not possible it returns false, otherwise it updates lat-lon-alt and returns true.
bool Neo6MGPS::try_read_gps(int64_t& lat_mm, int64_t& lon_mm, int64_t& alt_mm)
{
  while (ss.available() > 0){
    gps.encode(ss.read());
    if (gps.satellites.isUpdated() && gps.satellites.value() >= 4 && gps.location.isUpdated()){
      lat_lon_to_x_y_mm(gps.location.lat(), gps.location.lng(), lat_mm, lon_mm);
      alt_mm = gps.altitude.value() * 10;
      return true;
    }
  }
  return false;
}
