#include <MPU9250.h>
#include "MadgwickAHRS.h"
#include "QuaternionPID.h"

#include <TinyGPS++.h>
#include <NeoSWSerial.h>

#include "EarthPositionFilter.h"

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;

// a PID controller object
QuaternionPID controller{ 1.0, 0.5 };

#define GPS_RX_PIN 3
#define GPS_TX_PIN 4
#define GPS_BAUD_RATE 9600

// The TinyGPS++ object
TinyGPSPlus gps;
// The serial connection to the GPS device
NeoSWSerial ss(GPS_TX_PIN, GPS_RX_PIN);

// Kalman Filter object for Latitude, Longitude, Altitude
EarthPositionFilter lat_filter, lon_filter, alt_filter;

const unsigned char UBLOX_INIT[] PROGMEM = {
  // Rate (pick one)
  0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, //(10Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xC8,0x00,0x01,0x00,0x01,0x00,0xDE,0x6A, //(5Hz)
  //0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39, //(1Hz)

  // Disable specific NMEA sentences
  //0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x00,0x24, // GxGGA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x01,0x00,0x00,0x00,0x00,0x00,0x01,0x01,0x2B, // GxGLL off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x02,0x32, // GxGSA off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x03,0x00,0x00,0x00,0x00,0x00,0x01,0x03,0x39, // GxGSV off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x04,0x00,0x00,0x00,0x00,0x00,0x01,0x04,0x40, // GxRMC off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x05,0x00,0x00,0x00,0x00,0x00,0x01,0x05,0x47, // GxVTG off
  0xB5,0x62,0x06,0x01,0x08,0x00,0xF0,0x08,0x00,0x00,0x00,0x00,0x00,0x01,0x08,0x5C, // GxZDA off
};

// There is approximately 5.5 degrees East magnetic declination in Turkey on 24.02.2020.
// ( cos(-5.5*pi/180), 0, 0, sin(-5.5*pi/180) ) is the rotation quaternion required to
// rotate the true north frame into magnetic north frame
const float q_magnetic_declination[4] = {0.9953961983671789, 0, 0, -0.09584575252022398};


float roll, pitch, yaw, ax, ay, az, ux, uy, uz, q_a_tn[4];
int32_t lat_cm, lon_cm;
uint32_t before = 0, deltat;
void setup() {
  // serial to display data
  Serial.begin(2000000);
  while(!Serial);
  ss.begin(GPS_BAUD_RATE);

  // configure the ublox gps module
  for (size_t i = 0; i < sizeof(UBLOX_INIT); i++) {                        
    ss.write( pgm_read_byte(UBLOX_INIT+i) );
  };

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1);
  }
  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-1000 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_1000DPS);
  // setting DLPF bandwidth to 184 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_184HZ);
  // setting SRD to 1 for a 500 Hz update rate
  IMU.setSrd(1);

  // ENTER CALIBRATION VALUES
  // accel bias
  IMU.setAccelCalX(0.00900733333333333);
  IMU.setAccelCalY(0.000612666666666678);
  IMU.setAccelCalZ(-0.0303496666666667);
  IMU.setAccelTM(
    {{0.997971236148955,-0.00430622003848702,0.00118009636690604},
     {0.00470700811404384,0.998364615545652,-0.00638311318231112},
     {0.0012955016372647,0.00387494101216781,0.987478872856534}});

  // mag bias
  IMU.setMagCalX(-31.775899);
  IMU.setMagCalY(13.935825);
  IMU.setMagCalZ(16.369178);
  IMU.setMagTM(
    {{0.027530,0.000372,-0.003384},
     {0.000372,0.027866,0.000568},
     {-0.003384,0.000568,0.027386}});

  // calibrate the estimated orientation
  Serial.println("Calibrating orientation estimate, wait please!");
  uint32_t calib_start = micros();
  madgwick_beta = 4.0;
  while (true) {
    if(IMU.isDataReady()) {
      deltat = micros() - before;
      // read the sensor
      IMU.readSensor();
  
      // Update rotation of the sensor frame with respect to the NWU frame
      // where N is magnetic north, W is west and U is up.
      MadgwickAHRSupdate(IMU.getGyroX_rads(), IMU.getGyroY_rads(), IMU.getGyroZ_rads(),
                         IMU.getAccelX_g(), IMU.getAccelY_g(), IMU.getAccelZ_g(),
                         IMU.getMagX_uT(), IMU.getMagY_uT(), IMU.getMagZ_uT(), deltat/1000000.0);
      before += deltat;

      // calibration is done after about 3 seconds
      if (micros() - calib_start > 3000000) {
        Serial.println("Calibration done.");
        break;
      }
    }
  }
  madgwick_beta = MadgwickBetaDef;
}

//-------------------------------------------------------------------------------------------------

void loop() {
  float deltat_sec;
  if(IMU.isDataReady()) {
    deltat = micros() - before;
    deltat_sec = deltat / 1000000.0;
    // read the sensor
    IMU.readSensor();

    // Update rotation of the sensor frame with respect to the NWU frame
    // where N is magnetic north, W is west and U is up.
    MadgwickAHRSupdate(IMU.getGyroX_rads(), IMU.getGyroY_rads(), IMU.getGyroZ_rads(),
                       0, 0, 0,
                       IMU.getMagX_uT(), IMU.getMagY_uT(), IMU.getMagZ_uT(), deltat_sec);

    // make magnetic declination corrections to q_a
    quaternion_prod(q_magnetic_declination, q_a, q_a_tn);

    // find acceleration vector in local NWU reference frame
    rotate_vector_by_quaternion(q_a_tn, IMU.getAccelX_g(), IMU.getAccelY_g(), IMU.getAccelZ_g(), ax, ay, az);
    lat_filter.set_deltat(deltat_sec); lat_filter.predict(ax);
    lon_filter.set_deltat(deltat_sec); lon_filter.predict(-ay);
    alt_filter.set_deltat(deltat_sec); alt_filter.predict(az - 1.0);

    // find corrective actions ux, uy, uz
    controller.compute(q_a, IMU.getGyroX_rads(), IMU.getGyroY_rads(), IMU.getGyroZ_rads(), ux, uy, uz);

    // calculate roll, pitch, yaw
    roll  = atan2(2.0 * (q_a_tn[0] * q_a_tn[1] + q_a_tn[2] * q_a_tn[3]), 1.0 - 2.0 * (q_a_tn[1] * q_a_tn[1] + q_a_tn[2] * q_a_tn[2]));
    pitch = asin(2.0 * (q_a_tn[0] * q_a_tn[2] - q_a_tn[1] * q_a_tn[3]));
    yaw   = atan2(2.0 * (q_a_tn[0] * q_a_tn[3] + q_a_tn[1] * q_a_tn[2]), 1.0 - 2.0 * (q_a_tn[2] * q_a_tn[2] + q_a_tn[3] * q_a_tn[3]));
    roll *= (180.0 / PI);
    pitch *= (180.0 / PI);
    yaw   *= (180.0 / PI);

    // display the data
    Serial.print("dt:\t");
    Serial.print(deltat);
    Serial.print("\tRoll:\t");
    Serial.print(roll, 4);
    Serial.print("\tPitch:\t");
    Serial.print(pitch, 4);
    Serial.print("\tYaw:\t");
    Serial.print(yaw, 4);
    Serial.print("\tV_x:\t");
    Serial.print(lat_filter.get_vel_cm_per_sec(), 4);
    Serial.print("\tV_y:\t");
    Serial.print(lon_filter.get_vel_cm_per_sec(), 4);
    Serial.print("\tV_z:\t");
    Serial.println(alt_filter.get_vel_cm_per_sec(), 4);
    Serial.flush();
    before += deltat;
  }

  // get gps data if available and 'update' the position and velocity 'prediction's
  while (ss.available() > 0){
    gps.encode(ss.read());
    if (gps.satellites.isUpdated() && gps.satellites.value() >= 4 && gps.location.isUpdated()){
      lat_lon_to_x_y_cm(gps.location.lat(), gps.location.lng(), lat_cm, lon_cm);
      lat_filter.update(lat_cm);
      lon_filter.update(lon_cm);
      alt_filter.update(gps.altitude.value());
    }
  }
}
