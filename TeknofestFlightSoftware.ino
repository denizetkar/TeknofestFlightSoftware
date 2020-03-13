//------------------------------------------------------------------------------------------------
//---------------------includes-------------------------------------------------------------------

#include <MPU9250.h>
#include "MadgwickAHRS.h"
#include "QuaternionPID.h"

// TODO: After porting the code to STM32: Read positions from GPS in meters!
#include "Neo6MGPS.h"

// TODO: After porting the code to STM32: Turn int64_t variables into 'double' and track position in meters!
#include "EarthPositionFilter.h"
#include "FinController.h"

// TODO: After porting the code to STM32: No longer use "print_64bit.h"
#include "print_64bit.h"
#include "BitQueue.h"

//------------------------------------------------------------------------------------------------
//---------------------definitions----------------------------------------------------------------

#define IMU_CALIB_MAX_COUNT 500
#define MIN_NUM_GPS 5

// TODO: redefine the value below as at least (4g)^2=16 !
#define TAKEOFF_ACCELERATION_SQ 1.0

#define GPS_RX_PIN 3
#define GPS_TX_PIN 4
#define GPS_BAUD_RATE 9600

// TODO: Decide which motor type to use for fin correction
#ifndef FIN_CONTROL_BY_SERVO
// 8, 9, 10, 11 --> 8, 10, 9, 11 (ATTENTION!)
#define CONTROLLER0_PINS { 8, 10, 9, 11 }
#define CONTROLLER1_PINS { -1, -1, -1, -1 }
#define CONTROLLER2_PINS { -1, -1, -1, -1 }
#define CONTROLLER3_PINS { -1, -1, -1, -1 }
#else
#define CONTROLLER0_PINS 9
#define CONTROLLER1_PINS 10
#define CONTROLLER2_PINS 11
#define CONTROLLER3_PINS 12
#endif

#define VZ_NEG_Q_BIT_SIZE 192
// TODO: After porting the code to STM32: Change this altitude into meters!
#define MAIN_RECOVERY_ALTITUDE 600000

//------------------------------------------------------------------------------------------------
//---------------------setup and loop objects-----------------------------------------------------

// An MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(I2c, 0x68);
int status;

// A PID controller object
QuaternionPID controller{ 50.0, 0.5, 1.0 };

// The Neo-6M GPS object
Neo6MGPS neo6m(GPS_TX_PIN, GPS_RX_PIN);

// Kalman Filter object for Latitude, Longitude, Altitude
EarthPositionFilter lat_filter, lon_filter, alt_filter;

// Servo object for controlling fins
FinController fin_controller(CONTROLLER0_PINS, CONTROLLER1_PINS, CONTROLLER2_PINS, CONTROLLER3_PINS);

// Bit Queue for storing negativity condition of the latest measured Vz values
BitQueue<VZ_NEG_Q_BIT_SIZE> vz_neg_q;

//------------------------------------------------------------------------------------------------
//---------------------setup and loop constants---------------------------------------------------

// There is approximately 5.5 degrees East magnetic declination in Turkey on 24.02.2020.
// ( cos(-5.5*pi/180), 0, 0, sin(-5.5*pi/180) ) is the rotation quaternion required to
// rotate the true north frame into magnetic north frame
const float q_magnetic_declination[4] = {0.9953961983671789, 0, 0, -0.09584575252022398};

//------------------------------------------------------------------------------------------------
//---------------------setup and loop variables---------------------------------------------------

// Enumeration of flight states
enum FlightState: uint8_t {
  _BEFORE_FLIGHT = 0,
  _FLYING = 1,
  _FALLING_FAST = 2,
  _FALLING_SLOW = 3,
  _MAIN_COMP_SAFE_FAIL = 4
};
FlightState FLIGHT_STATE = FlightState::_BEFORE_FLIGHT;
float roll, pitch, yaw, X, Y, Z, ux, uy, uz, q_a_tn[4], deltat_sec;
// TODO: After porting the code to STM32: Change below positions to 'double' and into meters!
int64_t lat_mm, lon_mm, alt_mm, ground_alt_mm;
uint32_t before = 0, deltat;
uint8_t vz_neg_count = 0;

//------------------------------------------------------------------------------------------------
//---------------------setup function-------------------------------------------------------------

void setup() {
  // Serial to display data
  Serial.begin(2000000);
  while (!Serial);

  // Give initial values of 0 degrees
  Serial.println(F("Initializing the fin controls..."));
  fin_controller.makeFinCorrections(0, 0, 0);

  Serial.println(F("Initializing GPS module..."));
  neo6m.begin(GPS_BAUD_RATE);

  // Start communication with IMU
  Serial.println(F("Initializing IMU..."));
  status = IMU.begin();
  if (status < 0) {
    Serial.print(F("IMU initialization unsuccessful: "));
    Serial.println(status);
    while(1);
  }

  // Setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // Setting the gyroscope full scale range to +/-1000 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_1000DPS);
  // Setting DLPF bandwidth to 184 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_184HZ);
  // Setting SRD to 1 for a 500 Hz update rate
  IMU.setSrd(1);

  // ENTER CALIBRATION VALUES
  // ACCEL bias
  IMU.setAccelCalX(0.00900733333333333);
  IMU.setAccelCalY(0.000612666666666678);
  IMU.setAccelCalZ(-0.0303496666666667);
  IMU.setAccelTM(
    {{0.997971236148955,-0.00430622003848702,0.00118009636690604},
     {0.00470700811404384,0.998364615545652,-0.00638311318231112},
     {0.0012955016372647,0.00387494101216781,0.987478872856534}});
  // MAG bias
  IMU.setMagCalX(-44.988848);
  IMU.setMagCalY(16.593980);
  IMU.setMagCalZ(12.325044);
  IMU.setMagTM(
    {{0.021754,0.000565,-0.002232},
     {0.000565,0.021576,0.000495},
     {-0.002232,0.000495,0.021625}});

  // Calibrate the estimated orientation
  Serial.println(F("Calibrating orientation estimate..."));
  uint16_t imu_data_count = 0;
  while (true) {
    // Try to read the sensor
    if(IMU.tryReadSensor()) {

      // Update rotation of the sensor frame with respect to the NWU frame
      // where N is magnetic north, W is west and U is up.
      MadgwickAHRSupdate(0, 0, 0,
                         IMU.getAccelX_g(), IMU.getAccelY_g(), IMU.getAccelZ_g(),
                         IMU.getMagX_uT(), IMU.getMagY_uT(), IMU.getMagZ_uT(),
                         0.01 + 0.09 * cos(imu_data_count * PI / (2 * IMU_CALIB_MAX_COUNT)));
      imu_data_count++;
      // Calibration is done after about 3 seconds
      if (imu_data_count >= IMU_CALIB_MAX_COUNT) {
        break;
      }
    }
  }

  // Make sure our GPS module uses enough GPS satellites and initialize current position.
  Serial.println(F("Searching for GPS satellites..."));
  while (true) {
    if (neo6m.try_read_gps(lat_mm, lon_mm, alt_mm, MIN_NUM_GPS)) {
      // FOUND at least 4 GPS satellites!
      lat_filter.set_pos_mm(lat_mm);
      lon_filter.set_pos_mm(lon_mm);
      alt_filter.set_pos_mm(alt_mm);
      ground_alt_mm = alt_mm;
      break;
    }
  }

  // TODO: Send _BEFORE_FLIGHT signal to the backup computer, and wait for transmission!
  // TODO: Let the ground station know that flight computer is READY.

  // Wait for high acceleration
  Serial.println(F("Waiting for liftoff before loop..."));
  while (true) {
    // Try to read the sensor
    if(IMU.tryReadSensor()) {
      X = IMU.getAccelX_g(); Y = IMU.getAccelY_g(); Z = IMU.getAccelZ_g();
      if ((X*X + Y*Y + Z*Z) > TAKEOFF_ACCELERATION_SQ) break;
    }
  }
  Serial.println(F("FLYING!"));

  // The rocket is flying now
  FLIGHT_STATE = FlightState::_FLYING;
  before = micros();
}

//-----------------------------------------------------------------------------------------------
//---------------------loop function-------------------------------------------------------------

void loop() {
  bool flight_data_updated = false;

  // TODO: Send FLIGHT_STATE to the secondary flight computer EVERY 500ms!

  // Attempt to update flight data (orientation, position and velocity) from IMU
  if(IMU.tryReadSensor()) {
    deltat = micros() - before;
    deltat_sec = deltat / 1000000.0;

    // Update rotation of the sensor frame with respect to the NWU frame
    // where N is magnetic north, W is west and U is up.
    MadgwickAHRSupdate(IMU.getGyroX_rads(), IMU.getGyroY_rads(), IMU.getGyroZ_rads(),
                       0, 0, 0,
                       0, 0, 0, deltat_sec);

    // Make magnetic declination corrections to q_a
    quaternion_prod(q_magnetic_declination, q_a, q_a_tn);

    // Find acceleration vector in local NWU reference frame
    rotate_vector_by_quaternion(q_a_tn, IMU.getAccelX_g(), IMU.getAccelY_g(), IMU.getAccelZ_g(), X, Y, Z);
    lat_filter.set_deltat(deltat_sec); lat_filter.predict(X);
    lon_filter.set_deltat(deltat_sec); lon_filter.predict(-Y);
    alt_filter.set_deltat(deltat_sec); alt_filter.predict(Z - 1.0);

    // Display the data
    Serial.print(F("dt:\t"));
    Serial.print(deltat);
    Serial.print(F("\tX:\t"));
    print_int64_t(lat_filter.get_pos_mm());
    Serial.print(F("\tY:\t"));
    print_int64_t(lon_filter.get_pos_mm());
    Serial.print(F("\tZ:\t"));
    print_int64_t(alt_filter.get_pos_mm());
    Serial.print(F("\tVx:\t"));
    Serial.print(lat_filter.get_vel_mm_per_sec());
    Serial.print(F("\tVy:\t"));
    Serial.print(lon_filter.get_vel_mm_per_sec());
    Serial.print(F("\tVz:\t"));
    Serial.println(alt_filter.get_vel_mm_per_sec());
    Serial.flush();
    before += deltat;

    if (FLIGHT_STATE == FlightState::_FLYING) {
      // Find corrective actions ux, uy, uz
      rotate_vector_by_quaternion(q_magnetic_declination,
                                  IMU.getGyroX_rads(), IMU.getGyroY_rads(), IMU.getGyroZ_rads(),
                                  X, Y, Z);
      controller.compute(q_a_tn, X, Y, Z, ux, uy, uz);
      // Move corrections to fins
      fin_controller.makeFinCorrections(ux, uy, uz);
    }

    flight_data_updated = true;
  }

  // Get GPS data if available and 'update' the position and velocity 'prediction's
  if (neo6m.try_read_gps(lat_mm, lon_mm, alt_mm)) {
    lat_filter.update(lat_mm);
    lon_filter.update(lon_mm);
    alt_filter.update(alt_mm);

    flight_data_updated = true;
  }

  // TODO: If v_z variance is too high OR flight data has not been updated for 1 second,
  //       then send _MAIN_COMP_SAFE_FAIL to backup computer to fail safely! Then loop here forever.

  if (flight_data_updated) {
    if (FLIGHT_STATE == FlightState::_FLYING) {
      uint8_t first_bit = static_cast<uint8_t>(alt_filter.get_vel_mm_per_sec() <= 0.0);
      vz_neg_count -= vz_neg_q.push_first_pop_last(first_bit);
      vz_neg_count += first_bit;
      if (vz_neg_count > VZ_NEG_Q_BIT_SIZE * 2 / 3) {
        Serial.println(F("Apogee reached!"));
        // Rotate fins back to 0 degrees
        fin_controller.makeFinCorrections(0, 0, 0);

        // TODO: Initiate drogue recovery HERE!!!!!!!!!!!!!!
        FLIGHT_STATE = FlightState::_FALLING_FAST;
      }
    } else if (FLIGHT_STATE == FlightState::_FALLING_FAST) {
      if (alt_filter.get_pos_mm() < ground_alt_mm + MAIN_RECOVERY_ALTITUDE) {
        Serial.println(F("Less than 600m to ground!"));

        //TODO: Initiate main recovery HERE!!!!!!!!!!!!!!
        FLIGHT_STATE = FlightState::_FALLING_SLOW;
      }
    } else if (FLIGHT_STATE == FlightState::_FALLING_SLOW) {
    } else /*FlightState::_MAIN_COMP_SAFE_FAIL*/ {}
  }

  // If imu did not update for 500ms, then rotate fins back to 0 degrees
  if (FLIGHT_STATE == FlightState::_FLYING && (micros() - before) > 500000) {
    fin_controller.makeFinCorrections(0, 0, 0);
  }

#ifndef FIN_CONTROL_BY_SERVO
  // Let step motors run their ticks
  fin_controller.runSteppers();
#endif
}
