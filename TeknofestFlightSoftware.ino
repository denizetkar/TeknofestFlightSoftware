//------------------------------------------------------------------------------------------------
//---------------------includes-------------------------------------------------------------------

#include <MPU9250.h>
#include "MadgwickAHRS.h"
#include "QuaternionPID.h"

#include "Neo6MGPS.h"

#define TSA_TIME_IN 0
#define TSA_RESPONSE_TIME_OUT 10000
#define PJON_MAX_PACKETS 0
#define PJON_PACKET_MAX_LENGTH 16
#include <PJON.h>

#include "EarthPositionFilter.h"
#include "FinController.h"

#ifndef STM32_CORE_VERSION
#include "print_64bit.h"
#endif
#include "BitQueue.h"

//------------------------------------------------------------------------------------------------
//---------------------definitions----------------------------------------------------------------

#ifndef STM32_CORE_VERSION
#define IMU_CS_PIN 10
#else
#define IMU_MOSI_PIN PA7
#define IMU_MISO_PIN PA6
#define IMU_CLK_PIN PA5
#define IMU_CS_PIN PA4
#endif
#define IMU_CALIB_MAX_COUNT 500
#define MIN_NUM_GPS 6

// TODO: redefine the value below as at least (3g)^2=9 !
#define TAKEOFF_ACCELERATION_SQ 1.0
// The delay below must be >= 1000/(IMU Output Data Rate)
#define TAKEOFF_SENSING_DELAY 100

#ifndef STM32_CORE_VERSION
#define GPS_TX_PIN 2
#define GPS_RX_PIN 3
#else
#define GPS_TX_PIN PA3
#define GPS_RX_PIN PA2
#endif
#define GPS_BAUD_RATE 9600

#ifndef STM32_CORE_VERSION
#define REDUNDANT_COMP_TX_PIN 4
#define REDUNDANT_COMP_RX_PIN 5
#else
#define REDUNDANT_COMP_TX_PIN PB11
#define REDUNDANT_COMP_RX_PIN PB10
#endif
#define REDUNDANT_COMP_BAUD_RATE 9600
#define REDUNDANT_COMP_BUS_ID 44
#define MAIN_COMP_BUS_ID 45

// TODO: Decide which motor type to use for fin correction
#ifndef STM32_CORE_VERSION
#ifndef FIN_CONTROL_BY_SERVO
// 6, 7, 8, 9 --> 6, 8, 7, 9 (ATTENTION!)
#define CONTROLLER0_PINS { 6, 8, 7, 9 }
#define CONTROLLER1_PINS { NUM_DIGITAL_PINS, NUM_DIGITAL_PINS + 1, NUM_DIGITAL_PINS + 2, NUM_DIGITAL_PINS + 3 } /* NOT ENOUGH PINS ON UNO */
#define CONTROLLER2_PINS { NUM_DIGITAL_PINS, NUM_DIGITAL_PINS + 1, NUM_DIGITAL_PINS + 2, NUM_DIGITAL_PINS + 3 } /* NOT ENOUGH PINS ON UNO */
#define CONTROLLER3_PINS { NUM_DIGITAL_PINS, NUM_DIGITAL_PINS + 1, NUM_DIGITAL_PINS + 2, NUM_DIGITAL_PINS + 3 } /* NOT ENOUGH PINS ON UNO */
#else
#define CONTROLLER0_PINS 6
#define CONTROLLER1_PINS 7
#define CONTROLLER2_PINS 8
#define CONTROLLER3_PINS 9
#endif
#else
#ifndef FIN_CONTROL_BY_SERVO
// 6, 7, 8, 9 --> 6, 8, 7, 9 (ATTENTION!)
#define CONTROLLER0_PINS { PC15, PB9, PB8, PB7 }
#define CONTROLLER1_PINS { PB6, PB5, PB4, PB3 }
#define CONTROLLER2_PINS { PA15, PA12, PA11, PA8 }
#define CONTROLLER3_PINS { PB15, PB14, PB13, PB12 }
#else
#define CONTROLLER0_PINS PA6
#define CONTROLLER1_PINS PA7
#define CONTROLLER2_PINS PB0
#define CONTROLLER3_PINS PB1
#endif
#endif

#define VZ_NEG_Q_BIT_SIZE 512
#ifndef STM32_CORE_VERSION
#define MAIN_RECOVERY_ALTITUDE 600000
#else
#define MAIN_RECOVERY_ALTITUDE 600
#endif
#define MAIN_COMP_SAFE_FAIL_TIMEOUT 1000000

//------------------------------------------------------------------------------------------------
//---------------------setup and loop objects-----------------------------------------------------

// An MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
#ifndef STM32_CORE_VERSION
MPU9250 IMU(SPI, IMU_CS_PIN);
#else
SPIClass imu_spi(IMU_MOSI_PIN, IMU_MISO_PIN, IMU_CLK_PIN);
MPU9250 IMU(imu_spi, IMU_CS_PIN);
#endif
int status;

// A PID controller object
QuaternionPID controller{ 5.0, 0.5, 2.0 };

// The Neo-6M GPS object
Neo6MGPS neo6m(GPS_TX_PIN, GPS_RX_PIN);

// The serial object to communicate with the redundant computer
#ifndef STM32_CORE_VERSION
NeoSWSerial redundant_s(REDUNDANT_COMP_TX_PIN, REDUNDANT_COMP_RX_PIN);
#else
HardwareSerial redundant_s(REDUNDANT_COMP_TX_PIN, REDUNDANT_COMP_RX_PIN);
#endif

// Kalman Filter object for Latitude, Longitude, Altitude
EarthPositionFilter lat_filter, lon_filter, alt_filter;

// Servo object for controlling fins
#ifdef FIN_CONTROL_BY_STEP_MOTOR
FinController fin_controller(CONTROLLER0_PINS, CONTROLLER1_PINS, CONTROLLER2_PINS, CONTROLLER3_PINS);
#else
FinController fin_controller;
#endif

// Bit Queue for storing negativity condition of the latest measured Vz values
BitQueue<VZ_NEG_Q_BIT_SIZE> vz_neg_q;

//------------------------------------------------------------------------------------------------
//---------------------setup and loop constants---------------------------------------------------

// There is approximately 5.5 degrees East magnetic declination in Turkey on 24.02.2020.
// ( cos(-5.5*pi/180), 0, 0, sin(-5.5*pi/180) ) is the rotation quaternion required to
// rotate the true north frame into magnetic north frame
const double q_magnetic_declination[4] = {0.9953961983671789, 0, 0, -0.09584575252022398};

//------------------------------------------------------------------------------------------------
//---------------------setup and loop variables---------------------------------------------------

// Enumeration of flight states
enum FlightState : uint8_t {
  _BEFORE_FLIGHT = 0,
  _FLYING = 1,
  _FALLING_FAST = 2,
  _FALLING_SLOW = 3,
  _MAIN_COMP_SAFE_FAIL = 4
};
FlightState FLIGHT_STATE = FlightState::_BEFORE_FLIGHT;
#ifndef STM32_CORE_VERSION
double roll, pitch, yaw, X, Y, Z, ux, uy, uz, q_a_tn[4], deltat_sec;
int64_t lat_mm, lon_mm, alt_mm, ground_alt_mm;
#else
double roll, pitch, yaw, X, Y, Z, ux, uy, uz, q_a_tn[4], deltat_sec, lat_m, lon_m, alt_m, ground_alt_m;
#endif
uint32_t last_imu_read_time, last_gps_read_time, deltat;
uint8_t vz_neg_count = 0;

//------------------------------------------------------------------------------------------------
//---------------------setup function-------------------------------------------------------------

void setup() {
  PJON<ThroughSerialAsync> secure_rs(MAIN_COMP_BUS_ID);

  Serial.begin(230400);
  while (!Serial);

  // Give initial values of 0 degrees
  Serial.println(F("Initializing the fin controls..."));
#ifdef FIN_CONTROL_BY_STEP_MOTOR
  fin_controller.begin();
#else
  fin_controller.begin(CONTROLLER0_PINS, CONTROLLER1_PINS, CONTROLLER2_PINS, CONTROLLER3_PINS);
#endif
  fin_controller.makeFinCorrections(0, 0, 0);

  Serial.println(F("Initializing GPS module..."));
  neo6m.begin(GPS_BAUD_RATE);

  Serial.println(F("Initializing communication with redundant computer..."));
  redundant_s.begin(REDUNDANT_COMP_BAUD_RATE);
  secure_rs.strategy.set_serial(&redundant_s);
  secure_rs.include_sender_info(false);
  secure_rs.begin();

  // Start communication with IMU
  Serial.println(F("Initializing IMU..."));
  status = IMU.begin();
  if (status < 0) {
    Serial.print(F("IMU initialization unsuccessful: "));
    Serial.println(status);
    while (1);
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
  { {0.997971236148955, -0.00430622003848702, 0.00118009636690604},
    {0.00470700811404384, 0.998364615545652, -0.00638311318231112},
    {0.0012955016372647, 0.00387494101216781, 0.987478872856534}
  });
  // MAG bias
  IMU.setMagCalX(-41.334130);
  IMU.setMagCalY(17.872664);
  IMU.setMagCalZ(11.827231);
  IMU.setMagTM(
  { {0.023547, -0.000435, -0.002774},
    { -0.000435, 0.023738, 0.000579},
    { -0.002774, 0.000579, 0.024689}
  });

  // Calibrate the estimated orientation
  Serial.println(F("Calibrating orientation estimate..."));
  uint16_t imu_data_count = 0;
  while (true) {
    // Try to read the sensor
    if (IMU.tryReadSensor()) {

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
#ifndef STM32_CORE_VERSION
    if (neo6m.try_read_gps(lat_mm, lon_mm, alt_mm, MIN_NUM_GPS)) {
      // FOUND at least 5 GPS satellites!
      lat_filter.set_pos_mm(lat_mm);
      lon_filter.set_pos_mm(lon_mm);
      alt_filter.set_pos_mm(alt_mm);
      ground_alt_mm = alt_mm;
      break;
    }
#else
    if (neo6m.try_read_gps(lat_m, lon_m, alt_m, MIN_NUM_GPS)) {
      // FOUND at least 5 GPS satellites!
      lat_filter.set_pos_m(lat_m);
      lon_filter.set_pos_m(lon_m);
      alt_filter.set_pos_m(alt_m);
      ground_alt_m = alt_m;
      break;
    }
#endif
  }

  Serial.println(F("Sending FLIGHT_STATE to the redundant computer and waiting for ACK..."));
  while (secure_rs.send_packet(REDUNDANT_COMP_BUS_ID, &FLIGHT_STATE, 1) != PJON_ACK);
  // TODO: Let the ground station know that flight computer is READY.

  // Wait for high acceleration
  Serial.println(F("Waiting for liftoff before loop..."));
  while (true) {
    // Try to read the sensor
    if (IMU.tryReadSensor()) {
      X = IMU.getAccelX_g(); Y = IMU.getAccelY_g(); Z = IMU.getAccelZ_g();
      if ((X * X + Y * Y + Z * Z) > TAKEOFF_ACCELERATION_SQ) {
        delay(TAKEOFF_SENSING_DELAY);
        IMU.readSensor();
        X = IMU.getAccelX_g(); Y = IMU.getAccelY_g(); Z = IMU.getAccelZ_g();
        if ((X * X + Y * Y + Z * Z) > TAKEOFF_ACCELERATION_SQ) break;
      }
    }
  }
  Serial.println(F("FLYING!"));

  // The rocket is flying now
  FLIGHT_STATE = FlightState::_FLYING;
  while (secure_rs.send_packet(REDUNDANT_COMP_BUS_ID, &FLIGHT_STATE, 1) != PJON_ACK);
  last_imu_read_time = last_gps_read_time = micros();
}

//-----------------------------------------------------------------------------------------------
//---------------------loop function-------------------------------------------------------------

void loop() {
  bool flight_data_updated = false;

  // Send FLIGHT_STATE to the secondary flight computer !
  if (redundant_s.availableForWrite()) {
    redundant_s.write((uint8_t)FLIGHT_STATE);
  }

  // Attempt to update flight data (orientation, position and velocity) from IMU
  if (IMU.tryReadSensor()) {
    deltat = micros() - last_imu_read_time;
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
    // Calculate roll, pitch, yaw
    roll  = atan2(2.0 * (q_a_tn[0] * q_a_tn[1] + q_a_tn[2] * q_a_tn[3]), 1.0 - 2.0 * (q_a_tn[1] * q_a_tn[1] + q_a_tn[2] * q_a_tn[2]));
    pitch = asin(2.0 * (q_a_tn[0] * q_a_tn[2] - q_a_tn[1] * q_a_tn[3]));
    yaw   = atan2(2.0 * (q_a_tn[0] * q_a_tn[3] + q_a_tn[1] * q_a_tn[2]), 1.0 - 2.0 * (q_a_tn[2] * q_a_tn[2] + q_a_tn[3] * q_a_tn[3]));
    roll *= (180.0 / PI);
    pitch *= (180.0 / PI);
    yaw   *= (180.0 / PI);
    // Display the data
    Serial.print(F("dt: "));
    Serial.print(deltat);
    Serial.print(F("\tRoll: "));
    Serial.print(roll, 4);
    Serial.print(F("\tPitch: "));
    Serial.print(pitch, 4);
    Serial.print(F("\tYaw: "));
    Serial.print(yaw, 4);
#ifndef STM32_CORE_VERSION
    Serial.print(F("\tX: "));
    print_int64_t(lat_filter.get_pos_mm());
    Serial.print(F("\tY: "));
    print_int64_t(lon_filter.get_pos_mm());
    Serial.print(F("\tZ: "));
    print_int64_t(alt_filter.get_pos_mm());
#else
    Serial.print(F("\tX: "));
    Serial.print(lat_filter.get_pos_m());
    Serial.print(F("\tY: "));
    Serial.print(lon_filter.get_pos_m());
    Serial.print(F("\tZ: "));
    Serial.print(alt_filter.get_pos_m());
#endif
    Serial.print(F("\tvar(Vz): "));
    Serial.print(alt_filter.get_P(1, 1), 4);
    Serial.println();
    Serial.flush();
    last_imu_read_time += deltat;

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
#ifndef STM32_CORE_VERSION
  if (neo6m.try_read_gps(lat_mm, lon_mm, alt_mm)) {
    lat_filter.update(lat_mm);
    lon_filter.update(lon_mm);
    alt_filter.update(alt_mm);
    flight_data_updated = true;
    last_gps_read_time = micros();
  }
#else
  if (neo6m.try_read_gps(lat_m, lon_m, alt_m)) {
    lat_filter.update(lat_m);
    lon_filter.update(lon_m);
    alt_filter.update(alt_m);
    flight_data_updated = true;
    last_gps_read_time = micros();
  }
#endif

  // If flight data has not been updated for long enough by IMU or GPS,
  // then send _MAIN_COMP_SAFE_FAIL to backup computer to fail safely!
  deltat = micros();
  if (FLIGHT_STATE != FlightState::_MAIN_COMP_SAFE_FAIL)
  {
    if (deltat - last_imu_read_time > MAIN_COMP_SAFE_FAIL_TIMEOUT) {
      Serial.println(F("Main computer failed safely: IMU data lost!"));
      FLIGHT_STATE = FlightState::_MAIN_COMP_SAFE_FAIL;
      fin_controller.makeFinCorrections(0, 0, 0);
    }
    else if (deltat - last_gps_read_time > MAIN_COMP_SAFE_FAIL_TIMEOUT * 10) {
      Serial.println(F("Main computer failed safely: GPS data lost!"));
      FLIGHT_STATE = FlightState::_MAIN_COMP_SAFE_FAIL;
      fin_controller.makeFinCorrections(0, 0, 0);
    }
  }

  if (flight_data_updated) {
    if (FLIGHT_STATE == FlightState::_FLYING) {
#ifndef STM32_CORE_VERSION
      uint8_t first_bit = static_cast<uint8_t>(alt_filter.get_vel_mm_per_sec() <= 0);
#else
      uint8_t first_bit = static_cast<uint8_t>(alt_filter.get_vel_m_per_sec() <= 0.0);
#endif
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
#ifndef STM32_CORE_VERSION
      if (alt_filter.get_pos_mm() < ground_alt_mm + MAIN_RECOVERY_ALTITUDE) {
#else
      if (alt_filter.get_pos_m() < ground_alt_m + MAIN_RECOVERY_ALTITUDE) {
#endif
        Serial.println(F("Less than 600m to ground!"));

        //TODO: Initiate main recovery HERE!!!!!!!!!!!!!!
        FLIGHT_STATE = FlightState::_FALLING_SLOW;
      }
    } else if (FLIGHT_STATE == FlightState::_FALLING_SLOW) {
    } else { /*FlightState::_MAIN_COMP_SAFE_FAIL*/ }
  }

  // Let the motors run
  fin_controller.runMotors();
}
