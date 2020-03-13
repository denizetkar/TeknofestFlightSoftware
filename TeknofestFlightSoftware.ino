//------------------------------------------------------------------------------------------------
//---------------------includes-------------------------------------------------------------------

#include <MPU9250.h>
#include "MadgwickAHRS.h"
#include "QuaternionPID.h"

//TODO: After porting the code to STM32: Read positions from GPS in meters!
#include "Neo6MGPS.h"

//TODO: After porting the code to STM32: Turn int64_t variables into 'double' and track position in meters!
#include "EarthPositionFilter.h"
#include <Servo.h>

//TODO: After porting the code to STM32: No longer use "print_64bit.h"
#include "print_64bit.h"
#include "BitQueue.h"
//------------------------------------------------------------------------------------------------
//---------------------definitions----------------------------------------------------------------

#define IMU_CALIB_MAX_COUNT 500
#define MIN_NUM_GPS 5

//TODO: redefine the value below as at least (4g)^2=16 !
#define TAKEOFF_ACCELERATION_SQ 1.0

#define GPS_RX_PIN 3
#define GPS_TX_PIN 4
#define GPS_BAUD_RATE 9600

#define SERVO0_PIN 9
#define SERVO1_PIN 10
#define SERVO2_PIN 11
#define SERVO3_PIN 12
#define SERVO_ZERO_ANGLE 90

#define VZ_NEG_Q_BIT_SIZE 192
//TODO: After porting the code to STM32: Change this altitude into meters!
#define MAIN_RECOVERY_ALTITUDE 600000

//------------------------------------------------------------------------------------------------
//---------------------setup and loop objects-----------------------------------------------------

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);
int status;

// a PID controller object
QuaternionPID controller{ 50.0, 0.5, 1.0 };

// The Neo-6M GPS object
Neo6MGPS neo6m(GPS_TX_PIN, GPS_RX_PIN);

// Kalman Filter object for Latitude, Longitude, Altitude
EarthPositionFilter lat_filter, lon_filter, alt_filter;

// servo object for controlling fins
Servo servo0, servo1, servo2, servo3;

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
float roll, pitch, yaw, gx, gy, gz, ax, ay, az, ux, uy, uz, q_a_tn[4], deltat_sec;
//TODO: After porting the code to STM32: Change below positions to 'double' and into meters!
int64_t lat_mm, lon_mm, alt_mm, ground_alt_mm;
uint32_t before = 0, deltat;
uint8_t vz_neg_count = 0;

//------------------------------------------------------------------------------------------------
//---------------------other functions------------------------------------------------------------

void makeFinCorrections() {
  // servo0 looks towards +X, servo2 looks towards -X
  servo0.write(SERVO_ZERO_ANGLE - ux + uz);
  servo2.write(SERVO_ZERO_ANGLE + ux + uz);
  // servo1 looks towards -Y, servo3 looks towards +Y
  servo1.write(SERVO_ZERO_ANGLE + uy + uz);
  servo3.write(SERVO_ZERO_ANGLE - uy + uz);
}

//------------------------------------------------------------------------------------------------
//---------------------setup function-------------------------------------------------------------

void setup() {
  // serial to display data
  Serial.begin(2000000);
  while (!Serial);

  // attach servos
  Serial.println("Initializing the fin controls...");
  servo0.attach(SERVO0_PIN);
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo3.attach(SERVO3_PIN);
  // give initial values of 0 degrees
  ux = 0.0; uy = 0.0; uz = 0.0;
  makeFinCorrections();

  Serial.println("Initializing GPS module...");
  neo6m.begin(GPS_BAUD_RATE);

  // start communication with IMU
  Serial.println("Initializing IMU...");
  status = IMU.begin();
  if (status < 0) {
    Serial.print("IMU initialization unsuccessful: ");
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
  IMU.setMagCalX(-41.774776);
  IMU.setMagCalY(16.272968);
  IMU.setMagCalZ(13.122816);
  IMU.setMagTM(
    {{0.021566,0.000286,-0.002510},
     {0.000286,0.022636,0.000479},
     {-0.002510,0.000479,0.021711}});

  // calibrate the estimated orientation
  Serial.println("Calibrating orientation estimate...");
  uint16_t imu_data_count = 0;
  while (true) {
    if(IMU.isDataReady()) {
      // read the sensor
      IMU.readSensor();

      // Update rotation of the sensor frame with respect to the NWU frame
      // where N is magnetic north, W is west and U is up.
      MadgwickAHRSupdate(0, 0, 0,
                         IMU.getAccelX_g(), IMU.getAccelY_g(), IMU.getAccelZ_g(),
                         IMU.getMagX_uT(), IMU.getMagY_uT(), IMU.getMagZ_uT(),
                         0.01 + 0.09 * cos(imu_data_count * PI / (2 * IMU_CALIB_MAX_COUNT)));
      imu_data_count++;
      // calibration is done after about 3 seconds
      if (imu_data_count >= IMU_CALIB_MAX_COUNT) {
        break;
      }
    }
  }

  // Make sure our GPS module uses enough GPS satellites and initialize current position.
  Serial.println("Searching for GPS satellites...");
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

  //TODO: Send _BEFORE_FLIGHT signal to the backup computer, and wait for transmission!
  //TODO: Let the ground station know that flight computer is READY.

  // wait for high acceleration
  Serial.println("Waiting for liftoff before loop...");
  while (true) {
    if(IMU.isDataReady()) {
      // read the sensor
      IMU.readSensor();
      ax = IMU.getAccelX_g(); ay = IMU.getAccelY_g(); az = IMU.getAccelZ_g();
      if ((ax*ax + ay*ay + az*az) > TAKEOFF_ACCELERATION_SQ) break;
    }
  }
  Serial.println("FLYING!");

  // The rocket is flying now
  FLIGHT_STATE = FlightState::_FLYING;
  before = micros();
}

//-----------------------------------------------------------------------------------------------
//---------------------loop function-------------------------------------------------------------

void loop() {
  bool flight_data_updated = false;

  //TODO: Send FLIGHT_STATE to the secondary flight computer EVERY 500ms!

  // Attempt to update flight data (orientation, position and velocity) from IMU
  if(IMU.isDataReady()) {
    deltat = micros() - before;
    deltat_sec = deltat / 1000000.0;
    // read the sensor
    IMU.readSensor();

    // Update rotation of the sensor frame with respect to the NWU frame
    // where N is magnetic north, W is west and U is up.
    MadgwickAHRSupdate(IMU.getGyroX_rads(), IMU.getGyroY_rads(), IMU.getGyroZ_rads(),
                       0, 0, 0,
                       0, 0, 0, deltat_sec);

    // make magnetic declination corrections to q_a
    quaternion_prod(q_magnetic_declination, q_a, q_a_tn);

    // find acceleration vector in local NWU reference frame
    rotate_vector_by_quaternion(q_a_tn, IMU.getAccelX_g(), IMU.getAccelY_g(), IMU.getAccelZ_g(), ax, ay, az);
    lat_filter.set_deltat(deltat_sec); lat_filter.predict(ax);
    lon_filter.set_deltat(deltat_sec); lon_filter.predict(-ay);
    alt_filter.set_deltat(deltat_sec); alt_filter.predict(az - 1.0);

    // display the data
    Serial.print("dt:\t");
    Serial.print(deltat);
    Serial.print("\tX:\t");
    print_int64_t(lat_filter.get_pos_mm());
    Serial.print("\tY:\t");
    print_int64_t(lon_filter.get_pos_mm());
    Serial.print("\tZ:\t");
    print_int64_t(alt_filter.get_pos_mm());
    Serial.print("\tVX:\t");
    Serial.print(lat_filter.get_vel_mm_per_sec());
    Serial.print("\tVY:\t");
    Serial.print(lon_filter.get_vel_mm_per_sec());
    Serial.print("\tVZ:\t");
    Serial.println(alt_filter.get_vel_mm_per_sec());
    before += deltat;

    if (FLIGHT_STATE == FlightState::_FLYING) {
      // find corrective actions ux, uy, uz
      rotate_vector_by_quaternion(q_magnetic_declination,
                                  IMU.getGyroX_rads(), IMU.getGyroY_rads(), IMU.getGyroZ_rads(),
                                  gx, gy, gz);
      controller.compute(q_a_tn, gx, gy, gz, ux, uy, uz);
      // move corrections to fins
      makeFinCorrections();
    }

    flight_data_updated = true;
  }

  // If imu did not update for 500ms, then rotate fins back to 0 degrees
  if (micros() - before > 500000) {
    ux = 0.0; uy = 0.0; uz = 0.0;
    makeFinCorrections();
  }

  // get gps data if available and 'update' the position and velocity 'prediction's
  if (neo6m.try_read_gps(lat_mm, lon_mm, alt_mm)) {
    lat_filter.update(lat_mm);
    lon_filter.update(lon_mm);
    alt_filter.update(alt_mm);

    flight_data_updated = true;
  }

  //TODO: If v_z variance is too high OR flight data has not been updated for 1 second,
  //      then send _MAIN_COMP_SAFE_FAIL to backup computer to fail safely! Then loop here forever.

  if (flight_data_updated) {
    switch (FLIGHT_STATE) {
      case FlightState::_FLYING:
        uint8_t first_bit = static_cast<uint8_t>(alt_filter.get_vel_mm_per_sec() <= 0.0);
        vz_neg_count -= vz_neg_q.push_first_pop_last(first_bit);
        vz_neg_count += first_bit;
        if (vz_neg_count > VZ_NEG_Q_BIT_SIZE * 2 / 3) {
          //TODO: Initiate drogue recovery HERE!!!!!!!!!!!!!!
          FLIGHT_STATE = FlightState::_FALLING_FAST;
        }
        break;
      case FlightState::_FALLING_FAST:
        if (alt_filter.get_pos_mm() < ground_alt_mm + MAIN_RECOVERY_ALTITUDE) {
          //TODO: Initiate main recovery HERE!!!!!!!!!!!!!!
          FLIGHT_STATE = FlightState::_FALLING_SLOW;
        }
        break;
      case FlightState::_FALLING_SLOW:
        break;
      case FlightState::_MAIN_COMP_SAFE_FAIL:
        break;
    }
  }
}
