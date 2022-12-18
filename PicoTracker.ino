/*
 * #################################
 * HATIRE HEAD TRACKER FOR OPENTRACK
 * #################################
 * 
 * This program uses the Rpi Pico microcontroller,
 * MPU6050 Inertial Measurement Unit, and the QMC5883L
 * magnetometer to estimate orientation and output HATIRE
 * data for OpenTrack
 * 
 * Customize the code as its fit to you and install the necessary dependencies
 * 
 * If the sensors or microcontroller above aren't being used
 * major code modification will be necessary, please refer to the comments below
 * 
 * LICENSE
 * 
 * O. Rangel Morales
 * (GitHub: OrlandoR4)
 * MIT license, all text above must be included in any redistribution
 */

#include <Arduino.h>

#include "Wire.h"
#include "EEPROM.h"
#include "Quaternion.h"

#include "Adafruit_Sensor.h"
#include "Adafruit_MPU6050.h"
#include "QMC5883LCompass.h"

// Inertial Measurements
Vector3 acceleration_body;
Vector3 angular_velocity;
Vector3 magnetic_flux;

// Orientation
Quaternion true_orientation;
Quaternion offset_quaternion;
Quaternion orientation;

Vector3 euler_angles;
Vector3 acceleration_inertial;

// Calibration Variables
Vector3 offset_angular_velocity;
Vector3 offset_acceleration_body;
Vector3 offset_magnetic_flux;
Vector3 scalar_magnetic_flux(1, 1, 1);

// Orientation Filter
const float orientation_filter_frequency = 300;
const float correction_gain = 0.05;

// EEPROM per 32 Bytes
// 0 -> 2  : gyroscopic offsets
// 3 -> 5  : acceleration offsets
// 6 -> 8  : magnetic offsets
// 9 -> 11 : magnetic scalar
union data_union
{
  struct data_struct
  {
    float off_x_g;
    float off_y_g;
    float off_z_g;

    float off_x_a;
    float off_y_a;
    float off_z_a;

    float off_x_m;
    float off_y_m;
    float off_z_m;

    float scal_x_m;
    float scal_y_m;
    float scal_z_m;
  } calibration_struct;
  char calibration_string[sizeof(data_struct)];
} calibration_data;

// HATIRE
struct  {
  int16_t  Begin;           // 2  Begin
  uint16_t Cpt;             // 2  Computer frame or Code
  float    euler_angles[3]; // 12 [Yaw, Pitch, Roll] Euler angles in degrees
  float    position[3];     // 12 [x, y, z] Position in meters
  int16_t  End;             // 2  End
} hatire;

/**
 * Customize your sensors here!
 * 
 * Change the code to fit your custom sensors if you're not
 * using the QMC5883L and the MPU6050
 */
Adafruit_MPU6050 MPU_6050;
QMC5883LCompass QMC_5883;
/**
 * Customize your pin settings here!
 */
const int LED = LED_BUILTIN;
const int BUTTON = 17;
const int SCL_PIN = 21;
const int SDA_PIN = 20;
/**
 * Customize your sensor readings here!
 * 
 * Change the axes of the sensors depending on how your sensors are oriented
 * 
 * Change the code to fit your custom sensors if you're not
 * using the QMC5883L and the MPU6050
 */
void read_sensors()
{
  // Read IMU data
  sensors_event_t a, g, temp;
  MPU_6050.getEvent(&a, &g, &temp);
  // Radians per second
  angular_velocity.x = -g.gyro.y;
  angular_velocity.y = -g.gyro.x;
  angular_velocity.z = -g.gyro.z;
  // Meters per second squared
  acceleration_body.x = -a.acceleration.y;
  acceleration_body.y = -a.acceleration.x;
  acceleration_body.z = -a.acceleration.z;

  // Read Magnetometer Data
  QMC_5883.read();
  // Micro-teslas
  magnetic_flux.x =  QMC_5883.getX();
  magnetic_flux.y = -QMC_5883.getY();
  magnetic_flux.z = -QMC_5883.getZ();
}
// LED blink
void blink_LED(uint32_t frequency_ms)
{
  static bool state_LED;
  static uint32_t previous_time = 0;

  if (millis() - previous_time > frequency_ms)
  {
    previous_time = millis();

    state_LED = !state_LED;
    digitalWrite(LED, state_LED);
  }
}
// Error message to display to the serial monitor, LED blinks
void error_message(const String error_message_string)
{
  blink_LED(250);

  static uint32_t previous_time = 0;
  if (millis() - previous_time > 250)
  {
    previous_time = millis();

    Serial.println(error_message_string);
  }
}
// Gyroscope Calibration
void calibrate_gyroscope(uint32_t timer_trigger_ms)
{
  // Reset offsets
  offset_angular_velocity *= 0;

  // Track time
  uint32_t time = millis();
  uint32_t previous_time = millis();
  uint32_t delta_time = 0;
  uint32_t calibration_timer = 0;

  while (true)
  {
    // Timekeeping
    previous_time = time;
    time = millis();
    delta_time = time - previous_time;

    // Blink LED for indication
    blink_LED(50);

    // Get IMU data
    read_sensors();

    // Only log data if the gyroscope isn't being moved a lot
    if (angular_velocity.get_magnitude() < 15.f * DEG_TO_RAD)
    {
      // Add time
      calibration_timer += delta_time;

      // Add bias to the offset
      offset_angular_velocity += angular_velocity * float(delta_time);

      if (calibration_timer > timer_trigger_ms)
      {
        // Average the measurements over the elapsed time
        offset_angular_velocity /= float(calibration_timer);

        break;
      }
    }
  }

  return;
}
// Accelerometer Calibration
void calibrate_accelerometer(uint32_t timer_trigger_ms)
{
  // Reset offsets
  offset_acceleration_body *= 0;

  // Track time
  uint32_t time = millis();
  uint32_t previous_time = millis();
  uint32_t delta_time = 0;
  uint32_t calibration_timer = 0;

  while (true)
  {
    // Timekeeping
    previous_time = time;
    time = millis();
    delta_time = time - previous_time;

    // Blink LED for indication
    blink_LED(150);

    // Get IMU data
    read_sensors();

    // Only log data if the accelerometer isn't being moved a lot
    if (acceleration_body.get_magnitude() < 15)
    {
      // Add time
      calibration_timer += delta_time;

      // Add bias to the offset
      offset_acceleration_body += acceleration_body * float(delta_time);

      if (calibration_timer > timer_trigger_ms)
      {
        // Average the measurements over the elapsed time
        offset_acceleration_body /= float(calibration_timer);

        // Add gravity offset
        offset_acceleration_body.z += 9.8066f;

        break;
      }
    }
  }

  return;
}
// Magnetometer Calibration
void calibrate_magnetometer(uint32_t timer_trigger_ms)
{
  // Reset offsets
  offset_magnetic_flux *= 0;
  scalar_magnetic_flux = Vector3(1, 1, 1);

  // Track time
  uint32_t time = millis();
  uint32_t previous_time = millis();
  uint32_t delta_time = 0;
  uint32_t calibration_timer = 0;

  // Magnetometer calibration variables
  float min[3] = {0};
  float max[3] = {0};

  while (true)
  {
    // Timekeeping
    previous_time = time;
    time = millis();
    delta_time = time - previous_time;

    // Get magnetometer data
    read_sensors();

    // Track if the acceleration has changed
    bool changed = false;

    if (magnetic_flux.x > max[0]) { max[0] = magnetic_flux.x; changed = true; }
    if (magnetic_flux.y > max[1]) { max[1] = magnetic_flux.y; changed = true; }
    if (magnetic_flux.z > max[2]) { max[2] = magnetic_flux.z; changed = true; }

    if (magnetic_flux.x < min[0]) { min[0] = magnetic_flux.x; changed = true; }
    if (magnetic_flux.y < min[1]) { min[1] = magnetic_flux.y; changed = true; }
    if (magnetic_flux.z < min[2]) { min[2] = magnetic_flux.z; changed = true; }

    if (!changed)
    {
      // Turn off LED for indication
      digitalWrite(LED, LOW);

      // Add time
      calibration_timer += delta_time;
    }else{
      // Blink LED for indication
      blink_LED(50);

      // Reset time
      calibration_timer = 0;
    }

    // Exit if timer is satisfied
    if (calibration_timer > timer_trigger_ms)
      break;
  }
  
  const float ideal_halfscale = 1.f;

  scalar_magnetic_flux.x = 2.0f * ideal_halfscale / ( abs(max[0]) + abs(min[0]) );
  scalar_magnetic_flux.y = 2.0f * ideal_halfscale / ( abs(max[1]) + abs(min[1]) );
  scalar_magnetic_flux.z = 2.0f * ideal_halfscale / ( abs(max[2]) + abs(min[2]) );

  offset_magnetic_flux.x = (max[0] * scalar_magnetic_flux.x) - ideal_halfscale;
  offset_magnetic_flux.y = (max[1] * scalar_magnetic_flux.y) - ideal_halfscale;
  offset_magnetic_flux.z = (max[2] * scalar_magnetic_flux.z) - ideal_halfscale;

  return;
}
// Update attitude
void update_attitude()
{
  static uint32_t previous_time = 0;

  if (millis() - previous_time > uint32_t((1000.f)/(orientation_filter_frequency)))
  {
    // Timekeeping
    float delta_time_s = float(millis() - previous_time)*0.001f;
    previous_time = millis();

    // Update sensor data
    read_sensors();

    angular_velocity -= offset_angular_velocity;
    acceleration_body -= offset_acceleration_body;
    magnetic_flux = magnetic_flux * scalar_magnetic_flux - offset_magnetic_flux;

    // Update filter
    Vector3 ori_acceleration_inertial = true_orientation.rotate_vector(acceleration_body);

    true_orientation.update_with_rates(delta_time_s, angular_velocity);
    true_orientation.update_with_accel(ori_acceleration_inertial, Vector3(0, 0, -1), correction_gain);
    true_orientation.update_with_mag(magnetic_flux, acceleration_body, Vector3(0, 1, 0), Quaternion(1, 0, 0, 0), correction_gain);

    // Update output quaternion
    orientation = offset_quaternion * true_orientation;

    // Update inertial acceleration
    acceleration_inertial = orientation.rotate_vector(acceleration_body);
    acceleration_inertial.z += 9.8066f;
    
    // Update euler angles
    euler_angles = orientation.quaternion_to_euler();
  }
}
/**
 * Customize your initialization code here!
 * 
 * Change the sensors to your desired sensors or leave as is if you're using the
 * QMC5883L and the MPU6050
 * 
 * Change the pin configuration as well if you're not using a Raspberry Pi Pico
 */
void setup()
{
  // Initialize Serial Port
  Serial.begin(115200);

  // Begin EEPROM
  EEPROM.begin(256);

  // Initialize I2C Pins (IF USING RASPBERRY PI PICO)
  Wire.setSCL(SCL_PIN);
  Wire.setSDA(SDA_PIN);

  // Initialize pins
  pinMode(LED, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);

  // Initialize MPU6050
  while (!MPU_6050.begin())
    error_message("Failed to initialize MPU6050");

  // Initialize QMC5883
  QMC_5883.init();
  Wire.beginTransmission(0x0D);
  if (Wire.endTransmission()) { error_message("Failed to initialize QMC5883"); }

  // Configure MPU6050
  MPU_6050.setAccelerometerRange(MPU6050_RANGE_4_G);
  MPU_6050.setGyroRange(MPU6050_RANGE_2000_DEG);
  MPU_6050.setFilterBandwidth(MPU6050_BAND_94_HZ);

  // Configure HMC5883
  QMC_5883.setMode(0x01, 0x0C, 0x10, 0x00);
  
  // Setup HATIRE
  hatire.Begin = 0xAAAA;
  hatire.Cpt = 0;
  hatire.End = 0x5555;

  // Initialization Miscellaneous
  Serial.println("Successful Initialization!");

  // Optional Calibration
  if (!digitalRead(BUTTON))
  {
    // Calibrate Gyroscope
    Serial.println("Calibrating gyroscope, do not move the sensor...");
    calibrate_gyroscope(5000);

    // Calibrate Accelerometer
    Serial.println("Calibrating gyroscope, do not move the sensor...");
    calibrate_accelerometer(5000);
    
    // Calibrate Magnetometer
    Serial.println("Calibrating magnetometer, gently spin around the sensor on all axis...");
    calibrate_magnetometer(5000);

    // Set values
    calibration_data.calibration_struct.off_x_g  = offset_angular_velocity.x;
    calibration_data.calibration_struct.off_y_g  = offset_angular_velocity.y;
    calibration_data.calibration_struct.off_z_g  = offset_angular_velocity.z;
    calibration_data.calibration_struct.off_x_a  = offset_acceleration_body.x;
    calibration_data.calibration_struct.off_y_a  = offset_acceleration_body.y;
    calibration_data.calibration_struct.off_z_a  = offset_acceleration_body.z;
    calibration_data.calibration_struct.off_x_m  = offset_magnetic_flux.x;
    calibration_data.calibration_struct.off_y_m  = offset_magnetic_flux.y;
    calibration_data.calibration_struct.off_z_m  = offset_magnetic_flux.z;
    calibration_data.calibration_struct.scal_x_m = scalar_magnetic_flux.x;
    calibration_data.calibration_struct.scal_y_m = scalar_magnetic_flux.y;
    calibration_data.calibration_struct.scal_z_m = scalar_magnetic_flux.z;

    // Store Calibration
    EEPROM.put(0, calibration_data.calibration_string);
    EEPROM.commit();
  }
  else
  {
    EEPROM.get(0, calibration_data.calibration_string);

    // Set values
    offset_angular_velocity.x = calibration_data.calibration_struct.off_x_g ;
    offset_angular_velocity.y = calibration_data.calibration_struct.off_y_g ;
    offset_angular_velocity.z = calibration_data.calibration_struct.off_z_g ;
    offset_acceleration_body.x = calibration_data.calibration_struct.off_x_a ;
    offset_acceleration_body.y = calibration_data.calibration_struct.off_y_a ;
    offset_acceleration_body.z = calibration_data.calibration_struct.off_z_a ;
    offset_magnetic_flux.x = calibration_data.calibration_struct.off_x_m ;
    offset_magnetic_flux.y = calibration_data.calibration_struct.off_y_m ;
    offset_magnetic_flux.z = calibration_data.calibration_struct.off_z_m ;
    scalar_magnetic_flux.x = calibration_data.calibration_struct.scal_x_m;
    scalar_magnetic_flux.y = calibration_data.calibration_struct.scal_y_m;
    scalar_magnetic_flux.z = calibration_data.calibration_struct.scal_z_m;
  }

  // Begin Loop
  digitalWrite(LED, HIGH);
  delay(1000);
}
// Loop
void loop()
{
  // Update Orientation
  update_attitude();

  // Zero Orientation until released
  if (!digitalRead(BUTTON))
  {
    offset_quaternion = true_orientation.conjugate();
    blink_LED(100);
  }
  else
  {
    blink_LED(1000);
  }
  
  // Set hatire values
  hatire.euler_angles[0] = euler_angles.z * RAD_TO_DEG;
  hatire.euler_angles[1] = euler_angles.y * RAD_TO_DEG;
  hatire.euler_angles[2] = euler_angles.x * RAD_TO_DEG;

  hatire.position[0] = euler_angles.y * RAD_TO_DEG / 4.5f;
  hatire.position[1] = euler_angles.x * RAD_TO_DEG / 4.5f;
  hatire.position[2] = euler_angles.z * RAD_TO_DEG / 4.5f;

  // Write to hatire
  Serial.write((byte*)&hatire,30);
  hatire.Cpt++;
  if (hatire.Cpt > 999) 
    hatire.Cpt=0;

  // DEBUG
  /*Serial.print(angular_velocity.x, 1); Serial.print(", ");
  Serial.print(angular_velocity.y, 1); Serial.print(", ");
  Serial.print(angular_velocity.z, 1); Serial.print(", ");

  Serial.print(acceleration_body.x, 1); Serial.print(", ");
  Serial.print(acceleration_body.y, 1); Serial.print(", ");
  Serial.print(acceleration_body.z, 1); Serial.print(", ");

  Serial.print(magnetic_flux.x, 1); Serial.print(", ");
  Serial.print(magnetic_flux.y, 1); Serial.print(", ");
  Serial.print(magnetic_flux.z, 1); Serial.println("");*/

  /*Serial.print(euler_angles.x * RAD_TO_DEG, 3); Serial.print(", ");
  Serial.print(euler_angles.y * RAD_TO_DEG, 3); Serial.print(", ");
  Serial.print(euler_angles.z * RAD_TO_DEG, 3); Serial.println("");*/
}
