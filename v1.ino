#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SENSOR_SAMPLERATE_DELAY_MS (10)

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp; // I2C

Adafruit_BNO055 bno = Adafruit_BNO055();

int iter = 0;
int alt_iter = 0;
double ground = 0;
double alt = 0;
const int average_number_over = 5;
double   alt_array[60*1000/10/5];

/*****************************************************************************************/
/*****************************************************************************************/
/*                                    helper functions                                   */
/*****************************************************************************************/
/*****************************************************************************************/

// BNO basic info
void displayBNODetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

// BNO calibration status
void displayBNOCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}

// basic info about BNO
void displayBNOStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

/*****************************************************************************************/
/*****************************************************************************************/
/*                                         setup                                         */
/*****************************************************************************************/
/*****************************************************************************************/
void setup() {
  
  Serial.begin(9600);
  Serial.println("Data Collection for Orientation, Acceleration, and Altitude");
  Serial.println("");

  /* Initialise the sensors */
  
  // bno055 -----------------------------------------------------------------------
  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  displayBNODetails();
  displayBNOCalStatus();
  displayBNOStatus();

  bno.setExtCrystalUse(true);

  // bmp388 -----------------------------------------------------------------------
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  Serial.println("Setup-------------------------------");

  bmp.readAltitude(SEALEVELPRESSURE_HPA);
  delay(3000);
  for (int i=0; i<average_number_over; i++) {
    ground += bmp.readAltitude(SEALEVELPRESSURE_HPA);
  }
  ground = ground/average_number_over;
  Serial.println("Ground altitude= ");
  Serial.print(ground);
  Serial.println("");

  delay(1000);
}

/*****************************************************************************************/
/*****************************************************************************************/
/*                                         main                                          */
/*****************************************************************************************/
/*****************************************************************************************/
void loop() {

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  
  if (! bmp.performReading()) {
    Serial.println("Failed to perform BMP reading :(");
    return;
  }

  Serial.print("Time elapsed: ");
  Serial.print(millis());
  Serial.print("\tIterations: ");
  Serial.print(iter);
  Serial.println("");
  Serial.println("-----------------------------------------------------------");
  
  // Orientation
  sensors_event_t event;
  bno.getEvent(&event);
  Serial.print("Orientation:");
  Serial.println("");
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.println("");

  // Acceleration
  Serial.print("Acceleration:");
  Serial.println("");
  Serial.print("X: ");
  Serial.print(euler.x());
  Serial.print("\tY: ");
  Serial.print(euler.y());
  Serial.print("\tZ: ");
  Serial.print(euler.z());
  Serial.println("");

  // Pressure
  Serial.print("Pressure:");
  Serial.println("");
  Serial.print(bmp.pressure / 100.0);
  Serial.print(" hPa");
  Serial.println("");

  // Altitude
  Serial.print("Approx. Altitude: ");
  double new_alt = bmp.readAltitude(SEALEVELPRESSURE_HPA) - ground;
  alt += new_alt;
  Serial.print(new_alt);
  Serial.print(" m");
  Serial.println("\t");

  Serial.println("");
  Serial.println("");

  if (iter%average_number_over == 0 && iter != 0){
    alt = alt/average_number_over;
    Serial.print("Averaged altitude over ");
    Serial.print(average_number_over);
    Serial.print(" is: ");
    Serial.print(alt);
    Serial.println("");
    alt_array[alt_iter] = alt;
    alt_iter++;
    alt = 0;
  }
  iter++;
  delay(SENSOR_SAMPLERATE_DELAY_MS);
}
