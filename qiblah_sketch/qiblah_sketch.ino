/*
  Qibla finder - Open-source qibla finder with tilt compensation using 9-DoF IMU and GPS

  The circuit:
  - Arduino board
  - BMI270 shuttle board
  - Adafruit Mini GPS PA1010D
  - Monochrome 0.91" 128x32 I2C OLED Display

  Documentation:
  - 
  
  References: 
  - https://atadiat.com/en/e-towards-understanding-imu-basics-of-accelerometer-and-gyroscope-sensors/
  - https://atadiat.com/en/e-magnetometer-soft-iron-and-hard-iron-calibration-why-how/
  - https://atadiat.com/en/e-towards-understanding-imu-frames-vpython-visualize-orientation/   
  
  created 14 May 2023
  by Yahya Tawil
*/

/*
 * 
 * 
 * Hardware
┌──────────────────┐
│                  │
│ Arduino Board    │
│                  │
│                  │
│   SDA SCL    INT │
└──────┬┬─────┬──┬─┘
       ││ MAG_│  │IMU_
       ││ DRDY│  │DRDY
    I2C││     │  │
       ││     │  │
       ││     │  │
    ┌──┴┴─┐  ┌┴──┴─┐   ┌────────────┐
    │Scree│  │BMI_ │   │            │
    │  n  ├──┤270  ├───┤    GPS     │
    │     │  │ BMM_│   │  (optional)│
    │     ├──┤150  ├───┤            │
    └─────┘  └─────┘   └────────────┘
           I2C      I2C
           
 */
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GPS.h>
#include "BMI270_AUX_BMM150.h"
#include "logos.h"
#include "math.h"
#include "MadgwickAHRS.h"
#include "Wire.h"

// pins from data ready interrupt (IMU and Mag)
#define IMU_INT_PIN 25
#define MAG_INT_PIN 26 
/**********************/

// Connect to the GPS on the hardware I2C port
Adafruit_GPS GPS(&Wire);
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false
/**********************/

// Macros define program behaviour (disagned to enable one at once)
// print the pitch,roll, and yaw beside the compass values
// before and after tilt compensation 
#define COMPASS_LOG 0
#define MAG_CALIBRATION_LOG 0
// print on the console the magnetometer values after applying the calibration arrays 
#define MAG_CALIBRATED_LOG 0 
// print the IMU data in the format needed for calibration software tool
#define MOTION_CAL_SOFT 0
// print GYRO output for calibration purposes
#define GYRO_CAL 0
// send pitch,roll, and yaw only. This is for 3D visualization
// https://atadiat.com/en/e-towards-understanding-imu-frames-vpython-visualize-orientation/
#define VISUAL_3D 0
// Set to 1 if GPS reciver is used 
#define GPS_EN 1
/**********************/

// Mag. calibration
// Hard ironing
//B 
#define B1 -3.48
#define B2 0.42
#define B3 -5.78

//Soft ironing
//H column1
#define H11 1.036
#define H21 -0.023
#define H31 0.009
//H column2
#define H12 -0.023
#define H22 1.010
#define H32 -0.008
//H column3
#define H13 0.009
#define H23 -0.008
#define H33 0.956
/**********************/

// if GPS is not used go and grab your lat. and long. from:
//https://www.latlong.net/
double YOUR_LAT = 37.066666;
double YOUR_LONG   = 37.383331;
// Makkah's lat. and long. in decimal degree
const double MAKKAH_LAT = 21.422510;
const double MAKKAH_LONG = 39.826168;
/**********************/

// qibla tolerance margin
#define TOLERANCE_MARGINE 5 
/*********************/

// Declination
// get it from here: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml 
#define DECLINATION 5.7
/**********************/

//Dimensions for the screen
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define LOGO_WIDTH 32
#define LOGO_HEIGHT 32
/**********************/


// Buffer used for screen log with sprintf
char log_buffer[128];
/**********************/

// OLED display
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);
/**********************/



// Global variables 

// form the calibration arrayes
float B[3] = {B1, B2, B3}; // soft
float H[3][3] = {{H11, H12, H13}, {H21, H22, H23}, {H31, H32, H33}}; // hard 

bool ImuRdy;
bool MagRdy;

float mag_x;
float mag_y;
float mag_z;

int newHeading = 0;
int newHeading2 = 0;

float theta_acc_new;
float phi_acc_new;

//pitch, roll and yaw angles calculated using gyro
float theta_gyro = 0.0;
float phi_gyro = 0.0;
float yaw_gyro = 0.0; 
  
/* Create an instance of sensor data structure */
struct bmi2_sens_data sensor_data = { { 0 } };
struct bmm150_mag_data mag_data;

Madgwick orientation_filter;
int Madjwick_p; // pitch calculated by Madgwick filter
int Madjwick_r; // roll calculated by Madgwick filter
int Madjwick_y; // yaw calculated by Madgwick filter

float pitch;
float roll;

int qiblah;

uint32_t Gpstimer = millis();


// Calculate the qibla angles based on you lat. (la_L) and long (lo_L)
int qiblahFinder(float la_L, float lo_L, char LAT = 'X') {

  bool IsNoth = (la_L > 0.0 || LAT == 'N') ? true : false;
  
  char log_msg[128];

  sprintf(log_msg, "Find qiblah for location %f,%f \r\n", la_L, lo_L);
  Serial.print(log_msg);
  sprintf(log_msg, "North? %d \r\n", IsNoth);
  Serial.print(log_msg);

  float a = sin((lo_L - MAKKAH_LONG) * PI / 180);
  float b = cos(la_L * PI / 180) * tan(MAKKAH_LAT * PI / 180) - sin(la_L * PI / 180) * cos((la_L - MAKKAH_LAT) * PI / 180);

  sprintf(log_msg, "A/B: %f %f \r\n", a, b);
  Serial.print(log_msg);

  int Q = atan(a / b) * 180 / PI;
  sprintf(log_msg, "Q: %d \r\n", Q);
  Serial.print(log_msg);
  if (la_L < MAKKAH_LAT && lo_L > MAKKAH_LONG) {
    Q = IsNoth ? (360 - Q) : (180 - Q);
  }
  else if (la_L < MAKKAH_LAT && lo_L < MAKKAH_LONG) {
    Q = IsNoth ? Q : (180 + Q);
  }
  else if (la_L > MAKKAH_LAT && lo_L < MAKKAH_LONG) {
    Q = IsNoth ? (180 - Q) : (360 - Q);
  }
  else if (la_L > MAKKAH_LAT && lo_L > MAKKAH_LONG) {
    Q = IsNoth ? (180 + Q) : Q;
  }
  sprintf(log_msg, "Qiblah: %d \r\n", Q);
  Serial.print(log_msg);
  return Q;
}


void setupGPS() {
  GPS.begin(0x10);  // The I2C address to use is 0x10
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);// Request updates on antenna status, comment out to keep quiet
  delay(1000);
  GPS.println(PMTK_Q_RELEASE); // Ask for firmware version
}

void setupDisplay()
{
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;); // Don't proceed, loop forever
  }

  display.clearDisplay(); // Clear the display buffer

  display.display(); // Show the display buffer on the screen

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
}

void setup() {
  
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Started");
  Serial.println("Qiblah:");

  qiblah = qiblahFinder(YOUR_LAT, YOUR_LONG);

  Serial.println(qiblah);

  IMU.debug(Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  /* To initialize the hal function */
  Wire.begin();

  setupDisplay();

  orientation_filter.begin(20);

  pinMode(MAG_INT_PIN, INPUT);
  pinMode(IMU_INT_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(IMU_INT_PIN), _onImuDrdy, RISING);
  attachInterrupt(digitalPinToInterrupt(MAG_INT_PIN), _onAuxDrdy, RISING);

  print_qiblah_on_screen();

  delay(2000);

#if GPS_EN == 1
  setupGPS();
#endif
}

void _onImuDrdy() {
  ImuRdy = true;
}

void _onAuxDrdy() {
  MagRdy = true;
}


void Calculat_angles() {

  static float theta_acc_old = 0.0;
  static float phi_acc_old = 0.0;
  static float theta_acc = 0.0;
  static float phi_acc = 0.0;

  static float cal_gyro_x = 0.0;
  static float cal_gyro_y = 0.0;
  static float cal_gyro_z = 0.0; 

  static float last_timestamp = 0.0; 
  
  float delta_time =  (millis()-last_timestamp)/1000.;
  last_timestamp  = millis() ;

  theta_acc = atan2(sensor_data.acc.x, sensor_data.acc.z) / 2 / PI * 360;
  phi_acc = atan2(sensor_data.acc.y, sensor_data.acc.z) / 2 / PI * 360;

  cal_gyro_x = sensor_data.gyr.x;//* 360 / (2*3.141592654) ;
  cal_gyro_y = sensor_data.gyr.y;//* 360 / (2*3.141592654) ;
  cal_gyro_z = sensor_data.gyr.z;//* 360 / (2*3.141592654) ;
  
  theta_gyro = theta_gyro + cal_gyro_y * delta_time  ; // deg/sec
  phi_gyro = phi_gyro + cal_gyro_x * delta_time  ; // deg/sec
  yaw_gyro = yaw_gyro + cal_gyro_z * delta_time  ; // deg/sec

  theta_acc_new = 0.9 * theta_acc_old + 0.1 * theta_acc;
  phi_acc_new = 0.9 * phi_acc_old + 0.1 * phi_acc;

  theta_acc_old = theta_acc_new;
  phi_acc_old = phi_acc_new;

  updateMadjwick();

}

void updateMadjwick() {
  orientation_filter.update(sensor_data.gyr.x, sensor_data.gyr.y, sensor_data.gyr.z,
                            sensor_data.acc.x, sensor_data.acc.y, sensor_data.acc.z,
                            mag_data.x, mag_data.y, mag_data.z);

  Madjwick_p = static_cast<int>(orientation_filter.getPitch()) ;

  Madjwick_r = static_cast<int>(orientation_filter.getRoll());

  Madjwick_y = static_cast<int>(orientation_filter.getYaw());
}

void print_log()
{
#if MOTION_CAL_SOFT == 1
  Serial.print("Raw:");
  Serial.print(sensor_data.acc.x);
  Serial.print(',');
  Serial.print(sensor_data.acc.y);
  Serial.print(',');
  Serial.print(sensor_data.acc.z);
  Serial.print(',');

  Serial.print(sensor_data.gyr.x);
  Serial.print(',');
  Serial.print(sensor_data.gyr.y);
  Serial.print(',');
  Serial.print(sensor_data.gyr.z);
  Serial.print(',');

  Serial.print(mag_data.x * 10);
  Serial.print(',');
  Serial.print(mag_data.y * 10);
  Serial.print(',');
  Serial.println(mag_data.z * 10);
#endif

#if MAG_CALIBRATION_LOG == 1
  Serial.println("Mag:");
  Serial.print(mag_data.x);
  Serial.print(",");
  Serial.print(mag_data.y);
  Serial.print(",");
  Serial.println(mag_data.z);
#endif

#if MAG_CALIBRATED_LOG == 1
  Serial.print(mag_x);
  Serial.print(",");
  Serial.print(mag_y);
  Serial.print(",");
  Serial.println(mag_z);
#endif


#if COMPASS_LOG == 1
  Serial.print("Pich:");
  Serial.print(Madjwick_p);
  Serial.print(',');
  Serial.print("Roll:");
  Serial.print(Madjwick_r);
  Serial.print(',');
  Serial.print("Yaw:");
  Serial.print(yaw_gyro);
  Serial.print(',');
  Serial.print("Compass:");
  Serial.print(newHeading); // without tilt compensation 
  Serial.print(',');

  Serial.print("Compass2:");
  Serial.println(newHeading2); // with tilt compensation
#endif

#if GYRO_CAL == 1 
  Serial.print(sensor_data.gyr.x);
  Serial.print(',');
  Serial.print(sensor_data.gyr.y);
  Serial.print(',');
  Serial.println(sensor_data.gyr.z);
#endif

#if VISUAL_3D == 1 
  Serial.print(Madjwick_r);
  Serial.print(',');
  Serial.print(Madjwick_p);
  Serial.print(',');
  Serial.println(static_cast<int>(yaw_gyro));
#endif

}

void print_qiblah_on_screen() {

  display.clearDisplay(); // Clear the display buffer

  display.display(); // Show the display buffer on the screen

  display.drawBitmap(0, 0, Makkah_logo, LOGO_WIDTH, LOGO_HEIGHT, SSD1306_WHITE);
  display.display(); // Show the display buffer on the screen
  //  delay(100);
  display.drawBitmap(32, 0, qibla_logo, 96, 32, SSD1306_WHITE);
  display.display(); // Show the display buffer on the screen

}
void print_on_screen()
{
  display.clearDisplay(); // Clear the display buffer

  display.display(); // Show the display buffer on the screen

  display.setCursor(0, 0);
  sprintf(log_buffer, "Compass1:%d", newHeading);
  display.println(log_buffer);
  //display.display();      // Show initial text

  display.setCursor(0, 10);
  sprintf(log_buffer, "Compass2:%d", newHeading2);
  display.println(log_buffer);
  //display.display();      // Show initial text

  display.setCursor(0, 20);
  sprintf(log_buffer, "pitch:%d roll:%d", Madjwick_p, Madjwick_r);
  display.println(log_buffer);
  display.display();      // Show initial text
}



void calculate_heading()
{
  static int oldHeading = 0; // without tilt comp.
  static int oldHeading2 = 0; // with titl comp.

  float heading = atan2(mag_y, mag_x);

  if (heading < 0) {
    heading = heading + 2 * PI;
  }
  float heading_degree = heading * 180.0 / PI;
  heading_degree = heading_degree + DECLINATION ;

  newHeading = heading_degree;
  newHeading = 0.9 * oldHeading + 0.1 * newHeading;
  oldHeading = newHeading;


  pitch = Madjwick_p * PI / 180.0 ;
  roll = Madjwick_r * PI / 180.0 ;

  pitch = theta_acc_new * PI / 180.0;
  roll = phi_acc_new * PI / 180.0;

  mag_x = mag_x * cos(pitch) + mag_y * sin(roll) * sin(pitch) + mag_z * cos(roll) * sin(pitch);
  mag_y = mag_y * cos(roll) - mag_z * sin(roll);

  heading = atan2(mag_y, mag_x);

  if (heading < 0) {
    heading = heading + 2 * PI;
  }
  heading_degree = heading * 180.0 / PI;
  heading_degree = heading_degree + DECLINATION ;

  newHeading2 = heading_degree;
  newHeading2 = 0.9 * oldHeading2 + 0.1 * newHeading2;
  oldHeading2 = newHeading2;
}

void loop() {

#if GPS_EN == 1
  // read data from the GPS in the 'main loop'
  char c = GPS.read();

  if (GPS.newNMEAreceived()) {
    Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }
  // approximately every 2 seconds or so, print out the current stats
  if (millis() - Gpstimer > 2000) {
    Gpstimer = millis(); // reset the timer
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Location: ");

      Serial.print(GPS.latitudeDegrees); 
      Serial.print(", ");
      Serial.println(GPS.longitudeDegrees); 
      
      YOUR_LAT = GPS.latitudeDegrees;

      YOUR_LONG = GPS.longitudeDegrees;

      qiblah = qiblahFinder(YOUR_LAT, YOUR_LONG,GPS.lat); // update qiblah angle based on current lat. and lon.
      
    }
  }
#endif

  if (ImuRdy == true) // IMU dataready interupt flag
  {
    ImuRdy = false;
    IMU.readGyroAccel(sensor_data);
  }

  if (MagRdy == true) // mag. dataready interupt flag
  {
    MagRdy = false;
    IMU.readAuxMag(mag_data);

    //apply calibration
    float _mag_x = mag_data.x - 1 * B[0];
    float _mag_y = mag_data.y - 1 * B[1];
    float _mag_z = mag_data.z - 1 * B[2];

    mag_x = H[0][0] * _mag_x + H[0][1] * _mag_y + H[0][2] * _mag_z ;
    mag_y = H[1][0] * _mag_x + H[1][1] * _mag_y + H[1][2] * _mag_z ;
    mag_z = H[2][0] * _mag_x + H[2][1] * _mag_y + H[2][2] * _mag_z ;

    Calculat_angles();
    print_log();
    calculate_heading();
  }

  if (qiblah + TOLERANCE_MARGINE > newHeading2 && newHeading2 > qiblah - TOLERANCE_MARGINE)
  {
    print_qiblah_on_screen();
  }
  else
  {
    print_on_screen();
  }
}
