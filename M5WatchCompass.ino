
/* Analog Watch and Compass for M5Stack
 *  
 * by theBozo
 * near Zurich
 * 
 * All flights reserved 
 * 2019
 * 
 * Button A powers down/up to/from light sleep
 * Button B displays a watch
 * Button C displays a compass
 * 
 * Portions of the time code are taken from the "SimpleTime" example of ESP32
 * Portions of the AK8963 magnetometer code are taken from:
 * "https://github.com/kriswiner/ESP32/blob/master/MPU9250_MS5637/MPU9250_MS5637_AHRS.ino"
 * by: Kris Winer, December 14, 2016. 
 * Buy him beers!
 * 
 * At setup, the internet time is read from a WiFi access point set in SECRET_SSID and SECRET_PASS of M5Watch.h
 * Don't forget to include M5Watch.h and fill out SECRET_SSID and SECRET_PASS there.
 * 
 * Also at setup, a magnetometer calibration is performed when newMagCalibration = true.   
 * It is very necessary to calibrate the magnetometer before useful readings can be taken.
 * Slowly rotate the device aruond all axes for about 20 seconds during calibration.
 * Once you have read out the IMU.magbias calibration values for a device, you can hard code them into the variables indicated below to
 * speed up things by setting newMagCalibration = false.
 * Calibration is especially necessary if you have magnets at the M4Stack bottom!!
 * 
 *   
 */

#include <WiFi.h>
#include "time.h"
#include <M5Stack.h>
#include "utility/MPU9250.h"
#include "M5Watch.h"

char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;        // your network password 

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;
struct tm timeinfo;

int16_t centerX = 160;
int16_t centerY = 120;
int16_t radius1 = 110;


int16_t endX = 0;
int16_t endY = 0;
int16_t endX1 = 0;
int16_t endY1 = 0;
int16_t endX2 = 0;
int16_t endY2 = 0;
int16_t endXold = 0;
int16_t endYold = 0;
int16_t endX1old = 0;
int16_t endY1old = 0;
int16_t radius2 = 92; // seconds hand
int16_t radius3 = 80; // minutes hand
int16_t radius4 = 50; // hours hand
int16_t radius5 = 100; // compass

unsigned long before;     // 
unsigned long after;      // 
unsigned long duration;   // 

int8_t  sessionMinStart;

bool  clockmode = false;
bool  clockface = false;
bool  compassmode = false;
bool  compassface = false;
bool  menumode = false;
bool  menuface = false;
bool goToSleep = false;
boolean newMagData = false;
boolean newMagCalibration = true;                  // set to true to obtain new calibration values

uint8_t Mmode = 0x02;                               // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read, value is in MPU9250.h
float magScale[3]  = {0, 0, 0};                     // soft iron correction, not very necessary...? IMU.magbias hard iron correction however is VERY necessary
float lastHeading;
float pi = 3.1415926535897f;
float twopi = 6.2831853072f;

MPU9250 IMU;


void setup() {
  M5.begin();
  M5.Power.begin();
  Serial.begin(115200);
  Wire.begin();
  delay(4000);
  M5.Power.setWakeupButton(BUTTON_A_PIN);           // Set the wakeup button
  M5.Lcd.setBrightness(200);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(WHITE);
  M5.Lcd.setCursor(0, 0);
  M5.Lcd.setTextSize(2);
  M5.Lcd.println("Initialising...");
  M5.Lcd.println("");
  getntpTime();
  
  IMU.initMPU9250();
  // MagnetometerwhoImI();                          // test to see if magnetometer is alive
  
  IMU.initAK8963(IMU.magCalibration);
  IMU.getMres();
  if(newMagCalibration)
        { UserCalibrationAK8963(IMU.magbias, magScale);}
  else  { IMU.magbias[0] = + 87.;                   // enter value found by setting "newMagCaibration  = true" here
          IMU.magbias[1] = -586.;                   // enter value found by setting "newMagCaibration  = true" here
          IMU.magbias[2] = -548.;                   // enter value found by setting "newMagCaibration  = true" here
          magScale[0]    = 1.00;
          magScale[1]    = 1.04;
          magScale[2]    = 0.96; 
        }
  clockmode = true;
}
void loop() { 
  // kind of event & state loop...
  if (clockmode)
    {     if(!getLocalTime(&timeinfo))
            { M5.Lcd.println("Failed to obtain local time");
              return;
            }
          if(!clockface)
            { drawWatchFace();  
            }       
    drawWatchHands();
    delay(1000);
    }
  if (compassmode)
    {    if(!compassface)
            { drawCompassFace();
            }       
      drawCompassNeedle();
    delay(50);
    }
  if(M5.BtnA.wasPressed())        // Go to sleep
    {   // M5.Power.deepSleep();
      sessionMinStart =timeinfo.tm_min;
        M5.Power.lightSleep();
        sessionMinStart =timeinfo.tm_min;
     }
  if(M5.BtnB.wasPressed())        // Go to clock
  {   clockmode = true;
      clockface = false;
      menumode = false;
      compassmode = false;
      compassface = false;
  }
  if(M5.BtnC.wasPressed())        // Go to compass
  {  clockmode = false;
     menumode = false;
     compassmode = true;
     M5.Lcd.fillScreen(BLACK);
    drawCompassFace(); 
  }
  M5.update();                          // needed to get button states
}
void getntpTime() {
  
   M5.Lcd.fillScreen(BLACK);
   M5.Lcd.setCursor(0, 0);
   M5.Lcd.println("Attempting internet connection to: ");
   M5.Lcd.println("");
   M5.Lcd.println(ssid);

  WiFi.begin(ssid, pass);
  before = millis();
  while (WiFi.status() != WL_CONNECTED)  
  {   delay(100);
      M5.Lcd.print(".");
      after = millis();
      duration = after-before;
      if (duration > 20000)               // give up connecting to WiFi after 20 seconds and shut down
      { M5.Lcd.println("failed to connect to internet time, timeout..."); 
        M5.Lcd.println("Shutting down");
        delay(5000);
        M5.Power.powerOFF();
        return; }
  }
  //init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  if(!getLocalTime(&timeinfo))
    {   M5.Lcd.println("Failed to obtain local time");       
        return;
    }
   // M5.Lcd.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");

  //disconnect WiFi as it's no longer needed
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}
void drawWatchFace() {
    float angle;
    M5.Lcd.fillScreen(BLACK);
    //M5.Lcd.drawLine(centerX, centerY, centerX , centerY - radius1, WHITE );
   //M5.Lcd.drawLine(centerX, centerY, centerX - radius1, centerY, WHITE );
   for( int16_t i = 0; i <= 60; i++)
      {   angle = mintorad(i);
          if(i == 5 || i== 10 || i== 20 || i== 25 || i== 35 || i== 40 || i== 50 || i== 55 ) 
              { endX = (int16_t)(centerX + (sin(angle)*(radius1+7)));
                endY = (int16_t)(centerY - (cos(angle)*(radius1+7)));
                M5.Lcd.drawLine(centerX, centerY, endX , endY, WHITE );
              }
          if(i == 0 || i== 15 || i== 30 || i== 45 || i== 60) 
              { endX = (int16_t)(centerX + (sin(angle)*(radius1+10)));
                endY = (int16_t)(centerY - (cos(angle)*(radius1+10)));
                M5.Lcd.drawLine(centerX, centerY, endX , endY, ORANGE );
              }
          else{ endX = (int16_t)(centerX + (sin(angle)*radius1));
                endY = (int16_t)(centerY - (cos(angle)*radius1));
                M5.Lcd.drawLine(centerX, centerY, endX , endY, WHITE );
              }
      }
    M5.Lcd.fillCircle(centerX, centerY, radius1-15, BLACK );      // erase center...
    clockface = true;                                             // clockface is now drawn
}
float mintorad(int16_t  minutes) {
        float result;
        result = (twopi * minutes) / 60;
        return result;
}
float hourtorad(float  hours) {
        float result;
        result = (twopi * hours) / 12;
        return result;
}
float degtorad(int16_t  deg) {
        float result;
        result = (twopi* deg) / 360;
        return result;
}
void drawWatchHands () {
  int8_t  secs =timeinfo.tm_sec;
  int8_t  mins =timeinfo.tm_min;
  int8_t  hours=timeinfo.tm_hour;
  float angle;
  float rest;

  M5.Lcd.fillCircle(centerX, centerY, radius1-15, BLACK );
  angle = mintorad(secs);
  endX = (int16_t)(centerX + (sin(angle)*radius2));
  endY = (int16_t)(centerY - (cos(angle)*radius2));
  M5.Lcd.drawLine(centerX, centerY, endX , endY, ORANGE );

  angle = mintorad(mins);
  endX = (int16_t)(centerX + (sin(angle)*radius3));
  endY = (int16_t)(centerY - (cos(angle)*radius3));
  M5.Lcd.drawLine(centerX, centerY, endX , endY, WHITE );
  M5.Lcd.drawLine(centerX+1, centerY+1, endX+1 , endY+1, WHITE );
  M5.Lcd.drawLine(centerX-1, centerY-1, endX-1 , endY-1, WHITE );

  if(hours >= 12)
   { hours = hours-12;}
  rest = (float)(mins)/60;
  angle = hourtorad((float)(hours)+rest);
  endX = (int16_t)(centerX + (sin(angle)*radius4));
  endY = (int16_t)(centerY - (cos(angle)*radius4));
  M5.Lcd.drawLine(centerX, centerY, endX , endY, WHITE );
  M5.Lcd.drawLine(centerX+1, centerY+1, endX+1 , endY+1, WHITE );
  M5.Lcd.drawLine(centerX-1, centerY-1, endX-1 , endY-1, WHITE );       
}
void drawCompassFace() {
    float angle;
    M5.Lcd.fillScreen(BLACK);
   for( int16_t i = 0; i <= 36; i++)
      {   angle = degtorad(i*10);
         
       
          if(i == 0 || i== 9 || i== 18 || i== 27 || i== 36) 
              { endX = (int16_t)(centerX + (sin(angle)*(radius5+0)));
                endY = (int16_t)(centerY - (cos(angle)*(radius5+0)));
                M5.Lcd.drawLine(centerX, centerY, endX , endY, ORANGE );
              }
          else{ endX = (int16_t)(centerX + (sin(angle)*radius5));
                endY = (int16_t)(centerY - (cos(angle)*radius5));
                M5.Lcd.drawLine(centerX, centerY, endX , endY, WHITE );
              }     
      }
      M5.Lcd.fillCircle(centerX, centerY, radius5-15, BLACK );
      M5.Lcd.setCursor(155, 0);
      M5.Lcd.print("N");
      M5.Lcd.setCursor(275, 110);
      M5.Lcd.print("E");
      M5.Lcd.setCursor(35, 110);
      M5.Lcd.print("W");
      M5.Lcd.setCursor(155, 225);
      M5.Lcd.print("S");
      compassface = true;     
}
//void drawCompassNeedle(float angle2) 
void drawCompassNeedle() 
{ float angle2;
  angle2 = getHeading();
  M5.Lcd.drawLine(centerX, centerY, endXold , endYold, BLACK );           // erase old needle
  M5.Lcd.drawLine(centerX, centerY, endX1old , endY1old, BLACK );
   
  endX = (int16_t)(centerX + (sin(angle2)*radius3));
  endY = (int16_t)(centerY - (cos(angle2)*radius3));
  endX1 = (int16_t)(centerX - (sin(angle2)*radius4-0));
  endY1 = (int16_t)(centerY + (cos(angle2)*radius4-0));
//  M5.Lcd.fillCircle(centerX, centerY, radius5-15, BLACK );  
  M5.Lcd.drawLine(centerX, centerY, endX , endY, TFT_RED );
  M5.Lcd.drawLine(centerX, centerY, endX1 , endY1, WHITE );
 endXold = endX;
 endYold = endY;
 endX1old = endX1;
 endY1old = endY1;

 }
float getHeading()
{    float heading;

    newMagData = (IMU.readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01);
    if(!newMagData)                                     // don't have new data, skip
      { //Serial.println("no new data!");
          delay(10);
          return lastHeading;
      }
    else                                                // have new data
     {  IMU.readMagData(IMU.magCount);                  // Read the x/y/z adc values
    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    IMU.mx = (float)IMU.magCount[0] * IMU.mRes * IMU.magCalibration[0] - IMU.magbias[0];
    IMU.my = (float)IMU.magCount[1] * IMU.mRes * IMU.magCalibration[1] - IMU.magbias[1];
    IMU.mz = (float)IMU.magCount[2] * IMU.mRes * IMU.magCalibration[2] - IMU.magbias[2];

      IMU.mx *= magScale[0];
      IMU.my *= magScale[1];
      IMU.mz *= magScale[2]; 
             

    IMU.tempCount = IMU.readTempData();             // Read the adc values
    // Temperature in degrees Centigrade whatever that might be useful for...
    IMU.temperature = ((float) IMU.tempCount) / 333.87 + 21.0;

    heading  = (atan2(IMU.my,IMU.mx));
    if(heading < 0) heading   += (6.2831853072);      // Ensure heading stays between 0 and 2pi
    
    // also add/subtract your local magnetic declination here:  - 2,5 degrees for Switzerland
    heading = heading - 0.043633;
    //  heading = 360*(heading/ 6.2831853072);        // Just in case you want degrees...
      lastHeading = heading;
      return heading; 
  }
}

void UserCalibrationAK8963(float * dest1, float * dest2) 
{
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};

 M5.Lcd.println("Magnetometer calibration: Slowly turn the device around all axes until done after about 20 seconds...");
  delay(2000); 
    // shoot for ~fifteen seconds of mag data
    if(Mmode == 0x02) sample_count = 128;  // at 8 Hz ODR, new mag data is available every 125 ms
    if(Mmode == 0x06) sample_count = 1500;  // at 100 Hz ODR, new mag data is available every 10 ms
   for(ii = 0; ii < sample_count; ii++) {
    IMU.readMagData(mag_temp);  // Read the mag data   
    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
   if(Mmode == 0x02) delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
    if(Mmode == 0x06) delay(12);  // at 100 Hz ODR, new mag data is available every 10 ms
 
    }
    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

  
    dest1[0] = (float) mag_bias[0]*IMU.mRes*IMU.magCalibration[0];  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1]*IMU.mRes*IMU.magCalibration[1];   
    dest1[2] = (float) mag_bias[2]*IMU.mRes*IMU.magCalibration[2];   
    
    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);

    //fill printed values into hardcoded IMU.magbias values above to avoid new calibration (when setting newMagCalibration = false)
               Serial.println("IMU.magbias");
               Serial.println(dest1[0]); Serial.println(dest1[1]); Serial.println(dest1[2]); 
               Serial.println("magScale");
               Serial.println(dest2[0]); Serial.println(dest2[1]); Serial.println(dest2[2]);  
     
   M5.Lcd.println("Mag calibration is done!");
}

void MagnetometerwhoImI()                                   // Read the WHO_AM_I register of the magnetometer, this is just a communication test
{ byte d = IMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);  // Read WHO_AM_I register for AK8963
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setCursor(0, 0);
   M5.Lcd.println("AK8963: ");  M5.Lcd.print("I AM ");  M5.Lcd.println(d, HEX);  M5.Lcd.print("I should be ");  M5.Lcd.println(0x48, HEX); M5.Lcd.println("...therefore I am!");
  delay(1000); 
} 
