// This #include statement was automatically added by the Particle IDE.
#include <AssetTracker.h>

// This #include statement was automatically added by the Particle IDE.
#include <SdFat.h>

#include <math.h>
#include <time.h>
// Pick an SPI configuration.
// See SPI configuration section below (comments are for photon).
#define SPI_CONFIGURATION 0
//------------------------------------------------------------------------------
// Setup SPI configuration.
#if SPI_CONFIGURATION == 0
SdFatSoftSpi<D3, D2, D4> sd;
const uint8_t chipSelect = D5;
#endif  // SPI_CONFIGURATION
//------------------------------------------------------------------------------
#define DEBUG 0
int WKPIN = D1;
SYSTEM_MODE(SEMI_AUTOMATIC);
File myFile;
File exFile;
File IMUfile;
File SpeedFile;
AssetTracker t = AssetTracker();
long lastGPSpoint = 0;
long lastACCpoint = 0;
int numAcc = 0;
int numACCpoints = 0;
float curSumSQ = 0;
float sumsq = 0;
float avgSumSQ = 0;
int mag;
int skip = 0;
int alive = 0;
//uint32_t initialTime = 0;
//uint32_t offset = 0;
enum State { WAKE_STATE, ACQUIRE_STATE, SEND_STATE, POWER_DOWN_STATE};
State state = WAKE_STATE;
int pin;

void sendStateChange();
String ms_last3digit_grabber(int ms);
uint32_t gpsTIme();

void setup()
{
  if(DEBUG == 1)
  {
      Serial.begin(9600);
      // Wait for USB Serial
      while (!Serial)
      {
      SysCall::yield();
      }
  }

  if (!sd.begin(chipSelect, SPI_HALF_SPEED))
  {
    sd.initErrorHalt();
  }

  myFile.open("TrailData.txt", O_RDWR | O_CREAT);
  myFile.close();
    if(DEBUG == 2)
    {
        IMUfile.open("IMUData.txt", O_RDWR | O_CREAT);
        IMUfile.close();
        SpeedFile.open("Speeddata.txt", O_RDWR | O_CREAT);
        SpeedFile.close();
    }

  t.begin();	//Set up Asset Tracker bits

  pinMode(WKPIN, INPUT_PULLDOWN);

  attachInterrupt(WKPIN, sendStateChange, FALLING);
  delay(5000);
}

void loop()
{
  switch(state)
  {
    case WAKE_STATE:
      myFile.open("TrailData.txt", O_RDWR | O_CREAT);
      myFile.close();
      if(DEBUG == 2)
      {
        IMUfile.open("IMUData.txt", O_RDWR | O_CREAT);
        IMUfile.close();
        SpeedFile.open("Speeddata.txt", O_RDWR | O_CREAT);
        SpeedFile.close();
      }
      t.gpsOn();
      //t.antennaExternal();
      if(DEBUG == 1)
      {
        Serial.println("In WAKE_STATE");
        state = ACQUIRE_STATE;
        delay(500);
        Serial.println("Moving to ACQUIRE_STATE");
        delay(500);
        break;
      }
      // Cellular.on();
      // Cellular.connect();
      // while(!Cellular.ready());
      // Particle.connect();
      // Particle.syncTime();
      // delay(10000);
      // initialTime = Time.local();
      // offset = millis();
      // Particle.disconnect();
      // Cellular.disconnect();
      // Cellular.off();
      state = ACQUIRE_STATE;
      break;
    case ACQUIRE_STATE:
      t.updateGPS();
      if(DEBUG == 1)
      {
        Serial.println("In ACQUIRE_STATE");
        delay(500);
      }
      if ((millis()-lastGPSpoint) > (1000))
      {
        lastGPSpoint = millis();
        numAcc = numACCpoints;
        numACCpoints = 0;
        curSumSQ = sumsq;
        sumsq = 0;
        avgSumSQ = 0;
        if (numAcc>0) {
          avgSumSQ = curSumSQ/numAcc;
        }

        if(t.gpsFix())
        {
          int x;
          int y;
          int z;
          float speed;
          speed = t.getSpeed();
          if(speed < 2.0) //Vehicle not in motion
          {
            break;
          }
          if(DEBUG == 2)
          {
            x = t.readX();
            y = t.readY();
            z = t.readZ();
            mag = t.readXYZmagnitude();
          }
          float lat = t.readLatDeg();
          float lon = t.readLonDeg();
          //uint32_t epoch = initialTime + ((millis() - offset) / 1000);
          uint32_t epoch = gpsTime();
          //uint32_t ms = millis() % 1000;
          uint32_t ms = t.getMilliseconds();
          String ms_string = ms_last3digit_grabber(ms);
          String stamp = String::format("%lu%s", epoch, ms_string.c_str());
          int accel = sqrt(avgSumSQ);
          String data = String::format("{ \"La\": %f, \"Lo\": %f, \"T\": \"%s\", \"H\": %d }", lat, lon, stamp.c_str(), accel);
          myFile.open("TrailData.txt", O_RDWR | O_AT_END);
          myFile.println(data);
          myFile.close();
          if(DEBUG == 2)
          {
            String IMUstring = String::format("X is: %d, Y is: %d, Z is: %d, Mag is: %d", x, y, z, mag);
            String speedString = String::format("Speed is: %f at time: %s", speed, stamp.c_str());
            IMUfile.open("IMUdata.txt", O_RDWR | O_AT_END);
            IMUfile.println(IMUstring);
            IMUfile.close();
            SpeedFile.open("Speeddata.txt", O_RDWR | O_AT_END);
            SpeedFile.println(speedString);
            SpeedFile.close();
          }
        }
      }
      if ((millis()-lastACCpoint) > (10)) // Accelerometer every 10ms
      {
        lastACCpoint = millis();
        mag = t.readXYZmagnitude();
        numACCpoints++;
        sumsq += mag*mag;
      }
      break;
    case SEND_STATE:
      t.gpsOff();
      if(DEBUG == 1)
      {
        Serial.println("In SEND_STATE");
        state = POWER_DOWN_STATE;
        delay(500);
        Serial.println("Moving to POWER_DOWN_STATE");
        delay(500);
        break;
      }
      Cellular.on();
      Cellular.connect();
      while(!Cellular.ready());
      Particle.connect();
      myFile.open("TrailData.txt", O_RDWR);
      char arr[256];
      size_t n;
      skip = 0;
      alive = 0;

      // Read and publish TrailData.txt data
      while((n = myFile.fgets(arr, sizeof(arr))) > 0)
      {
        if (digitalRead(WKPIN) == HIGH) { //Check if Vehicle is turned back on
          delay(1000); // Wait and make sure
          if (digitalRead(WKPIN) == HIGH) {
            exFile.open("TrailDataExtras.txt", O_RDWR | O_AT_END | O_CREAT);
            String str(arr);
            exFile.println(str);
            while ((n = myFile.fgets(arr, sizeof(arr))) > 0) { // Move lines from TrailData to Extras
              String str(arr);
              exFile.println(str);
            }
            skip = 1;
            alive = 1;
            break;
          }
        }
          String str(arr);
          Particle.publish("Heat", str, PRIVATE);
          delay(3000);
      }
      myFile.remove();

      // Read and publish TrailDataExtras.txt data
      if (skip == 0) {
        exFile.open("TrailDataExtras.txt", O_RDWR | O_CREAT);
        while((n = exFile.fgets(arr, sizeof(arr))) > 0)
        {
          if (digitalRead(WKPIN) == HIGH) { //Check if Vehicle is turned back on
            delay(1000); // Wait and make sure
            if (digitalRead(WKPIN) == HIGH) {
              myFile.open("TrailData.txt", O_RDWR | O_AT_END | O_CREAT);
              String str(arr);
              myFile.println(str);
              while ((n = exFile.fgets(arr, sizeof(arr))) > 0) { // Move lines from Extras to TrailData
                String str(arr);
                myFile.println(str);
              }
              alive = 1;
              break;
            }
          }
            String str(arr);
            Particle.publish("Heat", str, PRIVATE);
            delay(3000);
        }
        exFile.remove();
      }
      if (alive) {
        state = ACQUIRE_STATE;
      }
      else {
        state = POWER_DOWN_STATE;
      }
      break;
    case POWER_DOWN_STATE:
      if(DEBUG == 1)
      {
        Serial.println("In POWER_DOWN_STATE");
      }
      delay(1000);
      System.sleep(WKPIN, RISING);
      delay(1000);
      state = WAKE_STATE;
      if(DEBUG == 1)
      {
        Serial.println("Moving to WAKE_STATE");
      }
      delay(500);
      break;
  }
}

void sendStateChange()
{
    state = SEND_STATE;
    if(DEBUG == 1)
    {
      Serial.println("Moving to SEND_STATE");
    }
}

String ms_last3digit_grabber(int ms)
{
  String digits;
  if(ms >= 0 && ms <= 9)
  {
    digits = String::format("00%d", ms);
  }
  else if(ms >= 10 && ms <= 99)
  {
    digits = String::format("0%d", ms);
  }
  else
  {
    digits = String::format("%d", ms);
  }
  return digits;
}

uint32_t gpsTime()
{
      struct tm T;
      time_t t_of_day;

      T.tm_year = t.getYear()-1900;
      T.tm_mon = t.getMonth() - 1;     // Month, 0 - jan
      T.tm_mday = t.getDay();          // Day of the month
      T.tm_hour = t.getHour();
      T.tm_min = t.getMinute();
      T.tm_sec = t.getSeconds();
      T.tm_isdst = 1;        // Is DST on? 1 = yes, 0 = no, -1 = unknown
      t_of_day = mktime(&T);

      //printf("seconds since the Epoch: %ld\n", (long) t_of_day);
      uint32_t timestamp = (uint32_t)t_of_day;
      return timestamp;
}
