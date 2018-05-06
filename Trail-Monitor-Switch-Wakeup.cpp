// This #include statement was automatically added by the Particle IDE.
#include <AssetTracker.h>

// This #include statement was automatically added by the Particle IDE.
#include <SdFat.h>

#include <math.h>
#include <time.h>


// Pick an SPI configuration.
#define SPI_CONFIGURATION 0
//------------------------------------------------------------------------------
// Setup SPI configuration.
#if SPI_CONFIGURATION == 0
SdFatSoftSpi<D3, D2, D4> sd;
const uint8_t chipSelect = D5;
#endif  // SPI_CONFIGURATION
//------------------------------------------------------------------------------
#define DEBUG 0 //Different debug values used throughout testing.
// Debug 0 is the normal operation mode.
// Debug 1 is for serail connections and print outs to a terminal.
// Debug 2 prints out Accelerometer information to the "IMUData.txt" file as well as speed data to the "Speeddata.txt" file
// Debug 3 used in earlier versions of code. No longer does anything
// Debug 4 used in earlier versions of code. No longer does anything
// Debug 5 prints out the acceleromter data used in the harshess algorithm to the "IMUData.txt" file as well as speed data to the "Speeddata.txt" file
#define range 4 //range attribute on particle for filtering not setup
#define SpeedFactor 1 //Speed factor to be implemeted later. Would dynamically adjust
//based on the speed of the vehicle to account for different Accelerometer readings at different speeds.
#define SuspensionFactor 2 //SuspensionFactor that is multiplied to the final harshness value.
//Currently set to 2, calibrated to a car on street roads. Will need to be lowered for off road vehicles.
int WKPIN = D1;
SYSTEM_MODE(SEMI_AUTOMATIC); //Does not automatically connect to particle cloud or cellular networks.
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
float lastLat;
float lastLon;
enum State { WAKE_STATE, ACQUIRE_STATE, SEND_STATE, POWER_DOWN_STATE};
State state = WAKE_STATE;
int pin;
int cellCount;
bool cellConnectFail;

String data1;
String data2;
String data3;
int count = 0;

//initialization for Filtering
float oldAccel = 0;

float filterCoefficient = 0.9; //lowpass
float lowpassValue = 0;
float highpassValue = 0;

int noisefig = 1;
int filteredMag = 0;
//end initialization for filtering
void sendStateChange();
String ms_last3digit_grabber(int ms);
uint32_t gpsTIme();
float convert(float raw);
float filter(float magnitude);

// Function setup()
// Sets up serial connection if on the correct debug
// Initializes the asset tracker object "t", sets the pulldown resistor on the WKPIN
// Sets up the interrupt to change from acquire state to send state
void setup()
{
  if(DEBUG == 1 || DEBUG == 3)
  {
      Serial.begin(9600);
      while (!Serial)
      {
      SysCall::yield();
      }
  }

  t.begin();

  pinMode(WKPIN, INPUT_PULLDOWN);

  attachInterrupt(WKPIN, sendStateChange, FALLING);
  delay(5000);
}

void loop()
{
  switch(state)
  {
    //Case WAKE_STATE
    // Initializes the SD card, creates the files needed, turns on the GPS with the
    // external antenna. If debug is 1, prints out information serially. Sets the state
    // to the ACQUIRE_STATE state and breaks out.
    case WAKE_STATE:
      if (!sd.begin(chipSelect, SPI_HALF_SPEED))
      {
        sd.initErrorHalt();
      }
      myFile.open("TrailData.txt", O_RDWR | O_CREAT);
      myFile.close();
      if(DEBUG == 2 || DEBUG == 3 || DEBUG == 5)
      {
          IMUfile.open("IMUData.txt", O_RDWR | O_CREAT);
          IMUfile.close();
          SpeedFile.open("Speeddata.txt", O_RDWR | O_CREAT);
          SpeedFile.close();
      }
      t.gpsOn();
      t.antennaExternal();
      if(DEBUG == 1)
      {
        Serial.println("In WAKE_STATE");
        state = ACQUIRE_STATE;
        delay(500);
        Serial.println("Moving to ACQUIRE_STATE");
        delay(500);
        break;
      }
      state = ACQUIRE_STATE;
      break;
    // Case ACQUIRE_STATE
    // Updates the gps info. If debug is 1, prints out info serially.
    // If the time is past the accelerometer delay (10 ms), grab an acceleromter point and filter it.
    // If debug is 5, print this raw acceleromter point to the IMU file. If the magnitude is greater
    // thn 0, increment the filtered mag count and add to the summed squated magnitude variable.
    // If the time is past the GPS delay (1 sec), average the total summed squared magnitude.
    // If the gps has a fix, grab all the necessary info for the gps point.
    // Two checks are in place before storing this gps point. The first is if the speed is less than
    // 1 knot, and if so break out and discard the data. The second checks if the gps point is the same
    // as the last gps point, if so break out and discard the data.
    // To print the data to the file, the data is first formatted. The first two points are written in the correct
    // json format, and the third is written with a new line appended to it.
    case ACQUIRE_STATE:
      t.updateGPS();
      if(DEBUG == 1)
      {
        Serial.println("In ACQUIRE_STATE");
        delay(500);
      }
        if ((millis()-lastACCpoint) > (10)) // Accelerometer every 10ms
        {
          lastACCpoint = millis();
          mag = t.readXYZmagnitude();
          if(DEBUG == 5)
          {
            String IMUstring = String::format("Raw Mag is: %d", mag);
            IMUfile.open("IMUData.txt", O_RDWR | O_AT_END);
            IMUfile.println(IMUstring);
            IMUfile.close();
          }
          filteredMag = round(convert(filter(mag))) * SpeedFactor * SuspensionFactor; //filter magnitude, convert to m/s^2
          if(filteredMag > 0)
          {
            numACCpoints++;
          }
          sumsq = sumsq + (filteredMag*filteredMag);
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
          if(speed < 1.0) //Vehicle not in motion
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
          if((lastLat == lat) && (lastLon == lon)) //Vehicle has not moved since last GPS point
          {
              break;
          }
          uint32_t epoch = gpsTime();
          uint32_t ms = millis() % 1000;
          String ms_string = ms_last3digit_grabber(ms);
          String stamp = String::format("%lu%s", epoch, ms_string.c_str());
          int accel = sqrt(avgSumSQ);
          if(count == 0)
          {
            data1 = String::format("{ \"La1\": %f, \"Lo1\": %f, \"T1\": \"%s\", \"H1\": %d, ", lat, lon, stamp.c_str(), accel);
            count++;
          }
          else if(count == 1)
          {
            data2 = String::format("\"La2\": %f, \"Lo2\": %f, \"T2\": \"%s\", \"H2\": %d, ", lat, lon, stamp.c_str(), accel);
            count++;
          }
          else
          {
            data3 = String::format("\"La3\": %f, \"Lo3\": %f, \"T3\": \"%s\", \"H3\": %d }", lat, lon, stamp.c_str(), accel);
            count = 0;
            String allData = String::format("%s%s%s", data1.c_str(), data2.c_str(), data3.c_str());
            myFile.open("TrailData.txt", O_RDWR | O_AT_END);
            myFile.println(allData);
            myFile.close();
          }
          if(DEBUG == 2 || DEBUG == 5)
          {
            String IMUstring = String::format("Timestamp is %s and harshness value is: %d", stamp.c_str(), accel);
            String speedString = String::format("Speed is: %f at time: %s", speed, stamp.c_str());
            IMUfile.open("IMUData.txt", O_RDWR | O_AT_END);
            IMUfile.println(IMUstring);
            IMUfile.close();
            SpeedFile.open("Speeddata.txt", O_RDWR | O_AT_END);
            SpeedFile.println(speedString);
            SpeedFile.close();
          }
          lastLat = lat;
          lastLon = lon;
        }
      }
      break;
    // Case SEND_STATE
    // Device enters this state when the interrupt attached to the wake pin fires
    // Truns off the gps unit, not needed to send out data
    // If debug is 1, print out info serially
    // Turn on the cellular modem and connect to a cellular network
    // While loop at 269 checks if the time to connect to the cellular network has exceeded 5 minutes.
    // If so set cellConnectFail to true and break out of loop
    // If cellConnectFailis true, set the state to POWER_DOWN_STATE, turn of cellular modem and break out of SEND_STATE
    // Connect to particle cloud and open the TrailData.txt file
    // Next two blocks of code read the data stored in the file and send the points to the particle webook
    // If at any point the vehicle turns on while sending the data, the remaining data will be written to
    // TrailDataExtras.txt and enter the ACQUIRE_STATE. This cycle will then repeat.
    // When done sending data, disconnect from Particle and cellular networks. Then shut off the cellular modem.
    // Finally, set the state to POWER_DOWN_STATE and break out.
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
      cellCount = 0;
      cellConnectFail = false;
      while(!Cellular.ready());
      {
        delay(10);
        cellCount++;
        if(cellCount > 10*100*60*5) //5 minutes
        {
          cellConnectFail = true;
          break;
        }
      }
      if(cellConnectFail == true)
      {
        state = POWER_DOWN_STATE;
        Cellular.disconnect();
        Cellular.off();
        break;
      }
      Particle.connect();
      myFile.open("TrailData.txt", O_RDWR | O_CREAT);
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
            exFile.print(str);
            while ((n = myFile.fgets(arr, sizeof(arr))) > 0) { // Move lines from TrailData to Extras
              String str(arr);
              exFile.print(str);
            }
            skip = 1;
            alive = 1;
            exFile.close();
            break;
          }
        }
          String str(arr);
          Particle.publish("Heat", str, PRIVATE);
          delay(1500);
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
              myFile.print(str);
              while ((n = exFile.fgets(arr, sizeof(arr))) > 0) { // Move lines from Extras to TrailData
                String str(arr);
                myFile.print(str);
              }
              myFile.close();
              alive = 1;
              break;
            }
          }
            String str(arr);
            Particle.publish("Heat", str, PRIVATE);
            delay(1500);
        }
        exFile.remove();
      }
      if (alive) {
        Particle.disconnect();
        Cellular.disconnect();
        Cellular.off();
        state = ACQUIRE_STATE;
      }
      else {
        Particle.disconnect();
        Cellular.disconnect();
        Cellular.off();
        state = POWER_DOWN_STATE;
      }
      break;
    // Case POWER_DOWN_STATE
    // If debug is 1, print out info serially.
    // Else put the device to sleep and wak on a rising edge on the WKPIN
    // When the device wakes up, it will resume the code from the point it went to sleep
    // Set the state to WAKE_STATE and break out
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

// Function sendStateChange()
// Called when the interrupt fires on a falling edge of the WKPIN
// Sets the state to SEND_STATE
void sendStateChange()
{
    state = SEND_STATE;
    if(DEBUG == 1)
    {
      Serial.println("Moving to SEND_STATE");
    }
}

// Function ms_last3digit_grabber
// Takes in the milliseconds of the device modded by 1000
// Formats this number into a 3 digit String
// Example:
//   7 -> 007
//   70 -> 070
//   700 -> 700
// Returns this new string
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

// Function gpsTime()
// Returns the current epoch time
uint32_t gpsTime()
{
      struct tm T;
      time_t t_of_day;

      T.tm_year = t.getYear() + 100;
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

//Filtering for roughness
int filter(int magnitude) {

	float t = (1 - filterCoefficient) / filterCoefficient;


	// Triple Exponential Moving Average low pass filter: a little lag
	lowpassValue = lowpassValue * filterCoefficient + magnitude * (1 - filterCoefficient);
	lowpassValue = lowpassValue * filterCoefficient + magnitude * (1 - filterCoefficient);
	lowpassValue = lowpassValue * filterCoefficient + magnitude * (1 - filterCoefficient);

	// Zero lag Exponential Moving Average low pass filter: less lag but a little value change
	//lowpassValue = (1 - filterCoefficient)*magnitude + filterCoefficient*(magnitude + (magnitude - mg[t]));

	// high pass filter
	highpassValue = magnitude - lowpassValue;


	oldAccel = highpassValue;

	if (noisefig == 1) { //the first value is a noise
		noisefig = 0;
		highpassValue = 0;
	}

	return abs(highpassValue);
}

//Funtion Convert
//Takes in raw IMU data as an int
//Convert from raw IMU to m/s^2
float convert(int raw) {

	float a = 0;

	a = ((float)raw / (float)32767) * (float)range * 9.8;

	return a;
}
