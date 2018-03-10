// This #include statement was automatically added by the Particle IDE.
#include <AssetTracker.h>

// This #include statement was automatically added by the Particle IDE.
#include <SdFat.h>

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
#define WAKE_DELAY 12000  //10 ms * 12000 = 2 minute delay
#define MOVEMENT_DELAY 60 //500 ms * 600 = 5 minute delay  500 * 60 = 30 second delay
SYSTEM_MODE(SEMI_AUTOMATIC);

File myFile;
AssetTracker t = AssetTracker();
long lastGPSpoint = 0;
uint32_t initialTime = 0;
int count = 0;
enum State { WAKE_STATE, ACQUIRE_STATE, SEND_STATE, POWER_DOWN_STATE};
State state = WAKE_STATE;
int movementCounter;
int wakeCounter;

void setup()
{
  Serial.begin(9600);
  // Wait for USB Serial
  while (!Serial)
  {
  SysCall::yield();
  }

  if (!sd.begin(chipSelect, SPI_HALF_SPEED))
  {
    sd.initErrorHalt();
  }

  myFile.open("TrailData.txt", O_RDWR | O_CREAT);
  myFile.close();

  t.begin();	//Set up Asset Tracker bits

  movementCounter = 0;
  wakeCounter = 0;

  Cellular.on();
  Cellular.connect();
  while(!Cellular.ready());
  Particle.connect();
  while(!Time.isValid())
  {
      Particle.syncTime();
  }
  initialTime = Time.local();
  Particle.disconnect();
  Cellular.disconnect();
  Cellular.off();
}

void loop()
{
  switch(state)
  {
    case WAKE_STATE:
      t.gpsOn();
      while(1)
      {
        t.updateGPS();
        if(t.gpsFix())  //If gps fix found. Go to ACQUIRE_STATE.
        {
          state = ACQUIRE_STATE;
          break;
        }
        else
        {
          delay(10);
          wakeCounter++;
        }
        if(wakeCounter >= WAKE_DELAY) //Wait for gps fix for 2 minutes. If not found go to POWER_DOWN_STATE.
        {
          state = POWER_DOWN_STATE;
          break;
        }
      }
      break;
    case ACQUIRE_STATE:
      t.updateGPS();
      if(movementCounter > MOVEMENT_DELAY)
      {
        state = SEND_STATE;
      }
      else if ((millis()-lastGPSpoint) > (500)) //500ms SECOND DELAY
      {
        // Remember when we published
    		lastGPSpoint = millis();

        int mag = t.readXYZmagnitude();
        //int speed = t.getSpeed();
        if(mag > 7000 && mag < 8500 /*&& speed < 5*/)
        {
          movementCounter++;
          // String info = String::format("MovementCounter is: %d and XYZmag is: %d", movementCounter, mag);
          // Serial.println(info);

        }
        else
        {
          movementCounter = 0;
          // String info = String::format("MovementCounter is: %d and XYZmag is: %d", movementCounter, mag);
          // Serial.println(info);
          if(t.gpsFix())
          {
            float lat = t.readLatDeg();
            float lon = t.readLonDeg();
            uint32_t epoch = initialTime + (millis() / 1000);
            //int ms = millis() % 1000;
            //String stamp = String::format("%lu%d", epoch, ms);
            String stamp = String::format("%lu", epoch);
            int accel = t.readZ();
            String data = String::format("{ \"Lat\": \"%f\", \"Lon\": \"%f\", \"Time\": \"%s\", \"Harsh\": \"%d\" }", lat, lon, stamp.c_str(), accel);
            myFile.open("TrailData.txt", O_RDWR | O_AT_END);
            myFile.println(data);
            myFile.close();
          }
        }
      }
      break;
    case SEND_STATE:
      Cellular.on();
      Cellular.connect();
      while(!Cellular.ready());
      Particle.connect();
      myFile.open("TrailData.txt", O_RDWR);
      char arr[256];
      size_t n;
      while((n = myFile.fgets(arr, sizeof(arr))) > 0)
      {
          String str(arr);
          Serial.println(str);
          //Particle.publish("Heat", str, PRIVATE);
      }
      myFile.close();
      //remove("TrailData.txt");
      state = POWER_DOWN_STATE;
      break;
    case POWER_DOWN_STATE:
      System.sleep(SLEEP_MODE_SOFTPOWEROFF, 1200);
      break;
  }
}
