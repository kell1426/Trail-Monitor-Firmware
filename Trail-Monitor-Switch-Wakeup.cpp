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
#define DEBUG 0
SYSTEM_MODE(SEMI_AUTOMATIC);
int id = 100;
File myFile;
AssetTracker t = AssetTracker();
long lastGPSpoint = 0;
uint32_t initialTime = 0;
uint32_t offset = 0;
enum State { WAKE_STATE, ACQUIRE_STATE, SEND_STATE, POWER_DOWN_STATE};
State state = WAKE_STATE;
int pin;

void sendStateChange();

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

  t.begin();	//Set up Asset Tracker bits

  pinMode(D1, INPUT_PULLDOWN);

  attachInterrupt(D1, sendStateChange, FALLING);
  delay(5000);
}

void loop()
{
  switch(state)
  {
    case WAKE_STATE:
      myFile.open("TrailData.txt", O_RDWR | O_CREAT);
      myFile.close();
      t.gpsOn();
      if(DEBUG == 1)
      {
        Serial.println("In WAKE_STATE");
        state = ACQUIRE_STATE;
        delay(500);
        Serial.println("Moving to ACQUIRE_STATE");
        delay(500);
        break;
      }
      Cellular.on();
      Cellular.connect();
      while(!Cellular.ready());
      Particle.connect();
      delay(5000);
      initialTime = Time.local();
      offset = millis();
      Particle.disconnect();
      Cellular.disconnect();
      Cellular.off();
      state = ACQUIRE_STATE;
      break;
    case ACQUIRE_STATE:
      //Serial.println("In ACQUIRE_STATE");
      //delay(1000);
      t.updateGPS();
      if(DEBUG == 1)
      {
        Serial.println("In ACQUIRE_STATE");
        delay(500);
      }
      if ((millis()-lastGPSpoint) > (1000))
      {
        lastGPSpoint = millis();
        if(t.gpsFix())
        {
          float lat = t.readLatDeg();
          float lon = t.readLonDeg();
          uint32_t epoch = initialTime + ((millis() - offset) / 1000);
          uint32_t ms = millis() % 1000;
          String stamp = String::format("%lu%lu", epoch, ms);
          int accel = t.readZ();
          String data = String::format("{ \"La\": %f, \"Lo\": %f, \"T\": \"%s\", \"H\": %d, \"id\": %d }", lat, lon, stamp.c_str(), accel, id);
          myFile.open("TrailData.txt", O_RDWR | O_AT_END);
          myFile.println(data);
          myFile.close();
        }
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
      while((n = myFile.fgets(arr, sizeof(arr))) > 0)
      {
          String str(arr);
          Particle.publish("Heat", str, PRIVATE);
          delay(1500);
      }
      myFile.remove();
      state = POWER_DOWN_STATE;
      break;
    case POWER_DOWN_STATE:
      if(DEBUG == 1)
      {
        Serial.println("In POWER_DOWN_STATE");
      }
      delay(1000);
      System.sleep(D1, RISING);
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
    Serial.println("Moving to SEND_STATE");
}
