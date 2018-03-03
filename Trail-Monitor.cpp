#include "SdFat.h"
#include "AssetTracker.h"

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
// TEST


File myFile;
AssetTracker t = AssetTracker();
int transmittingData = 1;
long lastPublish = 0;

void setup()
{
  Serial.begin(9600);
  // Wait for USB Serial
  while (!Serial)
  {
	SysCall::yield();
  }

  t.begin();	//Set up Asset Tracker bits
  t.gpsOn();	//Initialize GPS Module
  Particle.syncTime();

  // Initialize SdFat or print a detailed error message and halt
  // Use half speed like the native library.
  // Change to SPI_FULL_SPEED for more performance.
  if (!sd.begin(chipSelect, SPI_HALF_SPEED))
  {
    sd.initErrorHalt();
  }

  if (!myFile.open("TrailData.txt", O_RDWR | O_CREAT | O_AT_END))
  {
    sd.errorHalt("opening TrailData.txt for write failed");
  }
  myFile.close();

  Cellular.disconnect();
}

void loop()
{
	t.updateGPS();
  int speedCounter;
  // if the current time - the last time we published is greater than your set delay...
	if (millis()-lastPublish > (5000)) //5 SECOND DELAY
	{
    // Remember when we published
		lastPublish = millis();

		if(t.gpsFix() /* && Speed (Vehicle Moving)*/)
		{
			float lat = t.readLatDeg();
			float lon = t.readLonDeg();
			uint32_t epoch = Time.local();
			int accel = t.readZ();
			int harsh; //Look into which direction to use
			if(accel < 12000)
			{
				harsh = 1;
			}
			else if(accel >= 12000 && accel < 14000)
			{
				harsh = 2;
			}
			else if(accel >= 14000 && accel < 16000)
			{
				harsh = 3;
			}
			else if(accel >= 16000 && accel < 18000)
			{
				harsh = 4;
			}
			else if(accel >= 18000 && accel < 20000)
			{
				harsh = 5;
			}
			else if(accel >= 20000 && accel < 22000)
			{
				harsh = 6;
			}
			else if(accel >= 22000 && accel < 24000)
			{
				harsh = 7;
			}
			else if(accel >= 24000 && accel < 26000)
			{
				harsh = 8;
			}
			else //accel > 26000
			{
				harsh = 9;
			}
		  //String lats = String::format("%f", lat);
		  //Particle.publish("GPS", lats, PRIVATE);
		  //String lons = String::format("%f", lon);
	   //   Particle.publish("Lon", lons, PRIVATE);
	   //   String epochs = String::format("%lu", epoch);
	   //   Particle.publish("Time", epochs, PRIVATE);
      //String test_data = String::format("Lat: %f, Lon: %f, Time: %lu, Harsh: %d", lat, lon, epoch, harsh);
      //Particle.publish("Test Data", test_data, PRIVATE);
      String data = String::format("{ \"Lat\": \"%f\", \"Lon\": \"%f\", \"Time\": \"%lu\", \"Harsh\": \"%d\"}", lat, lon, epoch, harsh);
		  if (!myFile.open("TrailData.txt", O_RDWR | O_CREAT | O_AT_END))
			{
				sd.errorHalt("opening TrailData.txt for write failed");
			}
          //myFile.println(test_data);
        myFile.println(data);
	      myFile.close();

	   }
   }
 //Version 1. This one can be used if we want to attempt to send data while the vehicle is moving. Code not complete
 if(Cellular.ready())
   {
		//Open File in read/write mode
		if (!myFile.open("TrailData.txt", O_RDWR))
		{
			sd.errorHalt("opening TrailData.txt for read failed");
		}

		//Read data from SD card and delete it after the read
    int endOfFile = 0;
    /*String*/int data;
    while(Cellular.ready())
    {
      if((data = myFile.read()) >= 0)
      {
        endOfFile = 1;
        break;
      }
      Particle.publish("Heat", data, PRIVATE);
    }
    File myFile2;
    if (!myFile2.open("temp.txt", O_RDWR | O_CREAT | O_AT_END))
    {
      sd.errorHalt("opening temp.txt for write failed");
    }
    if(endOfFile == 0)
    {
      while((data = myFile.read()) >= 0)
      {
        myFile2.println(data)
      }
    }
    myFile.close();
    myFile2.close();
    remove("TrailData.txt");
    rename("temp.txt", "TrailData.txt");

  }

  //Version 2. This one can be used if we want to send data while the vehicle has stopped for a while. Code not complete.
  if(t.getSpeed() <= "some value")
  {
    speedCounter++;
  }
  else
  {
    speedCounter = 0;
  }

  if(speedCounter > 10000) //Will have to test how quicly speed counter increases
  {
    Cellular.connect(); //Manually connect to cellular
    while(!Cellular.ready()); //Wait for cellular connection to be established.
    if (!myFile.open("TrailData.txt", O_RDWR | O_CREAT))
    {
      sd.errorHalt("opening TrailData.txt for read failed");
    }
    String data;
    while((data = myFile.read()) >= 0)
    {
      Particle.publish("Heat", data, PRIVATE);
    }
    myFile.close();
    remove("TrailData.txt");
    speedCounter = 0;
    //Put device into low power mode
  }

}
