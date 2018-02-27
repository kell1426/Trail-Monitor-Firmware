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
int delayMinutes = 1;

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

  // Initialize SdFat or print a detailed error message and halt
  // Use half speed like the native library.
  // Change to SPI_FULL_SPEED for more performance.
  if (!sd.begin(chipSelect, SPI_HALF_SPEED))
  {
    sd.initErrorHalt();
  }

  // read from the file until there's nothing else in it:
  int data;
  while ((data = myFile.read()) >= 0)
  {
    Serial.write(data);
  }
  // close the file:
  myFile.close();
}

void loop()
{
	t.updateGPS();

  // if the current time - the last time we published is greater than your set delay...
	if (millis()-lastPublish > delayMinutes*60*1000)
	{
        // Remember when we published
		lastPublish = millis();

		if(t.gpsFix())
		{
			float lat = t.readLat();
			float lon = t.readLon();
			uint32_t time = t.getGpsTimestamp();
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

			String data = String::format("%f", "%f", "%lu", "%d", lat, lon, time, harsh);

		    if (!myFile.open("test.txt", O_RDWR | O_CREAT | O_AT_END))
			{
				sd.errorHalt("opening test.txt for write failed");
			}
			myFile.println(data);
			myFile.close();
	   }
   }
   if(//cell service is detected)
   {
	    String parse[4];
		//Open File in read/write mode
		if (!myFile.open("test.txt", O_RDWR))
		{
			sd.errorHalt("opening test.txt for read failed");
		}

		//Read data from SD card and delete it after the read
		String read = myFile.read();

		//Parse the data on each comma and store variables into parse[]

		myFile.close();

		//Format it to Key:Value JSON format
		String data = String::format("{ "\Lat\": \"%s\", \"Lon\": \"%s\", \"Time\": \"%s\", \"Harsh\": \"%s\"}", parse[0], parse[1], parse[2], parse[3]);

		//Send data to website
		Particle.publish("Heat", data, PRIVATE);





   }

}
