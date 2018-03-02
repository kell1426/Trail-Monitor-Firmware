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

  if (!myFile.open("test.txt", O_RDWR | O_CREAT | O_AT_END))
  {
    sd.errorHalt("opening test.txt for write failed");
  }
  myFile.close();
}

void loop()
{
	t.updateGPS();

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
        String test_data = String::format("Lat: %f, Lon: %f, Time: %lu, Harsh: %d", lat, lon, epoch, harsh);
        Particle.publish("Test Data", test_data, PRIVATE);
        //String data = String::format("{ \"Lat\": \"%s\", \"Lon\": \"%s\", \"Time\": \"%s\", \"Harsh\": \"%s\"}", lat, lon, epoch, harsh);
		  if (!myFile.open("test.txt", O_RDWR | O_CREAT | O_AT_END))
			{
				sd.errorHalt("opening test.txt for write failed");
			}
          myFile.println(test_data);
          //myFile.println(data);
	      myFile.close();

	   }
   }
 // if(Cellular.ready() /* && Speed (Vehicle Not Moving)*/)
 //   {
	// 	//Open File in read/write mode
	// 	if (!myFile.open("test.txt", O_RDWR))
	// 	{
	// 		sd.errorHalt("opening test.txt for read failed");
	// 	}
 //
	// 	//Read data from SD card and delete it after the read
	// 	String data = myFile.read();
 //
	// 	myFile.close();
 //
	// 	//Send data to website
	// 	Particle.publish("Heat", data, PRIVATE);
 //
 //
 //
 //
 //
 //  }

}
