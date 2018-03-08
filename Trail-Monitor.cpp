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
SYSTEM_MODE(SEMI_AUTOMATIC);

File myFile;
AssetTracker t = AssetTracker();
int transmittingData = 1;
long lastPublish = 0;
long lastSpeedCheck = 0;
String data[50];
int i = 0;
int j = 0;
bool writetime = false;
int speedCounter = 0;

void setup()
{
  Serial.begin(9600);
  // Wait for USB Serial
  while (!Serial)
  {
	SysCall::yield();
  }

  t.begin();	//Set up Asset Tracker bits
  Cellular.off();
  t.gpsOn();	//Initialize GPS Module
  delay(5000); //5 second delay
  int timeout = 0;
  while(!t.gpsFix())
  {
    delay(1000);
    t.updateGPS();
    timeout++;
    if(timeout >= 180) //Allow 2 minutes to find a GPS signal
    {
      System.sleep(SLEEP_MODE_SOFTPOWEROFF, 1200); //Sleep for 20 minutes
    }
  }
  if(t.getSpeed() <= 2)
  {
    delay(30000); //Delayfor 30 seconds
    t.updateGPS();
    if(t.getSpeed() <= 2)
    {
      Delay(60000);  //Delay for 60 seconds
      t.updateGPS();
      if(t.getSpeed() <= 2)
      {
        System.sleep(SLEEP_MODE_SOFTPOWEROFF, 1200);  //Dead to us, Sleep for 20 minutes
      }
    }
  }
  // Cellular.on();
  // Cellular.connect();
  // while()
  // Particle.connect();
  //Particle.syncTime();  //May need to change timestamp to GPS based

  // Initialize SdFat or print a detailed error message and halt
  // Use half speed like the native library.
  // Change to SPI_FULL_SPEED for more performance.
  if (!sd.begin(chipSelect, SPI_HALF_SPEED))
  {
    sd.initErrorHalt();
  }
  // if (!myFile.open("TrailData.txt", O_RDWR | O_CREAT | O_AT_END))
  // {
  //   sd.errorHalt("opening TrailData.txt for write failed");
  // }
  // myFile.close();

  //Cellular.disconnect();

  for (i = 0; i < 50; i++) {
    data[i]="";
  }
}

void loop()
{
	t.updateGPS();
  int speedCounter;
  // if the current time - the last time we published is greater than your set delay...
	if (millis()-lastPublish > (100)) //100ms SECOND DELAY
	{
    // Remember when we published
		lastPublish = millis();

		if(t.gpsFix())
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
      data[j] = String::format("{ \"Lat\": \"%f\", \"Lon\": \"%f\", \"Time\": \"%lu\", \"Harsh\": \"%d\"}", lat, lon, epoch, harsh);
	    j++;
      if(j == 50)
      {
        writetime = true;
        j=0;
      }
	   }
   }
 //Version 1. This one can be used if we want to attempt to send data while the vehicle is moving. Code not complete
 // if(Cellular.ready())
 //   {
	// 	//Open File in read/write mode
	// 	if (!myFile.open("TrailData.txt", O_RDWR))
	// 	{
	// 		sd.errorHalt("opening TrailData.txt for read failed");
	// 	}
 //
	// 	//Read data from SD card and delete it after the read
 //    int endOfFile = 0;
 //    /*String*/int data;
 //    while(Cellular.ready())
 //    {
 //      if((data = myFile.read()) >= 0)
 //      {
 //        endOfFile = 1;
 //        break;
 //      }
 //      Particle.publish("Heat", data, PRIVATE);
 //    }
 //    File myFile2;
 //    if (!myFile2.open("temp.txt", O_RDWR | O_CREAT | O_AT_END))
 //    {
 //      sd.errorHalt("opening temp.txt for write failed");
 //    }
 //    if(endOfFile == 0)
 //    {
 //      while((data = myFile.read()) >= 0)
 //      {
 //        myFile2.println(data)
 //      }
 //    }
 //    myFile.close();
 //    myFile2.close();
 //    remove("TrailData.txt");
 //    rename("temp.txt", "TrailData.txt");
 //
 //  }

  //Version 2. This one can be used if we want to send data while the vehicle has stopped for a while. Code not complete.
if(writetime) //5 SECOND DELAY
{ // Check Speed, if slow increment counter
  if(t.getSpeed() <= 2) //2 knots
  {
    speedCounter++;
  }
  else
  {
    // Data is valid, write it
    speedCounter = 0;
    int i;
    if (!myFile.open("TrailData.txt", O_RDWR | O_CREAT | O_AT_END))
    {
      sd.errorHalt("opening TrailData.txt for write failed");
    }
    for(i = 0; i < 50; i++)
    {
      myFile.print(data[i]+"\n");
      data[i] = "";
    }
    myFile.close();
  }
}
  if(speedCounter > 60) //If speedCounter is greater than 5 minutes
  {
    Cellular.on();  //Turn on cellular module
    Cellular.connect(); //Manually connect to cellular
    while(!Cellular.ready()); //Wait for cellular connection to be established.
    if (!myFile.open("TrailData.txt", O_RDWR | O_CREAT))
    {
      sd.errorHalt("opening TrailData.txt for read failed");
    }
    int data;
    while((data = myFile.read()) >= 0)
    {
      //Particle.publish("Heat", data, PRIVATE);
      Serial.println(data);
    }
    myFile.close();
    remove("TrailData.txt");
    speedCounter = 0;
    System.sleep(SLEEP_MODE_SOFTPOWEROFF, 600); // Sleep for 10 minutes
  }

}
