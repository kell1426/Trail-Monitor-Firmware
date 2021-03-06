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
long lastGPSpoint = 0;
long lastSpeedCheck = 0;
String data[50];
int i = 0;
int j = 0;
bool writetime = false;
//String speed[50];
uint32_t initialTime = 0;
int count = 0;

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
  delay(5000); //10 second delay
  //int timeout = 0;
  // while(!t.gpsFix())
  // {
  //   delay(1000);
  //   t.updateGPS();
  //   timeout++;
  //   if(timeout >= 120) //Allow 2 minutes to find a GPS signal
  //   {
  //     System.sleep(SLEEP_MODE_SOFTPOWEROFF, 1200); //Sleep for 20 minutes
  //   }
  // }
  while(1)
  {
    t.updateGPS();
    if(t.gpsFix())
    {
      break;
    }
  }
  //Could add in an acceleromer reading
  // && (t.readXYZmagnitude() < 8500 && t.readXYZmagnitude() > 7000)
  // if(t.getSpeed() <= 2)
  // {
  //   delay(30000); //Delayfor 30 seconds
  //   t.updateGPS();
  //   if(t.getSpeed() <= 2)
  //   {
  //     Delay(60000);  //Delay for 60 seconds
  //     t.updateGPS();
  //     if(t.getSpeed() <= 2)
  //     {
  //       System.sleep(SLEEP_MODE_SOFTPOWEROFF, 1200);  //Dead to us, Sleep for 20 minutes
  //     }
  //   }
  // }
  Cellular.on();
  Cellular.connect();
  while(!Cellular.ready());
  Particle.connect();
  Particle.syncTime();  //May need to change timestamp to GPS based
  initialTime = Time.local() * 1000;
  Serial.println(Time.local());
  Serial.println(initialTime);

  // Initialize SdFat or print a detailed error message and halt
  // Use half speed like the native library.
  // Change to SPI_FULL_SPEED for more performance.
  if (!sd.begin(chipSelect, SPI_HALF_SPEED))
  {
    sd.initErrorHalt();
  }

  for (i = 0; i < 50; i++)
  {
    data[i]="";
  }
  Particle.disconnect();
  Cellular.disconnect();
  Cellular.off();
}

void loop()
{
  t.updateGPS();
  //int speedCounter;
  // if the current time - the last time we published is greater than your set delay...
	if (millis()-lastGPSpoint > (100)) //100ms SECOND DELAY
	{
    // Remember when we published
		lastGPSpoint = millis();

		if(t.gpsFix())
		{
			float lat = t.readLatDeg();
			float lon = t.readLonDeg();
			//uint32_t epoch = Time.local(); //May be able to call Time.local() in setup and store (Time.local() * 1000) in a variable
                                     //Then set eopch to variable + millis();
      uint32_t epoch = initialTime + millis();
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
      data[j] = String::format("{ \"Lat\": \"%f\", \"Lon\": \"%f\", \"Time\": \"%lu\", \"Harsh\": \"%d\" }", lat, lon, epoch, harsh);
      //speed[j] = "Speed is: " + String::format("%f", t.getSpeed());
      j++;
      if(j == 50)
      {
        writetime = true;
        j=0;
      }
	   }
   }

  //Version 2. This one can be used if we want to send data while the vehicle has stopped for a while. Code not complete.
 // Check Speed, if slow increment counter
  if(/*t.getSpeed() <= 2 ||*/(t.readXYZmagnitude() < 8500) && (t.readXYZmagnitude() > 7000)) //2 knots
  {
    delay(5000);
    count++;
    Serial.println(count);
  }
  if(writetime)
  {
    // Data is valid, write it
    count = 0;
    int i;
    if (!myFile.open("TrailData.txt", O_RDWR | O_CREAT | O_AT_END))
    {
      sd.errorHalt("opening TrailData.txt for write failed");
    }
    for(i = 0; i < 50; i++)
    {
      myFile.println(data[i]);
      //myFile.println(speed[i]);
      data[i] = "";
    }
    myFile.close();
    writetime = false;
  }

  if(count >= 6) //If speedCounter is greater than 5 minutes (60)
  {
    Cellular.on();
    Cellular.connect(); //Manually connect to cellular
    while(!Cellular.ready()); //Wait for cellular connection to be established.
    Particle.connect();
    if (!myFile.open("TrailData.txt", O_RDWR))
    {
      sd.errorHalt("opening TrailData.txt for read failed");
    }
    char arr[256];
    size_t n;
    while((n = myFile.fgets(arr, sizeof(arr))) > 0)
    {
        String str(arr);
        Serial.println(str);
         //   //Particle.publish("Heat", data, PRIVATE);
    }
    myFile.close();
    // String data;
    // Particle.connect();
    // std::ifstream ifs;
    // ifs.open("TrailData.txt");
    // while(!ifs.eof())
    // {
    //   std::getline(ifs, data);
    //   //Particle.publish("Heat", data, PRIVATE);
    //   Serial.write(data);
    // }
    // ifs.close();
    //remove("TrailData.txt");
    count = 0;
    System.sleep(SLEEP_MODE_SOFTPOWEROFF, 1200); // Sleep for 20 minutes
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
