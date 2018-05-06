# Trail-Monitor-Firmware

## Usage
- Update the code and compile using the Particle Web IDE, with correct firmware version selected.
- The udated code can be pushed over cellular if the device is turned on.
### Flashing Compiled Code Downloaded from Particle Web IDE (named "Trail.bin") using Particle CLI Tool over Serial (USB) Connection
[Particle CLI Reference](https://docs.particle.io/reference/cli/)
#### Windows:
```
particle flash --serial firmware.bin 
```
#### UNIX:
```
particle serial flash Trail.bin
```

## Debug Modes
- Debug 0 is the normal operation mode.
- Debug 1 is for serail connections and print outs to a terminal.
- Debug 2 prints out Accelerometer information to the "IMUData.txt" file as well as speed data to the "Speeddata.txt" file
- Debug 3 used in earlier versions of code. No longer does anything
- Debug 4 used in earlier versions of code. No longer does anything
- Debug 5 prints out the acceleromter data used in the harshess algorithm to the "IMUData.txt" file as well as speed data to the "Speeddata.txt" file

## Suspension Factor
Used for adjusting the roughness RMS calculation to the suspension in the vehicle.  A higher suspension factor will result in higher roughness readings.  

## Speed Factor
Not yet implemented, would take the device's current speed into account in the roughness algorithm.
