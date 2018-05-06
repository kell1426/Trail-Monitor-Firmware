# Trail-Monitor-Firmware

## Usage
### Flashing Compiled Code Downloaded from Particle Web IDE (named "Trail.bin") using Particle CLI Tool
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
