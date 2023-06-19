# Traffic light

This folder contains all nessecary information for the traffic light.

## Hardware

To build the traffic light, you need:
- PVC pipes
- Wooden / cardboard frame
- 4 Neopixels
- Arduino

Any Arduino capable of running the Adafruit NeoPixel library should work. See https://github.com/adafruit/Adafruit_NeoPixel#supported-chipsets which chipsets are supported. The code is tested on an Arduino Uno and Arduino Nano, so at least it works on those.

Any frame for holding the NeoPixels at the same height and placement above the intersection should work. For example you can build a frame like the one of the Duckietown traffic light, of which you can find the building instructions [here](https://docs.duckietown.com/daffy/opmanual-duckietown/assembly/traffic_lights/index.html#traffic-light-assembly-18).

## Software

For the software part of the traffic light, you need the Arduino IDE to upload the code to the microcontroller. If you don't have the Arduino IDE, you can find it [here](https://www.arduino.cc/en/software).

After the Arduino IDE is installed, open the file [./traffic-light.ino](./traffic-light.ino) in the Arduino IDE. Then download the NeoPixel library by following the instruction in the first few lines of the comments in that code file. After the NeoPixel library is installed, you can upload the code to the microcontroller by following [this guide](https://support.arduino.cc/hc/en-us/articles/4733418441116-Upload-a-sketch-in-Arduino-IDE).

For the wired connections, you can either use the default pins or set the pins you want to use in lines 19-23 in the code file.

There are a few things you can configure, for example:
- The pins how to connect the NeoPixels to the MCU.
- Which sides of the traffic light are used. This is usefull for e.g. an 3-way intersection.
- The timings of the different colours.
- The total brightness of the traffic light.
- The RGB colours of the traffic light.

After changing these configurations, you need to re-upload the code to the MCU.