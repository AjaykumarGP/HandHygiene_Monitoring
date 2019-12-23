
#################################### Jetson Nano Pin Configuration ######################################################

Jetson TX1, TX2, AGX Xavier, and Nano development boards contain a 40 pin GPIO header, similar to the 40 pin header in the Raspberry Pi. These GPIOs can be controlled for digital input and output using the Python library provided in the Jetson GPIO Library package. The library has the same API as the RPi.GPIO library for Raspberry Pi in order to provide an easy way to move applications running on the Raspberry Pi to the Jetson board.


#Below link will help you through package installation, setting user permission and sample scripts etc.
https://github.com/NVIDIA/jetson-gpio

Reference Links
https://www.element14.com/community/community/designcenter/single-board-computers/blog/2019/05/21/nvidia-jetson-nano-developer-kit-pinout-and-diagrams
https://raspi.tv/2013/rpi-gpio-basics-4-setting-up-rpi-gpio-numbering-systems-and-inputs


Here I have used BCM numbering for the I/O pins
import Jetson.GPIO as GPIO
GPIO.setmode(GPIO.BCM) 

GPIO.setup([4,17,27,22,10,9,11,5], GPIO.OUT, initial=GPIO.LOW)

#pin 7 in dev board --> GPIO 4
#pin 11 --> GPIO 17
#pin 13 --> GPIO 27
#pin 15 --> GPIO 22
#pin 19 --> GPIO 10
#pin 21 --> GPIO 9
#pin 23 --> GPIO 11
#pin 29 --> GPIO 5

########################################## Connectivity ###################################################################
Bluetooth Low Energy Beacon(Adafruit Feather nRF52 Bluefruit LE - nRF52832)
https://www.adafruit.com/product/3406

And Used LoRaWAN - RAK811 LoRa Module for pushing data from local node to centeralized gate way for the report generation for administarter inference.
#############################################################################################################################

