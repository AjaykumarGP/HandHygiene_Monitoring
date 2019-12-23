 
######################## Hand hygiene monitoring system using Deep Neural Network ########################################################
# Author : Ajaykumar GP(June 20, 2019)
# Description: 
# This program uses a custom trained neural network to analyze the hand washing steps performed.
# It loads the classifier and uses it to perform Hand washing steps analyses on a frame by frame video input.
# It is a initial code for handwashing steps analysis.
##########################################################################################################################################

####################
  #tf_trt_models
####################
from PIL import Image
import sys
import os
import urllib
import tensorflow.contrib.tensorrt as trt
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import tensorflow as tf
import numpy as np
import time
from tf_trt_models.detection import download_detection_model, build_detection_graph
import cv2
import numpy as np
import keras
from keras.layers.core import Dense
from keras.optimizers import Adam
from keras.metrics import categorical_crossentropy
from keras.preprocessing.image import ImageDataGenerator
from keras.preprocessing import image
from keras.models import Model
from keras.applications import imagenet_utils
from sklearn.metrics import confusion_matrix
import itertools
import matplotlib.pyplot as plt 
import time
from sklearn.externals import joblib 
import cv2

value=0
saved_model=joblib.load("./saved_model/hand_wash_model_1_11_without_empty.pkl")
#for with empty class
#hand_steps={"0":"0","1":"1","2":"4","3":"5","4":"6","5":"7","6":"8","7":"9","8":"10","9":"11","10":"2","11":"3"}

#without empty classes
hand_steps={"1":"0","2":"3","3":"4","4":"5","5":"6","6":"7","7":"8","8":"9","9":"10","10":"1","11":"2"}

import Jetson.GPIO as GPIO
GPIO.setmode(GPIO.BCM) 

#pin 7 --> 4
#pin 11 --> 17
#pin 13 --> 27
#pin 15 --> 22
#pin 19 --> 10
#pin 21 --> 9
#pin 23 --> 11
#pin 29 --> 5

GPIO.setup([4,17,27,22,10,9,11,5], GPIO.OUT, initial=GPIO.LOW)

########################################################################################################################################
   																#BLE beacon
########################################################################################################################################
import blescan
import sys
import bluetooth._bluetooth as bluez

#########################################################################################################################################
       															# LoRaWAN Stack
#########################################################################################################################################
import time
import serial
import binascii

loraNodeSerialBaud = 115200
serialLib = 0
serial_write = None
serial_read = None
ser = None
abp = "abp"
otaa = "otaa"
EU868 = "EU868"
US915 = "US915"
KR920 = "KR920"
AU915 = "AU915"
AS923 = "AS923"
IN865 = "IN865"

SpreadFactors = {7:5,8:4,9:3,10:2,11:1,12:0}
ser=serial.Serial(port='/dev/ttyUSB0', baudrate=loraNodeSerialBaud)
serial_write = ser.write
serial_read = ser.readline
serialLib = 1

def lora_push(mac_addr,steps):
	############
	# UART Functions
	############

	def uart_tx(command):
		"""Takes the command and sends it via UART via the correct library"""
		# First add at to the beginning
		command = "at+%s\r\n" % command
		#print(command)
		serial_write(str.encode(command))

		line = ser.readline()
		print(line)
		return (line)

	def uart_rx():
		"""Returns serial data"""	
		line = ser.readline()
		#print(line)
		return (line)	

	############
	# Set Settings Functions
	############

	def set_devAddr(devAddr):
		"""Set Device Address"""
		command = "set_config=dev_addr:%s" % devAddr
		return uart_tx(command)

	def set_devEUI(devEui):
		"""Set Device EUI"""
		command = "set_config=dev_eui:%s" % devEui
		return uart_tx(command)

	def set_appEUI(appEui):
		"""Set Application EUI"""
		command = "set_config=app_eui:%s" % appEui
		return uart_tx(command)

	def set_appKey(appKey):
		"""Set Application Key"""
		command = "set_config=app_key:%s" % appKey
		return uart_tx(command)

	def set_networkKey(nwsk):
		"""Set Network Key"""
		command = "set_config=nwks_key:%s" % nwsk
		return uart_tx(command)

	def set_appSessionKey(apsk):
		"""Set Application Session Key"""
		command = "set_config=apps_key:%s" % apsk
		return uart_tx(command)

	def set_loraPower(pwr):
		"""Set Lora Power Level"""
		command = "set_config=pwr_level:%s" % pwr
		return uart_tx(command)

	def set_adrMode(adr):
		"""Set Adaptive Data Rate"""
		command = "set_config=adr:%s" % adr
		return uart_tx(command)

	def set_spreadingFactor(sf):
		"""Set Spreading Factor"""
		command = "dr=%s" % SpreadFactors[sf]
		return uart_tx(command)

	def set_region(sf):
		"""Set LoRaWAN Region"""
		command = "band=%s" % sf
		return uart_tx(command)


	############
	# Get Settings Functions
	############

	def get_devAddr():
		"""Get Device Address"""
		command = "get_config=dev_addr"
		return (str(uart_tx(command)))#.split("OK")[1].split("\\")[0])	

	def get_devEUI():
		"""Get Device EUI"""
		command = "get_config=dev_eui"
		return (str(uart_tx(command)).split("OK")[1].split("\\")[0])

	def get_appEUI():
		"""Get Application EUI"""
		command = "get_config=app_eui"
		return (str(uart_tx(command)).split("OK")[1].split("\\")[0])

	def get_appKey():
		"""Get Application Key"""
		command = "get_config=app_key"
		return (str(uart_tx(command)).split("OK")[1].split("\\")[0])

	def get_networkKey():
		"""Get Network Key"""
		command = "get_config=nwks_key"
		return (str(uart_tx(command)).split("OK")[1].split("\\")[0])

	def get_appSessionKey():
		"""Get Application Session Key"""
		command = "get_config=apps_key"
		return (str(uart_tx(command)).split("OK")[1].split("\\")[0])

	def get_loraPower():
		"""Get Lora Power Level"""
		command = "get_config=pwr_level"
		return (str(uart_tx(command)).split("OK")[1].split("\\")[0])

	def get_adrMode():
		"""Get Adaptive Data Rate"""
		command = "get_config=adr"
		return (str(uart_tx(command)).split("OK")[1].split("\\")[0])

	def get_spreadingFactor():
		"""Get Spreading Factor"""
		command = "get_config=dr"
		return (str(uart_tx(command)).split("OK")[1].split("\\")[0])

	############
	# LoRa Functions
	############

	def join(mode):
		"""Join"""
		if(mode == abp):
			command = "join=abp"
			uart_tx(command)
		elif(mode == otaa):
			command = "join=otaa"
			uart_tx(command)
			# Extra response from otaa join
			line = ser.readline()
			return (True)

	def send_raw_packet(packet, port):
		"""Send raw bytes packet"""

	def send_string_packet(string, port=1, pktType=0):
		"""Send a string packet"""
		print(binascii.hexlify(string.encode()))
		command = "send=%s,%s,%s" % (pktType, port,
								 binascii.hexlify(string.encode())
								 .decode("utf-8"))
		uart_tx(command)
		# There will be an extra response
		line = ser.readline()
		return (line)

	def send_int_packet(int, port):
		"""Send integer packet"""

	def recieve_packet():
		"""Check To See if there is any response"""

	def reset_radio():
		"""Reset the RAK811 Radio Module"""
		command = "reset=0"
		return(str(uart_tx(command)).split("\\")[0], str(uart_rx()).split("\\")[0])
	
	def lora_mode(mode):
		"""Change between LoRaWAN & LoRaP2P Modes"""

	def lora_band(band):
		"""Set LoRaWAN Region"""
		command = "band="+band
		uart_tx(command)

	############
	# Main Function
	############
	
	#set_devAddr("26012E87")
	#set_devEUI("00BCB9CAAF702697")
	#set_appEUI("70B3D57ED001B3E6")
	#set_appKey("93C8FFCA898CB32142DFDE01E8287924")
	#set_appSessionKey("8477086B0F3B017D9EC0938B2B819164")
	#set_networkKey("D2C49A6052A9E5978885F38B7F679624")
	#join(otaa)
	print("sending...")
	if mac_addr=="f7:be:37:96:7d:b0":
		user_id="Ajaykumar"
	string=user_id +","+str(steps)+","+"payload"
	print(send_string_packet(string))
	print("Data sent, returning to prediction")
	return "Data_pushed_to_powerBI"
	
#############################################################################################################################################
																	#GPIO LED indication
#############################################################################################################################################	

def gpio_in(pin):

    curr_value = GPIO.HIGH
    try:
        
        print("Outputting {} to pin {}".format(curr_value, pin))
        GPIO.output(pin, curr_value)

    finally:
        print("led on")

def gpio_out():
	output_pin=[4,17,27,22,10,9,11,5]
	curr_value = False
	
	try:
		
		print("Outputting {} to pin {}".format(curr_value, output_pin))
		GPIO.output(4, curr_value)
		GPIO.output(17, curr_value)
		GPIO.output(27, curr_value)
		GPIO.output(22, curr_value)
		GPIO.output(10, curr_value)
		GPIO.output(9, curr_value)
		GPIO.output(11, curr_value)
		GPIO.output(5, curr_value)
       
	finally:
		print("pins disconnected")
		
def dict_maximum(dic):
    global value
    for key,val in dic.items():
        if value==val:
            return key

##############################################################################################################################################
															#Video input & inferance with trained model
##############################################################################################################################################

def videocapturing(mac_addr):
    time.sleep(1)
    global value,saved_model
    cap=cv2.VideoCapture(0)
    empty=0
    one=0
    two=0
    three=0
    four=0
    five=0
    six=0
    seven=0
    eight=0
    eliminating=[]
    lora_flag=0
    finished_steps=[]
    now = time.time()
    future = now + 100
	
    while time.time() < future:
        time.sleep(0.02)
        ret,frame=cap.read()
        resize_frame=cv2.resize(frame, (224, 224)) 
        hand=resize_frame
        img_load=np.expand_dims(hand,axis=0)
        img_load = img_load/255
        pred=saved_model.predict(img_load)
        cleaned=[j for i in pred for j in i]
        cleaned=np.array(cleaned)
        np.set_printoptions(suppress=True)
        dic={}
        flag=0
        for i in range(0,len(cleaned)):
            dic[i]=cleaned[flag]
            flag+=1
            
        value=max(dic.values())
        label=dict_maximum(dic)
        label=str(label)
        curr_step=""
        display_steps=""

        for key, value in hand_steps.items():
	        if label == value:
		        curr_step=key
		        display_steps=key+ " : "+ value
        print(curr_step)
        #for with empty class
        #hand_steps={"0":"0","1":"1","2":"4","3":"5","4":"6","5":"7","6":"8","7":"9","8":"10","9":"11","10":"2","11":"3"}

        #without empty classes
        #hand_steps={"1":"0","2":"3","3":"4","4":"5","5":"6","6":"7","7":"8","8":"9","9":"10","10":"1","11":"2"}

        cv2.putText(frame," "+str(display_steps),(1,250),cv2.FONT_HERSHEY_SIMPLEX,2,(255,255,255),2)
        cv2.imshow("BGR_video frame",frame)
		

		#buffer adding
        if curr_step == '0':
            empty+=1
        elif curr_step=='1' or curr_step=='2' or curr_step=='3':
            one+=1    
        elif curr_step=='4' or curr_step=="5":
	        two+=1
        elif curr_step=='6':
            three+=1
        elif curr_step=='7':
            four+=1
        elif curr_step=='8':
            five+=1
        elif curr_step=='9':
            six+=1
        elif curr_step=='10':
            seven+=1
        elif curr_step=='11':
            eight+=1

		#buffer prediction
        buffer_threshold=20
        '''if empty > buffer_threshold:
            pass
            print("inside output empty")'''
        if one > buffer_threshold and "1" not in eliminating:
            gpio_in(4)
            
            eliminating.append("1")
            print("one finished")        

        elif two > buffer_threshold and "2" not in eliminating:
            gpio_in(17)
            eliminating.append("2")
            print("two finished")

        elif three > buffer_threshold and "3" not in eliminating:
            gpio_in(27)
            eliminating.append("3")
            print("three finished")

        elif four > buffer_threshold and "4" not in eliminating:
            gpio_in(22)
            eliminating.append("4")
            print("four finished")

        elif five > buffer_threshold and "5" not in eliminating:
            gpio_in(10)
            eliminating.append("5")
            print("five finished")

        elif six > buffer_threshold and "6" not in eliminating:
            gpio_in(9)
            eliminating.append("6")
            print("six finished")

        elif seven > buffer_threshold and "7" not in eliminating:
            gpio_in(11)
            eliminating.append("7")
            print("seven finished")

        elif eight > buffer_threshold and "8" not in eliminating:
            gpio_in(5)
            eliminating.append("8")
            print("eight finished")         
         
        result=sorted(eliminating)
        print("You have finished : {}".format(result))
        if len(eliminating) >=8:
            print("Congrats you have completed all the WHO standard handwashing steps")
            cap.release()
            cv2.destroyAllWindows()
            
            if mac_addr=="f7:be:37:96:7d:b0":
                cap.release()
                cv2.destroyAllWindows()
                total=len(eliminating)
                lora_result=lora_push(mac_addr,total)
                print("returned to prediction")
                return lora_result

        if cv2.waitKey(1) ==13:
            break

    cap.release()
    cv2.destroyAllWindows()
    if mac_addr=="f7:be:37:96:7d:b0":
        remaining_steps = 8-len(eliminating)
        print("You missed out {} steps".format(remaining_steps))
        total=len(eliminating)
        lora_result=lora_push(mac_addr,total)
        print("returned to prediction")
        gpio_out()
        return lora_result


def bluetooth_log():
	
	dev_id = 0
	try:
		sock = bluez.hci_open_dev(dev_id)
		print("ble thread started")
	except:
		print("error accessing bluetooth device...")
		sys.exit(1)

	blescan.hci_le_set_scan_parameters(sock)
	blescan.hci_enable_le_scan(sock)
	changed=''
	
	while True:
		returnedList = blescan.parse_events(sock, 10)
	   
		back=''
		rssi=''
		for beacon in returnedList:
			changed=''
			#print(beacon)
			back=beacon
			changed=str(back)
			mac=changed[8:26]
			
			if mac.strip() == 'f7:be:37:96:7d:b0':
				rssi=changed[34:]
				print(mac.strip())
				print(rssi[1:])
				if int(rssi[1:]) < 57:
					print("------------------------------")
					print(rssi[1:])
					print("Gonna hand wash!!!")
					print("------------------------------")
					#sending user log
					mac_addr=mac.strip()
					result=videocapturing(mac_addr)
					print(result)
					#light out led's
					gpio_out()
					print("finished process!!!")
					print("scanning again")
					time.sleep(15)
					break
					
if __name__=='__main__':
	print("You can start \n")
	bluetooth_log()
	
	
		
    





