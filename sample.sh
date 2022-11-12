echo "prct no : "
read p

if [ "$p" == "1" ]; then
	echo "
import RPi.GPIO as GPIO
import time
numTimes=int(input('Enter tottal number of times to blink'))
speed=float(input('Enter length of each blink(seconds) : '))
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
GPIO.setup(5,GPIO.OUT)
GPIO.setup(10,GPIO.OUT)
GPIO.setup(19,GPIO.OUT)
G PIO.setup(26,GPIO.OUT)
GPIO.setup(29,GPIO.OUT)
def Blink(numTimes,speed):
	for i in range(0,numTimes):
		GPIO.output(5,True)
		GPIO.output(10,True)
		GPIO.output(19,True)
		GPIO.output(26,True)
		GPIO.output(29,True)
		GPIO.output(29,False)
		time.sleep(speed)
		GPIO.output(26,False)
		time.sleep(speed)
		GPIO.output(19,False)
		time.sleep(speed)
		GPIO.output(10,False)
		time.sleep(speed)
		GPIO.output(5,False)
		time.sleep(speed)
Blink(numTimes,speed)
print('Done')	
" > pattern.py
fi

if [ "$p" == "2" ]; then
	echo "
# GET https://raspberrytips.nl/files/tm1637.py
import sys
import time
import datetime
import RPi.GPIO as GPIO
import tm1637
#CLK -> GPIO23 (Pin 16)
#Di0 -> GPIO24 (Pin 18)
Display = tm1637.TM1637(23,24,tm1637.BRIGHT_TYPICAL)
Display.Clear()
Display.SetBrightnes(1)
while(True):
	now = datetime.datetime.now()
	hour = now.hour
	minute = now.minute
	second = now.second
	currenttime = [ int(hour / 10), hour % 10, int(minute / 10), minute % 10 ]
	Display.Show(currenttime)
	Display.ShowDoublepoint(second % 2)
	time.sleep(1)	
	" > segment.py
fi

if [ "$p" == "3" ]; then
	echo "
# sudo apt-get install python-pip
# sudo pip install telepot
# git clone https://github.com/salmanfarisvp/TelegramBot.git
import sys
import time
import random
import datetime
import telepot
import RPi.GPIO as GPIO
def on(pin):
	GPIO.output(pin,GPIO.HIGH)
	return
def off(pin):
	GPIO.output(pin,GPIO.LOW)
	return
GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.OUT)
def handle(msg):
	chat_id = msg['chat']['id']
	command = msg['text']
	print 'Got command: %s' % command
	if command == 'on':
		bot.sendMessage(chat_id, on(11))
	elif command =='off':
		bot.sendMessage(chat_id, off(11))
bot = telepot.Bot('Bot Token')
bot.message_loop(handle)
print 'I am listening...'
while 1:
	time.sleep(10)

# bot = telepot.Bot('Bot Token')	


	" > telegram.py
fi

if [ "$p" == "4" ]; then
	echo "
# sudo nano /boot/config.txt

# dtparam=spi=on
# dtoverlay=pi3-disable-bt
# core_freq=250
# enable_uart=1
# force_turbo=1

# sudo systemctl stop serial-getty@ttyS0.service
# sudo systemctl disable serial-getty@ttyS0.service
# sudo systemctl enable serial-getty@ttyAMA0.service
# sudo apt-get install minicom
# sudo pip install pynmea2
# sudo cat /dev/ttyAMA0

import time
import serial
import string
import pynmea2
import RPi.GPIO as gpio
gpio.setmode(gpio.BCM)
port = '/dev/ttyAMA0'
ser = serial.Serial(port, baudrate = 9600, timeout = 0.5)
while 1:
	try:
		data = ser.readline()
		print data
	except:
		print('loading')
if data[0:6] == '$GPGGA':
	msg = pynmea2.parse(data)
	print msg
	time.sleep(2)

	" > gps.py
fi


if [ "$p" == "5" ]; then
	echo "
# wget https://bitbucket.org/MattHawkinsUK/rpispy-misc/raw/master/python/lcd_i2c.py	
# Enable the I2C Interface
# sudo python lcd_i2c.py

import smbus
import time
I2C_ADDR = 0x27 # I2C device address
LCD_WIDTH = 16 # Maximum characters per line
LCD_CHR = 1 # Mode - Sending data
LCD_CMD = 0 # Mode - Sending command
LCD_LINE_1 = 0x80 # LCD RAM address for the 1st line
LCD_LINE_2 = 0xC0 # LCD RAM address for the 2nd line
LCD_LINE_3 = 0x94 # LCD RAM address for the 3rd line
LCD_LINE_4 = 0xD4 # LCD RAM address for the 4th line



LCD_BACKLIGHT = 0x08 # On
#LCD_BACKLIGHT = 0x00 # Off
ENABLE = 0b00000100 # Enable bit
# Timing constants
E_PULSE = 0.0005
E_DELAY = 0.0005

#Open I2C interface

bus = smbus.SMBus(0) # Rev 1 Pi uses 0
bus = smbus.SMBus(1) # Rev 2 Pi uses 1
def lcd_init():
	lcd_byte(0x33,LCD_CMD) # 110011 Initialise
	lcd_byte(0x32,LCD_CMD) # 110010 Initialise
	lcd_byte(0x06,LCD_CMD) # 000110 Cursor move direction
	lcd_byte(0x0C,LCD_CMD) # 001100 Display On,Cursor Off, Blink Off
	lcd_byte(0x28,LCD_CMD) # 101000 Data length, number of lines, font size
	lcd_byte(0x01,LCD_CMD) # 000001 Clear display
	time.sleep(E_DELAY)
def lcd_byte(bits, mode):
	bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
	bits_low = mode | ((bits<<4) & 0xF0) | LCD_BACKLIGHT
	bus.write_byte(I2C_ADDR, bits_high)
	lcd_toggle_enable(bits_high)
	bus.write_byte(I2C_ADDR, bits_low)
	lcd_toggle_enable(bits_low)
def lcd_toggle_enable(bits):
	time.sleep(E_DELAY)
	bus.write_byte(I2C_ADDR, (bits | ENABLE))
	time.sleep(E_PULSE)
	bus.write_byte(I2C_ADDR,(bits & ~ENABLE))
	time.sleep(E_DELAY)
def lcd_string(message,line):
	message = message.ljust(LCD_WIDTH,' ')
	lcd_byte(line, LCD_CMD)
	for i in range(LCD_WIDTH):
	lcd_byte(ord(message[i]),LCD_CHR)
def main():
	lcd_init()
	while True:
		lcd_string('RPiSpy <',LCD_LINE_1)
		lcd_string('I2C LCD <',LCD_LINE_2)
		time.sleep(3)
		lcd_string('> RPiSpy',LCD_LINE_1)
		lcd_string('> I2C LCD',LCD_LINE_2)
		time.sleep(3)
if __name__ == '__main__':
	try:
		main()
	except KeyboardInterrupt:
		pass
	finally:
		lcd_byte(0x01, LCD_CMD)	
	" > lcd.py
	
fi

if [ "$p" == "6" ]; then
	touch camera1.py
	echo "
import picamera
from time import sleep
camera = picamera.PiCamera()
camera.resolution = (1024, 768)
camera.brightness = 60
camera.start_preview()
camera.annotate_text = 'Hi Pi User'
sleep(5)
camera.capture('image1.jpeg')
camera.stop_preview()" > camera1.py

	touch camera2.pycd
	
	echo "
import picamera
from time import sleep
camera = picamera.PiCamera()
camera.resolution = (640, 480)
print()
#start recording using pi camera
camera.start_recording("/home/pi/demo.h264")
#wait for video to record
camera.wait_recording(20)
#stop recording
camera.stop_recording()
camera.close()
print('video recording stopped')	
	" > camera2.py
fi



if [ "$p" == "7" ]; then
	echo "
# sudo raspi-config
# sudo apt-get update
# sudo apt-get upgrade	
# cd ~	
# sudo apt-get install build-essential python-dev python-smbus git
# git clone https://github.com/adafruit/Adafruit_Python_ADS1x15.git
# cd Adafruit_Python_ADS1x15
# sudo python setup.py install
# cd examples
# python simpletest.py
# sudo apt-get install python-matplotlib
# sudo apt-get install python-pip12
# sudo pip install drawnow
# sudo nano scope.py


import time
import matplotlib.pyplot as plt
from drawnow import *
import Adafruit_ADS1x15
adc = Adafruit_ADS1x15.ADS1115()
GAIN = 1
val = [ ]
cnt = 0
plt.ion()
adc.start_adc(0, gain=GAIN)
print('Reading ADS1x15 channel 0')
def makeFig():
	plt.ylim(-5000,5000)
	plt.title('Osciloscope')
	plt.grid(True)
	plt.ylabel('ADC outputs')
	plt.plot(val, 'ro-', label='Channel 0')
	plt.legend(loc='lower right')
while (True):
	value = adc.get_last_result()
	print('Channel 0: {0}'.format(value))
	time.sleep(0.5)
	val.append(int(value))
	drawnow(makeFig)
	plt.pause(.000001)
	cnt = cnt+1
	if(cnt>50):
		val.pop(0)	
		" > osci.py
fi



if [ "$p" == "8" ]; then
	echo "
#	Select 5 Interfacing Options -> I2C -> yes
# sudo raspi-config
# Select 5 Interfacing Options -> I2C -> yes
# sudo apt-get update
# sudo apt-get install libusb-dev libpcsclite-dev i2c-tools

# cd ~
# wget http://dl.bintray.com/nfc-tools/sources/libnfc-1.7.1.tar.bz2
# tar -xf libnfc-1.7.1.tar.bz2
# cd libnfc-1.7.1
# ./configure --prefix=/usr --sysconfdir=/etc
# make
# sudo make install
# cd /etc
# sudo mkdir nfc

# sudo nano /etc/nfc/libnfc.conf

# allow_autoscan = true
# allow_intrusive_scan = false
# log_level = 1

# device.name = '_PN532_I2c'
# device.connstring = 'pn532_i2c:/dev/i2c-1'

# i2cdetect –yes 1 
# nfc-list
# nfc-poll


# ----------------------  ----------------------  ----------------------

# sudo raspi-config
Select 9 Advanced Options -> SPI -> yes.
sudo apt-get update
sudo apt-get install libusb-dev libpcsclite-dev i2c-tools

cd ~
wget http://dl.bintray.com/nfc-tools/sources/libnfc-1.7.1.tar.bz2
tar -xf libnfc-1.7.1.tar.bz2

cd libnfc-1.7.1
./configure --prefix=/usr --sysconfdir=/etc
make
sudo make install

cd /etc

sudo mkdir nfc
sudo nano /etc/nfc/libnfc.conf

allow_autoscan = true
allow_intrusive_scan = false
log_level = 1
device.name = '_PN532_SPI'
device.connstring = 'pn532_spi:/dev/spidev0.0:500000'

#ls /dev/spidev0.*
#nfc-list

#sudo nano /etc/nfc/libnfc.conf

# device.connstring = 'pn532_spi:/dev/spidev0.0:50000'
#nfc-poll



------

import subprocess
import time
def nfc_raw():
	lines=subprocess.check_output('/usr/bin/nfc-poll',stderr=open('/dev/null','w'))
	return lines
def read_nfc():
	lines=nfc_raw()
	return lines
try:
	while True:
		myLines=read_nfc()
		buffer=[]
		for line in myLines.splitlines():
			line_content=line.split()
			if(not line_content[0] =='UID'):
				pass
			else:
				buffer.append(line_content)
		str=buffer[0]
		id_str=str[2]+str[3]+str[4]+str[5]
		print (id_str)
except KeyboardInterrupt
	pass



	" > rfid.py
fi

if [ "$p" == "9" ]; then
	echo "
code:
import RPi.GPIO as GPIO
from time import sleep
relay_pin = 26
GPIO.setmode(GPIO.BOARD)
GPIO.setup(relay_pin, GPIO.OUT)
GPIO.output(relay_pin, 1)
try:
	while True:
		GPIO.output(relay_pin, 0)
		sleep(5)
		GPIO.output(relay_pin, 1)
		sleep(5)
except KeyboardInterrupt:
	pass
	GPIO.cleanup()
	" > automation.py

fi















