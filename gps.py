#!/usr/bin/python
import time as _time
import csv
import grequests
import os
import serial
import serial.tools.list_ports
import time
import sys
import math
import wiringpi
import pynmea2
import ConfigParser
import signal
import atexit

from datetime import datetime
from datetime import timedelta
                                
from twisted.internet.task import LoopingCall
from twisted.internet import reactor
from requests_twisted import TwistedRequestsSession
import logging
import json
import ast

from gps_helper import updating_cloud
from gps_helper import verifyVDB
from gps_helper import log_set
from gps_helper import config_set
from gps_helper import We_on_home
from gps_helper import distanceTravelled
from gps_helper import save_csv
from gps_helper import str_color
from logging.handlers import RotatingFileHandler
import logging.handlers

config = ConfigParser.RawConfigParser()
config.read('/home/pi/GPS/gps.conf')
config_set(config)

log = logging.getLogger()
log.setLevel(logging.INFO)

format = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")

fh = logging.handlers.RotatingFileHandler('/home/pi/GPS/gps.log', maxBytes=(1048576*30), backupCount=3)
fh.setFormatter(format)
log.addHandler(fh)

ch = logging.StreamHandler(sys.stdout)
ch.setFormatter(format)
log.addHandler(ch)

log_set(log)

#------------------------------------------------------------#
# Globals vars
#---------------------------------------------------------------#
CRT = 0
CRG = 0 
CRPO = 0 

timeDelay = 0
theTotalDistance = 0.0
theTotalDistanceToday = 0.0
counter = 1

theGPSDevice = serial.Serial()
previousGeoPoint = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
currentGeoPoint = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

previousTime = 0.0
theVelocity = 0.0
theBearing = 0.0
theDate = 0.0
previousDate = 0.0
currentTime = 0.0
previousTime = 0.0


outputFile = open('/home/pi/GPS/LoggedData.csv', 'a')  # open log file
reader = pynmea2.NMEAStreamReader()
data = ""

delay = 0
home = False
was_home = True
mesur_dist = False

Stop_Counter = 0
Generator_Time = 0

def Init():
 global timeDelay
 global theTotalDistance
 global theTotalDistanceToday
 global counter
 global theGPSDevice
 global previousGeoPoint
 global currentGeoPoint
 global previousTime
 global theVelocity
 global theBearing
 global theDate
 global previousDate
 global theTotalDistance
 global theTotalDistanceToday
 global currentTime
 global previousTime
 global mesur_dist
 global Generator_Time
 global CRPO

 CRPO = 1

#----------------------------------------------------------------#
# Init Serial port
#----------------------------------------------------------------#
 timeDelay = 10

 signal.signal(signal.SIGINT, signal_handler)
 signal.signal(signal.SIGTERM, signal_handler)

 wiringpi.wiringPiSetup()
 wiringpi.pinMode(21, 0)
 wiringpi.pinMode(22, 0)
 wiringpi.pinMode(26, 1)

 thePortNumber = ''  # hopefully this will become non-empty later
 listOfCOMPorts = list(serial.tools.list_ports.comports())
 for i in range(len(listOfCOMPorts)):
    if listOfCOMPorts[i][1][0:12] == 'USB-Serial C':
      thePortNumber = listOfCOMPorts[i][0]      # get the active/used port number
 if thePortNumber == '':
    log.info('Sorry but the GPS device is not plugged in')
    sys.exit()
 if len(sys.argv) == 1 or  len(sys.argv) == 3:
 	theGPSDevice = serial.Serial(port=thePortNumber, baudrate=4800, bytesize=8, stopbits=1, parity='N', xonxoff=False, timeout=0)
 if len(sys.argv) == 2:
 	theGPSDevice = open('/home/pi/GPS/testgps.txt', 'r')
 log.info(str_color("blue", thePortNumber))
#-----------------------------------------------#
#  Read retain from file
#-----------------------------------------------#
 try:
  outputFile_dist = open('/home/pi/GPS/retain', 'r')  # open log file
  line = outputFile_dist.readline()
  theTotalDistance = float(line)
  line = outputFile_dist.readline()
  mesur_dist = ast.literal_eval(line)
  line = outputFile_dist.readline()
  Generator_Time = ast.literal_eval(line)
  outputFile_dist.close()
  log.info("RETAIN : Get Distance from saved file " + str(theTotalDistance))
  log.info("RETAIN : Get mesure from saved file " + str(mesur_dist))
  log.info("RETAIN : Get Generator_Time from saved file " + str(Generator_Time))
 except:
  log.info("Error not get Retain")


def GPS_Loop():
 global timeDelay
 global theTotalDistance
 global theTotalDistanceToday
 global counter
 global theGPSDevice
 global previousGeoPoint
 global currentGeoPoint
 global previousTime
 global theVelocity
 global theBearing
 global theDate
 global previousDate
 global theTotalDistance
 global theTotalDistanceToday
 global currentTime
 global previousTime
 global reader
 global data
 global outputFile
 global mesur_dist
 global config
 global CRG
 global CRT
 global CRPO
 global Stop_Counter

#-------------LOGIC--------------------
 CRG = wiringpi.digitalRead(21)
 CRT = wiringpi.digitalRead(22)
 wiringpi.digitalWrite(26,CRPO)


 if CRG == 0:
	Stop_Counter = 0
 if CRG and Stop_Counter > 60:
   	if CRPO:
		log.info('Turn off Power')
        CRPO = 0
 else:
	CRPO = 1 

#------------LOGIC-------------------------

 if len(sys.argv) == 2:
         data = theGPSDevice.read(1)
 if len(sys.argv) == 1 or  len(sys.argv) == 3:
	 data = theGPSDevice.read()
 theDistanceChange  = 0.0
 try:
  for msg in reader.next(data):
    log.info(msg)

    inputLine = str(msg) + "  "
    currentGeoPoint = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]    # Latitude  Longitude  Time  Velocity  Bearing  Date
    if verifyVDB(inputLine, currentGeoPoint):
        currentTime = currentGeoPoint[2]
        theVelocity = previousGeoPoint[3] * 1.150777    # convert knots to miles per hour
        theBearing = previousGeoPoint[4]
        theDate = currentGeoPoint[5]
   	outputFile.write(str(msg) + chr(13) + chr(10))                       # log the data
        if ( (currentTime - previousTime) > timeDelay  ) and ( (currentGeoPoint[0] != previousGeoPoint[0]) or (currentGeoPoint[1] != previousGeoPoint[1]) ):
	 if (previousGeoPoint[0] != 0 and previousGeoPoint[1] != 0):
            theDistanceChange = distanceTravelled(previousGeoPoint, currentGeoPoint)
            if (theVelocity < 1.0):
                theDistanceChange =0.0;
            if (theDate != previousDate):
                theTotalDistanceToday = 0.0
            else:
                theTotalDistanceToday = theTotalDistanceToday + theDistanceChange

            theTotalDistance = theTotalDistance + theDistanceChange
    	 previousGeoPoint[0] = currentGeoPoint[0]
    	 previousGeoPoint[1] = currentGeoPoint[1]
    	 previousGeoPoint = currentGeoPoint
    	 previousTime = currentTime
    	 previousDate = theDate
         if theDistanceChange > 0:
		log.info("Added distance " + str(theDistanceChange) + " " + str(currentGeoPoint[0]) + "/" + str(currentGeoPoint[1]) + " - " + str(previousGeoPoint[0]) + "/" + str(previousGeoPoint[1]))
	 log.debug("Counter = " + str(counter))                 # log.info the value of the counter            
    	 counter = counter + 1                              # increment the counter only when we have a valid sentence
	 return 1
    else:
	currentGeoPoint = previousGeoPoint	
 except:
  log.debug ("Can't parse");

def Logic_Loop():
 global home
 global was_home
 global theTotalDistance
 global mesur_dist
 global new_data
 global currentGeoPoint
 global config
 global Generator_Time

 log.info(str_color("blue","[Cloud loop] Lat = "+str(currentGeoPoint[0])+" Lon = "+str(currentGeoPoint[1])+" Mesur dist = "+str(mesur_dist)))

 if currentGeoPoint[0] != 0 and currentGeoPoint[1] != 0:
 	home = We_on_home(currentGeoPoint,config)
        if not home and was_home and not mesur_dist:
	   log.info("we go out from BASE !!!!!!!!!!!!!!!!")
	   mesur_dist = True
	   theTotalDistance = 0.0
#	   Generator_Time = 0
	was_home = home
	
	if mesur_dist and theTotalDistance > 1.0 and home:
	   log.info("Going distance is " + str(theTotalDistance))
	   save_csv(theTotalDistance,config,1)
           save_csv(Generator_Time,config,2)
	   mesur_dist = False
	   Generator_Time = 0


        if mesur_dist and theTotalDistance <= 1.0 and home:
           log.info("Weon home with small distance (clear) is " + str(theTotalDistance))
           theTotalDistance = 0.0
	   mesur_dist = False

	if mesur_dist:
	   log.info(" Distance is " + str(theTotalDistance))

 return True

def signal_handler(signal, frame):
    print 'You pressed Ctrl+C!'
    reactor.removeAll()
    reactor.iterate()
    reactor.stop()

Test_Counter = 0

def Timers():
 global Stop_Counter
 global theVelocity
 global theTotalDistance
 global currentGeoPoint
 global Generator_Time
 global CRG
 global Test_Counter

 outputFile_dist = open('/home/pi/GPS/retain', 'w')  # open log file
 outputFile_dist.write(str(theTotalDistance)+ chr(13) + chr(10))
 outputFile_dist.write(str(mesur_dist)+ chr(13) + chr(10))
 outputFile_dist.write(str(Generator_Time)+ chr(13) + chr(10))
 outputFile_dist.close()

 if CRG==0:
	 Generator_Time = Generator_Time + 1
 Stop_Counter = Stop_Counter + 1
 if  currentGeoPoint[0] != 0 and currentGeoPoint[1] != 0:
 	 Test_Counter = Test_Counter + 1

 if len(sys.argv) == 3:
  if Test_Counter > 5 and Test_Counter < 20:
	  home_latitude = config.set('conf','home_latitude','15')
	  home_longitude = config.set('conf','home_longitude','15')
	  log.info("SIMULLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL OUTTTTTT")
  if Test_Counter > 10:
	  theTotalDistance = 500
  if Test_Counter > 20:
          home_latitude = config.set('conf','home_latitude','34.044634')
          home_longitude = config.set('conf','home_longitude','-118.494581')
	  log.info("SIMULLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL BACK")
 return True

def fail(f):
    log.info("we got an exception: %s" % (f.getTraceback(),))


Init()

Loop1 = LoopingCall(GPS_Loop)
Loop1.start(0.001).addErrback(fail)

Loop2 = LoopingCall(Logic_Loop)
Loop2.start(1).addErrback(fail)

Loop3 = LoopingCall(updating_cloud)
Loop3.start(1).addErrback(fail)

Loop4 = LoopingCall(Timers)
Loop4.start(1).addErrback(fail)

reactor.run()
