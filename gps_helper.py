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
import RPi.GPIO as GPIO
import pynmea2
import ConfigParser
from datetime import datetime
from datetime import timedelta

from twisted.internet.task import LoopingCall
from twisted.internet import reactor
from requests_twisted import TwistedRequestsSession

import logging
session = TwistedRequestsSession()
log = logging.getLogger()

new_data = 0
data_was_updated = 0
sending_in_progress = 0

csvfile = open('/home/pi/GPS/setSensorData.csv', 'ab')
fieldnames = ["Operation","Flag","ObjectId","ObjectType","MobileRecordId","Functional Group Name","Organization Name","Organization Number","Value","Datetime","Sensor Name","SensorRecordId"]
writer = csv.DictWriter(csvfile, fieldnames=fieldnames, quoting=csv.QUOTE_ALL)

dt_now_PLC =  datetime.now()

config  = ConfigParser.RawConfigParser()

def config_set(config_main):
    global config
    config = config_main

def log_set(log_main):
    global log
    log = log_main

def convertGPSToXYZ(geoPoint):
    earthRadius = 3963.34
    Z = earthRadius*math.sin(math.radians(geoPoint[0]))
    r = math.sqrt( math.pow(earthRadius, 2) - math.pow(Z, 2) )
    Y = r * math.sin(math.radians(geoPoint[1]))
    X = math.sqrt( math.pow(earthRadius, 2) - math.pow(Y, 2) - math.pow(Z, 2) )
    if math.fabs(geoPoint[1])>90.0:
        X = -1.0 * X
    return [X, Y, Z]

def distanceTravelled(geoPoint1, geoPoint2):
    earthOne = convertGPSToXYZ(geoPoint1)
    earthTwo = convertGPSToXYZ(geoPoint2)
    return math.sqrt( math.pow(earthOne[0] - earthTwo[0], 2) + math.pow(earthOne[1] - earthTwo[1], 2) + math.pow(earthOne[2] - earthTwo[2], 2) )




def convertHHMMSSToTotalSeconds(inputTime):
    # the original input time is in the format hhmmss.xyz
    seconds = math.modf(inputTime / 100.0)
    # seconds is now a 2-tuple containing (.ssxyz, hhmm.)
    minutes = math.modf(seconds[1] / 100.0)
    # minutes is now a 2-tuple containing (.mm, hh)
    return (seconds[0] * 100.0) + ((minutes[0] * 100.0) * 60.0) + (minutes[1] * 3600.0)




def verifyVDB(inputLine, geoPoint):
#  inputLine = "$GPRMC,025100.000,A,3403.5535,N,11833.0495,W,1.16,275.11,010915,,,A*72 "
#  perform the beginning and the ending line checks before checking any of the interior data values
#  because all the interior data values are obtained after splitting the data
    global dt_now_PLC
    if inputLine[0:6] != '$GPRMC':
        log.info('Bad sentence type ' + inputLine[0:6])
        return False
    if inputLine[len(inputLine)-5:len(inputLine)-4] != '*':
        log.info('Bad ending asterisk ' + inputLine[len(inputLine)-5:len(inputLine)-4])
        return False
    calculatedCheckSum = 0
    for i in range(1,len(inputLine)-5):
        calculatedCheckSum = calculatedCheckSum ^ ord(inputLine[i])  # ^ is XOR
    checkSumString = hex(calculatedCheckSum).upper()
    checkSumString = checkSumString[2:4]
    if len(checkSumString) == 1:
        checkSumString = '0' + checkSumString
    if checkSumString != inputLine[len(inputLine)-4:len(inputLine)-2]:
        log.info('Mismatched check sums are: ' + checkSumString + ' and ' + inputLine[len(inputLine)-4:len(inputLine)-2])
        return False
    theSplit = inputLine.split(',')
    if (theSplit[2] != 'A'):
        log.info('Bad Validity')
        return False
    if (theSplit[4] != 'N') and (theSplit[4] != 'S'):
        log.info('Bad N S')
        return False
    try:
        degreesLatitude = float(theSplit[3]) / 100.0
        if theSplit[3] == 'S':
            degreesLatitude = -1.0*degreesLatitude
        geoPoint[0] = degreesLatitude    
    except ValueError:
        log.info('Latitude value error')
        return False  
    if (theSplit[6] != 'E') and (theSplit[6] != 'W'):
        log.info('Bad E W')
        return False
    try:
        degreesLongitude = float(theSplit[5]) / 100.0
        if theSplit[6] == 'E':
            degreesLongitude = -1.0*degreesLongitude
        geoPoint[1] = degreesLongitude    
    except ValueError:
        log.info('Longitude value error')
        return False
    try:
        theTime = float(theSplit[1])
        log.info('The UTC Time is: ' + str(theTime))
	theTime = convertHHMMSSToTotalSeconds(theTime)
        log.info('The UTC Time (total seconds) is: ' + str(theTime))
        geoPoint[2] = theTime
    except ValueError:
        log.info('Bad UTC Time')
        return False
    try:
        theVelocity = float(theSplit[7])
        log.info('The Velocity is: ' + str(theVelocity))
        geoPoint[3] = theVelocity
    except ValueError:
        log.info('Bad Velocity')
        return False
    try:
        theBearing = float(theSplit[8])
        log.info('The Bearing is: ' + str(theBearing))
        geoPoint[4] = theBearing
    except ValueError:
        log.info('Bad Bearing')
        return False
    try:
        theDate = theSplit[9]
        theDate = theDate[2:4] + "/" + theDate[0:2] + "/20" + theDate[4:6]  # format the date as: mm/dd/yyyy
        log.info('The Date is: ' + str(theDate))
        geoPoint[5] = theDate
    except ValueError:
        log.info('Bad Date')
        return False

    theDate = theSplit[9]
    theTime = theSplit[1]
    Month = theDate[2:4]
    Day = theDate[0:2]
    Year = "20" + theDate[4:6]
    Hour = theTime[0:2]
    Min = theTime[2:4]
    Sec = theTime[4:6]
    new_datetime = (int(Year), int(Month), int(Day), int(Hour), int(Min), int(Sec),int(0))
    dt_now_PLC = datetime( *new_datetime[:6])
    dt =dt_now_PLC - timedelta(hours=7)
    dt_now_PLC = dt
    log.info("Date Time UTC " + str(dt_now_PLC))
    return True    

def Check_WiFi():
  hostname = "www.rcofox.com"
  response = os.system("ping -c 1 " + hostname)
  if response == 0:
    pingstatus = "Network Active"
    return True
  else:
    pingstatus = "Network Error"
    return False

def Power_Off():
  response = os.system("sudo shutdown -h now")


def save_csv(val,config):
		global dt_now_PLC
		global new_data
		now = dt_now_PLC
		dt = dt_now_PLC
		timestamp = _time.mktime(dt.timetuple())
		datetime_now = datetime.strftime(dt, "%Y-%m-%d %H:%M:%S")
		log.info("write to CSV for  Distance: "+str(val))                                    
		writer.writerow({"Operation": config.get('conf','Operation'),\
				 "Flag": config.get('conf','Flag'),\
				 "ObjectId": config.get('conf','ObjectId'),\
                                 "ObjectType": config.get('conf','ObjectType'),\
                                 "MobileRecordId": "SensorData"+"-"+str(timestamp),\
                                 "Functional Group Name": config.get('conf','Functional Group Name'),\
                                 "Organization Name": config.get('conf','Organization Name'),\
                                 "Organization Number": config.get('conf','Organization Number'),\
                                 "Value": 0 if val == -9999 else str('%.2f' % val),\
                                 "Datetime": datetime_now,\
                                 "Sensor Name": config.get('conf','Sensor Name'),\
                                 "SensorRecordId": config.get('conf','SensorRecordId')\
				})
		new_data = 1

def updating_cloud():
    global csvfile
    global writer
    global new_data
    global session
    global data_was_updated
    global sending_in_progress
    global delay
    global config

    log.info("POST loop " + str(new_data))
#    file_emty = os.stat('/home/pi/GPS/setSensorData.csv').st_size==0
#    if not file_emty and Check_WiFi() and We_on_home():
#	new_data = 1
    post_url = config.get('conf','post_url')
    if new_data == 1 and sending_in_progress == 0:
        csvfile.close()
	sending_in_progress = 1
	#-----------------------------------------------#
	# if have ne data make API setSensorData
	#-----------------------------------------------#
        log.info('Upload data to Cloud')
        multiple_files = [('text', ('setSensorData.csv', open('/home/pi/GPS/setSensorData.csv', 'rb'), 'text/plain'))]
        r = session.post(post_url, files=multiple_files, timeout=60, stream=True)
	r.addCallback(print_status)
	r.addErrback(handleFailure) 

    if data_was_updated == 1:   
	#----------------------------------------------------------------#
	# Ask command for devices (first getState then if ok resetState)
	#----------------------------------------------------------------#
           log.info("Data was updatedin CSV or Cloud")
	   data_was_updated = 0

def We_on_home(currentGeoPoint,config):
  home_latitude = config.get('conf','home_latitude')
  home_longitude = config.get('conf','home_longitude')
  lat_diff = abs(float(home_latitude) - currentGeoPoint[0])
  log_diff = abs(float(home_longitude) -  currentGeoPoint[1])
  log.info(currentGeoPoint[0])
  log.info(currentGeoPoint[1])
  if lat_diff < 0.0002 and log_diff < 0.0002:
    log.info('WIFI We at home!!!!!')
    return True
  else:
    log.info('WIFI We not home!!!!!!')
    return False

def handleFailure(f):
         global csvfile
         global writer
         global new_data
         global data_was_updated
         global sending_in_progress

	 sending_in_progress = 0 
	 log.info("Timeout POST Sensor data to Cloud ")
         csvfile = open('/home/pi/GPS/setSensorData.csv', 'ab')
         data_was_updated = 1 
	 new_data = 0
         writer = csv.DictWriter(csvfile, fieldnames=fieldnames, quoting=csv.QUOTE_ALL)

def print_status(r):
 		global csvfile
    		global writer
    		global new_data
		global data_was_updated
		global sending_in_progress

		sending_in_progress = 0
                log.info('Status: '+str(r.status_code))
		log.info('Body: '+str(r.text))

                if r.status_code == 200:
                        csvfile = open('/home/pi/GPS/setSensorData.csv', 'wb')
                        new_data = 0
                        data_was_updated = 1
	                writer = csv.DictWriter(csvfile, fieldnames=fieldnames, quoting=csv.QUOTE_ALL)
                elif r.status_code == 404:
                        csvfile = open('/home/pi/GPS/setSensorData.csv', 'ab')
                        new_data = 0
                	data_was_updated = 1
		else:
                        csvfile = open('/home/pi/setSensorData.csv', 'ab')
                        new_data = 0
			data_was_updated = 1
	        writer = csv.DictWriter(csvfile, fieldnames=fieldnames, quoting=csv.QUOTE_ALL)
