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

config = ConfigParser.RawConfigParser()
config.read('/home/pi/GPS/gps.conf')
post_url = config.get('conf','post_url')
session = TwistedRequestsSession()

import logging
logging.basicConfig(filename='/home/pi/GPS/gps.log',level=logging.INFO,format='%(asctime)s %(message)s', datefmt='%m/%d/%Y %I:%M:%S %p')
logging.basicConfig(format='%(asctime)s %(message)s', datefmt='%m/%d/%Y %I:%M:%S %p')
log = logging.getLogger()
#log.setLevel(logging.INFO)


csvfile = open('/home/pi/GPS/setSensorData.csv', 'ab')
fieldnames = ["Operation","Flag","ObjectId","ObjectType","MobileRecordId","Functional Group Name","Organization Name","Organization Number","Value","Datetime",\
"Sensor Name","SensorRecordId"]
writer = csv.DictWriter(csvfile, fieldnames=fieldnames, quoting=csv.QUOTE_ALL)

#------------------------------------------#
#Save data to CSV file
#------------------------------------------#
dt_now_PLC =  datetime.now()

def save_csv(val):
		global dt_now_PLC
		now = dt_now_PLC
		dt = dt_now_PLC
		timestamp = _time.mktime(dt.timetuple())
		datetime_now = datetime.strftime(dt, "%Y-%m-%d %H:%M:%S")
		log.info("write to CSV for  Distance:"+str(val))                                    
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
#        log.info('Bad sentence type ' + inputLine[0:6])
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


#---------------------------------------------------------------#
# Globals vars
#---------------------------------------------------------------#
timeDelay = 0
theTotalDistance = 0.0
theTotalDistanceToday = 0.0
counter = 1
theGPSDevice = serial.Serial()
previousGeoPoint = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
currentGeoPoint = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

previousTime = previousGeoPoint[2]
theVelocity = previousGeoPoint[3] * 1.150777
theBearing = previousGeoPoint[4]
theDate = previousGeoPoint[5]
previousDate = theDate
theTotalDistance = 0.0
theTotalDistanceToday = 0.0
currentTime = 0.0
previousTime = currentTime



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
#----------------------------------------------------------------#
# Init Serial port
#----------------------------------------------------------------#
 timeDelay = 10
 GPIO.setmode(GPIO.BCM)
 GPIO.setup(2, GPIO.IN)

 thePortNumber = ''  # hopefully this will become non-empty later
 listOfCOMPorts = list(serial.tools.list_ports.comports())
 for i in range(len(listOfCOMPorts)):
    if listOfCOMPorts[i][1][0:12] == 'USB-Serial C':
      thePortNumber = listOfCOMPorts[i][0]      # get the active/used port number
 if thePortNumber == '':
    log.info('Sorry but the GPS device is not plugged in')
    sys.exit()
 theGPSDevice = serial.Serial(port=thePortNumber, baudrate=4800, bytesize=8, stopbits=1, parity='N', xonxoff=False, timeout=0)
 log.info(thePortNumber)






theTotalDistance = 0.0
theTotalDistanceToday = 0.0
counter = 1
headerLine = 'Count:' + "," + 'Date (dd/mm/yyyy):' + "," + 'Time (decimal seconds):' + "," + 'Latitude (decimal degrees):' + "," + 'Longitude (decimal degrees):' + "," + 'Velocity (miles per hour):' + "," + 'Current Bearing (degrees):' + "," + 'Total Distance (miles):' + "," + 'Total Distance Today (miles):' + chr(13) + chr(10)
outputFile = open('/home/pi/GPS/LoggedData.csv', 'a')  # open log file
outputFile.write(headerLine)
outputFile.close()


reader = pynmea2.NMEAStreamReader()
data = ""

def LoopProc():
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
 
 data = theGPSDevice.read()

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
        if ( (currentTime - previousTime) > timeDelay  ) and ( (currentGeoPoint[0] != previousGeoPoint[0]) or (currentGeoPoint[1] != previousGeoPoint[1]) ):
            theDistanceChange = distanceTravelled(previousGeoPoint, currentGeoPoint)
            if (theVelocity == 0.0):
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
            outputLine = str(counter) + "," + str(theDate) + "," + str(currentTime) + "," + str(currentGeoPoint[0]) + "," + str(currentGeoPoint[1]) + "," + format(theVelocity, '.1f') + "," + format(theBearing, '.1f') + "," + format(theTotalDistance, '.1f') + "," + format(theTotalDistanceToday, '.1f') + chr(13) + chr(10)
            outputFile = open('/home/pi/GPS/LoggedData.csv', 'a')  # open log file in append mode
            outputFile.write(outputLine)                       # log the data
            outputFile.close()                                 # keep the file closed, except when writing to it!
            outputFile = open('/home/pi/GPS/gpsnow.csv', 'w')      # open log file and erase any existing file first
            outputFile.write(headerLine)                       # first write the header
            outputFile.write(outputLine)                       # now write the last logged data line
            outputFile.close()                                 # keep the file closed, except when writing to it!
            log.info("Counter = " + str(counter))                 # log.info the value of the counter            
            counter = counter + 1                              # increment the counter only when we have a valid sentence
	    return 1

 except:
  log.info ("Can't parse");

def We_on_home():
  global currentGeoPoint
  home_latitude = config.get('conf','home_latitude')
  home_longitude = config.get('conf','home_longitude')
  lat_diff = abs(float(home_latitude) - currentGeoPoint[0])
  log_diff = abs(float(home_longitude) -  currentGeoPoint[1])
  log.info(currentGeoPoint[0])
  log.info(currentGeoPoint[1])
  if lat_diff < 0.00009 and log_diff < 0.00009:
    log.info('WIFI We at home!!!!!')
    return True
  else:
    log.info('WIFI We not home!!!!!!')
    return False

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


new_data = 0
data_was_updated = 0
sending_in_progress = 0

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
delay = 0

def updating_cloud():
    global csvfile
    global writer
    global new_data
    global session
    global data_was_updated
    global sending_in_progress
    global delay

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

home = False
was_home = True
mesur_dist = False

def Cloud_Loop():
 global home
 global was_home
 global theTotalDistance
 global mesur_dist
 global new_data

 if currentGeoPoint[0] != 0 and currentGeoPoint[1] != 0:
 	home = We_on_home()
        if not home and was_home and not mesur_dist:
	   log.info("we go out from BASE !!!!!!!!!!!!!!!!")
	   mesur_dist = True
	   theTotalDistance = 0.0
	was_home = home
	
	if mesur_dist and theTotalDistance > 5.0 and home and Check_WiFi():
	   log.info("Going distance is " + str(theTotalDistance))
	   save_csv(theTotalDistance)
	   mesur_dist = False
 	   new_data = 1
	if mesur_dist:
	   log.info(" Distance is " + str(theTotalDistance))
 return True



Stop_Counter = 0 

def Timers():
 global Stop_Counter
 global theVelocity
 global theTotalDistance
#log.info("Timer loop")
# if theVelocity < 1 and not GPIO.input(2):
 Stop_Counter = Stop_Counter + 1
# else:
#	Stop_Counter = 0
# if Stop_Counter > 5 and Stop_Counter < 20:
#	  home_latitude = config.set('conf','home_latitude','15')
#	  home_longitude = config.set('conf','home_longitude','15')
#	  log.info("SIMULLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL OUTTTTTT")
# if Stop_Counter > 10:
#	  theTotalDistance = 500
# if Stop_Counter > 20:
#          home_latitude = config.set('conf','home_latitude','34.026816')
#          home_longitude = config.set('conf','home_longitude','118.296722')
#	  log.info("SIMULLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL BACK")
 return True

def fail(f):
    log.info("we got an exception: %s" % (f.getTraceback(),))


Init()

Loopc = LoopingCall(LoopProc)
Loopc.start(0.001).addErrback(fail)

Loop_cloud = LoopingCall(Cloud_Loop)
Loop_cloud.start(1).addErrback(fail)

Loop_POST = LoopingCall(updating_cloud)
Loop_POST.start(1).addErrback(fail)

Loop_timers = LoopingCall(Timers)
Loop_timers.start(1).addErrback(fail)

reactor.run()
