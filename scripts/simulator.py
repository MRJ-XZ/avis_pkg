#!/usr/bin/env python3
#import AVISEngine
import time
import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as Imagemsg
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Int16
#avis begin
import os
import io
import re
import math
import base64
import socket
from PIL import Image
from array import array

counter=0
speed = 0
steering = 0

#Take in base64 string and return PIL image
def stringToImage(base64_string):
    imgdata = base64.b64decode(base64_string)
    return Image.open(io.BytesIO(imgdata))

#convert PIL Image to an RGB image( technically a numpy array ) that's compatible with opencv
def toRGB(image):
    return cv2.cvtColor(np.array(image), cv2.COLOR_BGR2RGB)

class car():
    steering_value = 0
    speed_value = 0
    sensor_status = 1
    image_mode = 1
    get_Speed = 1
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    data_arr = [speed_value,steering_value,image_mode,sensor_status,get_Speed]
    data_str = "Speed:" + str(data_arr[0]) + ",Steering:" + str(data_arr[1]) + ",ImageStatus:" + str(data_arr[2]) + ",SensorStatus:" + str(data_arr[3]) + ",GetSpeed:" + str(data_arr[4])
    image = None
    sensors = None
    current_speed = None
    def connect(self,server,port):
        try:
            self.sock.connect((server, port))
            self.sock.settimeout(5.0)
            print("connected to ", server, port)
            return True
        except:
            print("Failed to connect to ", server, port , end='\r')
            return False

    def setSteering(self,steering):
        self.steering_value = steering
        self.image_mode = 0
        self.sensor_status = 0
        self.updateData()
        self.sock.sendall(self.data_str.encode("utf-8"))
        time.sleep(0.01)

    def setSpeed(self,speed):
        self.speed_value = speed
        self.image_mode = 0
        self.sensor_status = 0
        self.updateData()
        self.sock.sendall(self.data_str.encode("utf-8"))
        time.sleep(0.01)
    
    def move(self):
        self.updateData()
        self.sock.sendall(self.data_str.encode("utf-8"))
         
    def getData(self):
        self.image_mode = 1
        self.sensor_status = 1
        self.updateData()
        self.sock.sendall(self.data_str.encode("utf-8"))
        recive = self.sock.recv(131072).decode("utf-8")
        imageTagCheck = re.search('<image>(.*?)<\/image>', recive)
        sensorTagCheck = re.search('<sensor>(.*?)<\/sensor>', recive)
        speedTagCheck = re.search('<speed>(.*?)<\/speed>', recive)
        
        
        try:
            if(imageTagCheck):
                imageData = imageTagCheck.group(1)
                im_bytes = base64.b64decode(imageData)
                im_arr = np.frombuffer(im_bytes, dtype=np.uint8)  # im_arr is one-dim Numpy array
                imageOpenCV = cv2.imdecode(im_arr, flags=cv2.IMREAD_COLOR)
                self.image = imageOpenCV
            
            if(sensorTagCheck):
                sensorData = sensorTagCheck.group(1)
                sensor_arr = re.findall("\d+", sensorData)
                sensor_int_arr = list(map(int, sensor_arr)) 
                self.sensors = sensor_int_arr
            else:
                self.sensors = [1500,1500,1500]
            if(speedTagCheck):
                current_sp = speedTagCheck.group(1)
                self.current_speed = int(current_sp)
            else:
                self.current_speed = 0
            
            
        except:
            pass
            

    def getImage(self):
        return self.image

    def getSensors(self):
        return self.sensors
    
    def getSpeed(self):
        return self.current_speed
    
    def updateData(self):
        data = [self.speed_value,self.steering_value,self.image_mode,self.sensor_status,self.get_Speed]
        self.data_str = "Speed:" + str(data[0]) + ",Steering:" + str(data[1]) + ",ImageStatus:" + str(data[2]) + ",SensorStatus:" + str(data[3]) + ",GetSpeed:" + str(data[4])
    
    def stop(self):
        self.setSpeed(0)
        self.setSteering(0)
        self.sock.sendall("stop".encode("utf-8"))
        self.sock.close()
        print("done")
    
    def __del__(self):
        self.stop()
#avis end

def main_callback():
    
    global counter , speed , steering
    #Counting the loops
    counter = counter + 1

    #Set the power of the engine the car to 20, Negative number for reverse move, Range [-100,100]
    car.setSpeed(speed)

    #Set the Steering of the car -10 degree from center
    car.setSteering(steering)
    
    # print('speed: ' ,msg.data[0] ,'steering: ' , msg.data[1] ,'              ', end='\r' )

    #Get the data. Need to call it every time getting image and sensor data
    car.getData()

    #Start getting image and sensor data after 4 loops. for unclear some reason it's really important 
    if(counter > 4):
        #returns a list with three items which the 1st one is Left sensor data, the 2nd one is the Middle Sensor data, and the 3rd is the Right one.
        sensors = car.getSensors() 
        
        #EX) sensors[0] returns an int for left sensor data in cm
        sensor_msg.data = sensors

        sensor_publisher.publish(sensor_msg)
        #publish sensor data to topic

        #returns an opencv image type array. if you use PIL you need to invert the color channels.
        image = car.getImage()
        #rosImage= bridge.imgmsg_to_cv2(image ,"bgr8")
        #publish new images for midline and yolo node
        image_publisher.publish(bridge.cv2_to_imgmsg(image , "bgr8"))
        #returns an integer which is the real time car speed in KMH
        carSpeed = car.getSpeed()
        carspeed_publisher.publish(carSpeed)
        #Don't print data for better performance
        if(debug_mode):
            print("Speed : ",carSpeed) 
            #currently the angle between the sensors is 30 degree TODO : be able to change that from conf.py
            print("Left : " + str(sensors[0]) + "   |   " + "Middle : " + str(sensors[1])  +"   |   " + "Right : " + str(sensors[2]))

        #showing the opencv type image
        # cv2.imshow('frames', image)
        #break the loop when q pressed
        # if cv2.waitKey(1) == ord('q'):
        #     return
      
        # time.sleep(0.001)
        #A brief sleep to make sure everything 
        
def speedCallback(msg):
    global speed
    speed = msg.data
    main_callback()
    
def steeringCallback(msg):
    global steering
    steering = msg.data
    
if __name__=='__main__':
    #Calling the class
    car = car()
    while True:
        if car.connect("127.0.0.1", 25001):
            break
    sensor_msg = Int16MultiArray()
    debug_mode = False
    #for converting images from cv2 format to ros format
    bridge = CvBridge()
    #define our node simulator to ros
    rospy.init_node("simulator",anonymous=True)
    #define new Subscriber for controling the car
    rospy.Subscriber("speed" , Int16 , speedCallback)
    rospy.Subscriber("steering" , Int16 , steeringCallback)

    #define new publisher 'images' to publish images on ros
    image_publisher = rospy.Publisher('images' , Imagemsg,queue_size=1)
    #define new publisher 'images' to publish images on ros
    sensor_publisher = rospy.Publisher('sensors'  , Int16MultiArray , queue_size=1)
    carspeed_publisher = rospy.Publisher('carspeed'  , Int16 , queue_size=1)
    #sleep for 3 seconds to make sure that client connected to the simulator 
    # time.sleep(1)
    rospy.spin()
    
    car.stop()
    cv2.destroyAllWindows()