#!/usr/bin/python
# -*- coding: UTF-8 -*

import cv2
from cv2 import cv
import imutils
import numpy as np

import rospy
import time
from robot_grasp.msg import real_detected_obj_array
from robot_grasp.msg import real_detected_obj
from robot_grasp.srv import calibration_transform
from robot_grasp.srv import real_kinova_pick
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge, CvBridgeError

index=0
result_list = []
grasp_list = ['bag', 'bear', 'sausage', 'cake', 'milk box']
bridge = CvBridge()
#pub = rospy.Publisher('pose', real_detectresult_paramed_obj_array, queue_size=1)

def get_transform(obj_param_in_cam):
    get_transform_prox = rospy.ServiceProxy('/transform',calibration_transform)
    return get_transform_prox(obj_param_in_cam).obj_real_detected_in_base

def get_real_routine(obj_param_in_base):
    get_real_routine_prox = rospy.ServiceProxy('/get_real_pick_place_routine',real_kinova_pick)
    return get_real_routine_prox(obj_param_in_base).success

def callback(img):
	try:
		image = bridge.imgmsg_to_cv2(img, "bgr8")
	except CvBridgeError as e:
		print(e)
	# convert the resized image to grayscale, blur it slightly,
	# and threshold it
        image = image[0:700, 100:1200]
	gray = cv2.cvtColor(image.copy(), cv2.COLOR_BGR2GRAY)
	blurred = cv2.GaussianBlur(gray, (5, 5), 0)
	thresh = cv2.threshold(blurred, 155, 255, cv2.THRESH_BINARY)[1]
	# thresh = cv2.adaptiveThreshold(blurred,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,20,2)
	# kernel = np.ones((5,5), np.uint8)
	# thresh = cv2.morphologyEx(thresh, kernel, cv2.MORPH_CLOSE, kernel)
	#cv2.imshow("thresh", thresh)
        #cv2.waitKey(0)

	# find contours in the thresholded image
	cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
	    cv2.CHAIN_APPROX_SIMPLE)
	cnts = cnts[0] if imutils.is_cv2() else cnts[1]

        obj_param_single = real_detected_obj()

	for c in cnts:
		area = cv2.contourArea(c)
		if area < 500:
		   continue
		# compute the center of the contour, then detect the name of the
		# shape using only the contour
		M = cv2.moments(c)
		cX = int(M["m10"] / M["m00"])
		cY = int(M["m01"] / M["m00"])
		list_ = []
		position = [cX, cY]
		orientation = 0
		property_ = {}
		property_['circle'] = 0
		property_['rectangle'] = 0
		property_['eclipse'] = 0
		name = ' ' 

		# Circle detection
		(x,y),radius = cv2.minEnclosingCircle(c)
		center = (int(x),int(y))
		radius = int(radius)
		circle_area = np.pi*radius*radius
		# print(area/circle_area)
		if area/circle_area > 0.7 and area < 15500:  
		    img = cv2.circle(image,center,radius,(255,0,0),2)
		    # cv2.putText(image, 'circle', center, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
		    position = [x, y]  # more accurate
		    property_['circle'] = 1
		    # define mask with coutour
		    mask = np.zeros(gray.shape, np.uint8)
		    cv2.drawContours(mask,[c],0,255,-1)
		    # cv2.imwrite("mask.png", mask)
		    mean_val, _, _, _ = cv2.mean(gray,mask = mask)
		    mean_val = int(mean_val)
		    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(gray,mask = mask)
		    if max_val-min_val > 190:  # 受光照影响，差值最大为cake
			 name = 'cake'result_param
			 cv2.putText(image, 'cake', center, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
		    elif mean_val > 200:  # 受光照影响，大值为lemon，小值为orange
			 name = 'lemon'  
			 cv2.putText(image, 'lemon', center, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
		    else:
			 name = 'orange'
			 cv2.putText(image, 'orange', center, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
		    print(name)
		# Rectangle detection
		if property_['circle'] == 0:
		    # print("hello")
		    rect = cv2.minAreaRect(c)  # top-left corner(x,y), (width, height), angle of rotation
		    rectangle_area = rect[1][0]*rect[1][1]
		    # print(area/rectangle_area)
		    aspect_ratio = rect[1][0]/rect[1][1] if rect[1][0] > rect[1][1] else rect[1][1]/rect[1][0]
		    # print aspect_ratio
		    if area/rectangle_area > 0.80 and aspect_ratio < 1.5:
			box = cv2.cv.BoxPoints(rect)
		
			# image = cv2.drawContours(image, [box], 0, (255,0,0), 2)
			name = 'milk box'
			cv2.putText(image, 'milk box', center, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
			property_['rectangle'] = 1
			# 旋转角度θ是水平轴（x轴）逆时针旋转，与碰到的矩形的第一条边的夹角。并且这个边的边长是width，另一条边边长是height。也就是说，在这里，width与height不是按照长短来定义的。相对于x轴，逆时针旋转角度为负，顺时针旋转角度为正。在这里，θ∈（-90度，0]
			orientation = rect[2] if rect[1][0] > rect[1][1] else rect[2]-90  # 如果宽大于长，则旋转角再减去90度，得到的orientation为沿水平轴逆时针旋转的角度值（负值）
			print("milk box %f" % orientation)
		    # print(aspect_ratio)
		    if aspect_ratio > 1.9 and area < 7500:
			ellipse = cv2.fitEllipse(c)
			image = cv2.ellipse(image,ellipse,(255,0,0),2)
			name = 'sausage'
			cv2.putText(image, 'sausage', center, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
			property_['eclipse'] = 1
			orientation = rect[2] if rect[1][0] > rect[1][1] else rect[2]-90  
			print("sausage %f" % orientation)

		# other detectionsresult_param
		if name == ' ':
			mask = np.zeros(gray.shape, np.uint8)
			cv2.drawContours(mask,[c],0,255,-1)
			min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(gray,mask = mask)
			mean_val, _, _, _ = cv2.mean(gray,mask = mask)
			mean_val = int(mean_val)
			perimeter = cv2.arcLength(c,True)
			    
			    # print(area, perimeter)
			if area > 16000 and perimeter < 700:
			    # print(area)
			    name = 'hippo'
			    cv2.putText(image, 'hippo', center, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
			    rect = cv2.minAreaRect(c) 
			    orientation = rect[2] if rect[1][0] > rect[1][1] else rect[2]-90
			    print("hippo %f" % orientation)

			elif perimeter < 650 and perimeter > 500 and mean_val > 200:
			    name = 'cream'
			    cv2.putText(image, 'cream', center, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
			    rect = cv2.minAreaRect(c) 
			    orientation = rect[2] if rect[1][0] > rect[1][1] else rect[2]-90
			    print("cream %f" % orientation)

			elif perimeter < 5000 and mean_val < 210 and perimeter < 800:
			    name = 'ball'
			    # ball recovery
			    (x,y),radius = cv2.minEnclosingCircle(c)
			    center = (int(x),int(y))
			    radius = int(radius)
			    img = cv2.circle(image,center,radius,(255,0,0),2)
			    position = [x, y]  # more accurate
			    property_['cirresult_paramcle'] = 1
			    cv2.putText(image, 'ball', center, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

			elif perimeter < 920 and area < 10000 and mean_val > 170 and mean_val < 200:  # 受光照影响
			    # print(max_val - min_val)
			    name = 'bear'
			    cv2.putText(image, 'bear', center, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
			    rect = cv2.minAreaRect(c) 
			    orientation = rect[2] if rect[1][0] > rect[1][1] else rect[2]-90
			    print("bear %f" % orientation)

			else:
			    # print(perimeter)
			    name = 'bag'
			    cv2.putText(image, 'bag', center, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
			    rect = cv2.minAreaRect(c) 
			    orientation = rect[2] if rect[1][0] > rect[1][1] else rect[2]-90 
			    print("bag %f" % orientation)
			print(name)
                if name not in grasp_list:
			continue
                result_param = []
                transformed_param = []
                result_param.append(position[0]+100)
		result_param.append(position[1])
		result_param.append(orientation)
                obj_param_single.name = name
                obj_param_single.pose = result_param
              
                result_param = []
                
		# print(obj_param_single.pose)
                #obj_param_single.pose.append(position)
                #obj_param_single.pose.append(orientation)

		# list_.append(name)
		# list_.append([position, orientation])
                #cv2.imshow('Image', image2)
                #cv2.waitKey(0)
		#cv2.imshow('Image', image)
		#cv2.waitKey(0)
		#cv2.destroyAllWindows()
		transformed_param = get_transform(obj_param_single)
		print(transformed_param)
                result_param.append(transformed_param[0])
		result_param.append(transformed_param[1])
		result_param.append(transformed_param[2])
		result_param.append(orientation)
                obj_param_single.pose = result_param

                success = get_real_routine(obj_param_single)               
            
               #get_real_routine(list_)
               #  list_.append(property_)
	# result_list.append(list_)
	
	#deal_image = image
	#pub.publish(result_list)


def main():
  rospy.init_node('classfication', anonymous=True)
  
  # pub2 = rospy.Publisher('deal_img', Image, queue_size=1)
  rate = rospy.Rate(1)
  while not rospy.is_shutdown():
    rospy.Subscriber('/kinect2/hd/image_color_rect', Image, callback)
    
    rate.sleep()


if __name__ == '__main__':
    main()



