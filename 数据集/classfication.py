#!/usr/bin/python
# -*- coding: UTF-8 -*

import cv2
import imutils
import numpy as np

import rospy
from robot_grasp.msg import DetectedObjectsArray
from robot_grasp.msg import DetectedObject
from sensor_msgs.msg import Image

result_list = []

def callback(img):

	image = cv2.imread(img)
	 
	# convert the resized image to grayscale, blur it slightly,
	# and threshold it
	gray = cv2.cvtColor(image.copy(), cv2.COLOR_BGR2GRAY)
	blurred = cv2.GaussianBlur(gray, (5, 5), 0)
	thresh = cv2.threshold(blurred, 110, 255, cv2.THRESH_BINARY)[1]
	# thresh = cv2.adaptiveThreshold(blurred,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,20,2)
	# kernel = np.ones((5,5), np.uint8)
	# thresh = cv2.morphologyEx(thresh, kernel, cv2.MORPH_CLOSE, kernel)
	cv2.imwrite("2.png", thresh)

	# find contours in the thresholded image 
	cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
	cnts = cnts[0] if imutils.is_cv2() else cnts[1]


	# loop over the contours
	for c in cnts:
		if c.size < 300:
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
                global result_list

		# Circle detection
		area = cv2.contourArea(c)
		(x,y),radius = cv2.minEnclosingCircle(c)
		center = (int(x),int(y))
		radius = int(radius)
		circle_area = np.pi*radius*radius
		# print(area/circle_area)
		if area/circle_area > 0.75:  
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
		    if max_val-min_val > 150:  # 受光照影响，差值最大为cake
		         name = 'cake'
		         cv2.putText(image, 'cake', center, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
		    elif mean_val > 160:  # 受光照影响，大值为lemon，小值为orange
		         name = 'lemon'  
		         cv2.putText(image, 'lemon', center, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
		    else:
		         name = 'orange'
		         cv2.putText(image, 'orange', center, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

		# Rectangle detection
		if property_['circle'] == 0:
		    rect = cv2.minAreaRect(c)  # top-left corner(x,y), (width, height), angle of rotation
		    rectangle_area = rect[1][0]*rect[1][1]
		    # print(area/rectangle_area)
		    aspect_ratio = rect[1][0]/rect[1][1] if rect[1][0] > rect[1][1] else rect[1][1]/rect[1][0]
		    # print aspect_ratio
		    if area/rectangle_area > 0.9:
		        box = cv2.boxPoints(rect)
		        box = np.int0(box)
		        image = cv2.drawContours(image, [box], 0, (255,0,0), 2)
		        name = 'milk box'
		        cv2.putText(image, 'milk box', center, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
		        property_['rectangle'] = 1
		        # 旋转角度θ是水平轴（x轴）逆时针旋转，与碰到的矩形的第一条边的夹角。并且这个边的边长是width，另一条边边长是height。也就是说，在这里，width与height不是按照长短来定义的。相对于x轴，逆时针旋转角度为负，顺时针旋转角度为正。在这里，θ∈（-90度，0]
		        orientation = rect[2] if rect[1][0] > rect[1][1] else rect[2]-90  # 如果宽大于长，则旋转角再减去90度，得到的orientation为沿水平轴逆时针旋转的角度值（负值）
		        print("milk box %f" % orientation)
		    if aspect_ratio > 2:
		        ellipse = cv2.fitEllipse(c)
		        image = cv2.ellipse(image,ellipse,(255,0,0),2)
		        name = 'sausage'
		        cv2.putText(image, 'sausage', center, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
		        property_['eclipse'] = 1
		        orientation = rect[2] if rect[1][0] > rect[1][1] else rect[2]-90  
		        print("sausage %f" % orientation)

		# other detections
		if name == ' ':
		        mask = np.zeros(gray.shape, np.uint8)
		        cv2.drawContours(mask,[c],0,255,-1)
		        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(gray,mask = mask)
		        perimeter = cv2.arcLength(c,True)
		        if max_val - min_val > 200 and perimeter > 800 and area > 40000: # 受光照影响
		            # print(max_val - min_val)
		            name = 'bear'
		            cv2.putText(image, 'bear', center, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
		            rect = cv2.minAreaRect(c) 
		            orientation = rect[2] if rect[1][0] > rect[1][1] else rect[2]-90
		            print("bear %f" % orientation)
		            # print(area, perimeter)
		        elif area > 40000:
		            # print(area)
		            name = 'hippo'
		            cv2.putText(image, 'hippo', center, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
		            rect = cv2.minAreaRect(c) 
		            orientation = rect[2] if rect[1][0] > rect[1][1] else rect[2]-90
		            print("hippo %f" % orientation)
		        elif area < 20000:
		            name = 'ball'
		            # ball recovery
		            (x,y),radius = cv2.minEnclosingCircle(c)
		            center = (int(x),int(y))
		            radius = int(radius)
		            img = cv2.circle(image,center,radius,(255,0,0),2)
		            position = [x, y]  # more accurate
		            property_['circle'] = 1
		            cv2.putText(image, 'ball', center, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
		        elif perimeter > 1100:
		            # print(perimeter)
		            name = 'bag'
		            cv2.putText(image, 'bag', center, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
		            rect = cv2.minAreaRect(c) 
		            orientation = rect[2] if rect[1][0] > rect[1][1] else rect[2]-90 
		            print("bag %f" % orientation)
		        else:
		            name = 'cream'
		            cv2.putText(image, 'cream', center, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
		            rect = cv2.minAreaRect(c) 
		            orientation = rect[2] if rect[1][0] > rect[1][1] else rect[2]-90
		            print("cream %f" % orientation)
	       
		list_.append(name)
		list_.append([position, orientation])
	      #  list_.append(property_)
		result_list.append(list_)
		# multiply the contour (x, y)-coordinates by the resize ratio,
		# then draw the contours and the name of the shape on the image
		c = c.astype("int")
		cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
		# show the output image
		# cv2.imshow("Image", image)
		# cv2.waitKey(0)
	#for list_ in result_list:
	#    print(list_)
  
	cv2.imwrite("Image.png", image)


def main():
  rospy.init_node('classfication', anonymous=True)
  pub = rospy.Publisher('pose', DetectedObjectsArray, queue_size=1)
  rate = rospy.Rate(1)
  while not rospy.is_shutdown():
    rospy.Subscriber('image_raw', Image, callback)
    pub.publish(result_list) 
    rate.sleep()


if __name__ == '__main__':
    main()



