#!/usr/bin/env python3
 
import rospy
import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage


def non_maximum_supression(bboxes, threshold=0.3):
    
    bboxes = sorted(bboxes, key=lambda detections: detections[3],
            reverse=True)
    new_bboxes=[]
    
    new_bboxes.append(bboxes[0])
    
    bboxes.pop(0)

    for _, bbox in enumerate(bboxes):

        for new_bbox in new_bboxes:

            x1_tl = bbox[0]
            x2_tl = new_bbox[0]
            x1_br = bbox[0] + bbox[2]
            x2_br = new_bbox[0] + new_bbox[2]
            y1_tl = bbox[1]
            y2_tl = new_bbox[1]
            y1_br = bbox[1] + bbox[3]
            y2_br = new_bbox[1] + new_bbox[3]
            
            x_overlap = max(0, min(x1_br, x2_br)-max(x1_tl, x2_tl))
            y_overlap = max(0, min(y1_br, y2_br)-max(y1_tl, y2_tl))
            overlap_area = x_overlap * y_overlap
            
            area_1 = bbox[2] * new_bbox[3]
            area_2 = new_bbox[2] * new_bbox[3]
            
            total_area = area_1 + area_2 - overlap_area

            overlap_area = overlap_area / float(total_area)

            if overlap_area < threshold:
                
                new_bboxes.append(bbox)

    return new_bboxes



class PEDESDetector:
    def __init__(self):
   
        self.rate = rospy.Rate(20)
    
        self.image_sub = rospy.Subscriber("/image_jpeg/compressed", CompressedImage, self.callback)

        self.pedes_detector = cv2.HOGDescriptor()                              
        self.pedes_detector.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        

    def callback(self, msg):

        self.rate.sleep()

        np_arr = np.frombuffer(msg.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        img_gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)

        (rects_temp, _) = self.pedes_detector.detectMultiScale(img_gray, winStride=(4, 4), padding=(8, 8), scale=8)
        
        if len(rects_temp) != 0:
            
            rects = non_maximum_supression(rects_temp)

            for (x,y,w,h) in rects:

                cv2.rectangle(img_bgr, (x,y),(x+w,y+h),(0,255,255), 2)

        cv2.imshow("Image window", img_bgr)
        cv2.waitKey(1) 


if __name__ == '__main__':

    rospy.init_node('pedes_detector', anonymous=True)

    pedes_detector = PEDESDetector()

    rospy.spin() 