from ast import List
import re
from turtle import width
import cv2
from typing import Tuple
import numpy as np
from sympy import Quaternion

from scipy.spatial.transform import Rotation



class Orientation:
    def __init__(self, Image: np.ndarray, mask: cv2.typing.MatLike | None = None, bounding_box: Tuple[int, ...] | None = None):
        self.orientation = None
        self.quaternion: tuple = None
        self.image = Image
        if self.image.shape[2] == 3:
            self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        if mask is not None and bounding_box is not None:
            self.set_mask(mask, bounding_box)
     
    def set_mask(self,mask: cv2.typing.MatLike, bounding_box: Tuple[int, ...]):
        self.mask = np.zeros(self.image.shape, dtype=np.uint8)
        self.mask[bounding_box[0]:bounding_box[2], bounding_box[1]:bounding_box[3]] = mask
    
    def compute_orientation(self) -> Tuple[list, list, list]:
        img = self.image
        if self.mask is None:
            img = cv2.adaptiveThreshold(self.image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)
        else:
            img = cv2.bitwise_and(self.image, self.image, mask=self.mask)
        contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) == 0:
            return None
        c = max(contours, key=cv2.contourArea)
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        
        _, _, angle = rect # center, size, angle
        
        if rect[1][0] > rect[1][1]:
            angle = angle + 90
            
        angle = 180 - angle    
        angle_quat_rad = np.deg2rad(angle)
              
        self.quaternion = self.to_quaternion(angle_quat_rad)
      
        self.orientation = rect
                
        return self.orientation
    
    @staticmethod
    def to_quaternion(angle_rad: float) -> Tuple[float, float, float, float]:
        
        quat = Rotation.from_quat([0, 0, np.sin(angle_rad / 2), np.cos(angle_rad / 2)])
        
        deg = quat.as_euler('zyx', degrees=False)[0]
    
        # if Rotation around z >= pi or z <= pi, then the quaternion is mirrored
        if deg >= np.pi/2 and deg <= 3*np.pi/2:
            deg = deg - Rotation.from_euler('z', np.pi).as_euler('zyx')[0]
        elif deg <= -np.pi/2 and deg >= -3*np.pi/2:
            deg = deg + np.pi
        
        quat = Rotation.from_euler('z', deg)
        
        quaternion = quat.as_quat()
        quaternion = tuple(quaternion)
        
        return quaternion

        
