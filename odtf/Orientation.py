from ast import List
import cv2
from typing import Tuple
import numpy as np
from sympy import Quaternion

class Orientation:
    def __init__(self, Image: np.ndarray, mask: cv2.typing.MatLike | None = None, bounding_box: Tuple[int, ...] | None = None):
        self.orientation: tuple = None
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
        sz = len(c)
        data_pts = np.empty((sz, 2), dtype=np.float64)
        for i in range(data_pts.shape[0]):
            data_pts[i, 0] = c[i,0,0]
            data_pts[i, 1] = c[i,0,1]
        
        mean = np.empty((0))
        mean, eigenvectors, eigenvalues = cv2.PCACompute2(data_pts, mean)
        self.quaternion = self.to_quaternion(eigenvectors)
        self.orientation = (mean.astype(np.float32).flatten().tolist(), eigenvectors.astype(np.float32).flatten().tolist(), eigenvalues.astype(np.float32).flatten().tolist())
        return self.orientation
    
    @staticmethod
    def to_quaternion(eigenvectors) -> Tuple[float, float, float, float]:
        angle = np.arctan2(-eigenvectors[0,1], eigenvectors[0,0])
        if angle > 0:
            angle -= np.pi
        return (0.0, 0.0, np.sin(angle/2), np.cos(angle/2))
    

        
