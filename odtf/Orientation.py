import cv2
from typing import Tuple
import numpy as np

class Orientation:
    def __init__(self, Image: np.ndarray, mask: np.ndarray = None, center: Tuple[int, int] = None):
        self.orientation: tuple = None
        self.image = Image
        if self.image.shape[2] == 3:
            self.image = cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)
        if mask is not None:
            self.set_mask(mask, center)
    
    def set_mask(self,mask: np.ndarray, center: Tuple[int, int] = None):
        self.mask = np.zeros(self.image.shape, dtype=np.uint8)
        mask_size = mask.shape
        x = center[1] - mask_size[1]//2
        y = center[0] - mask_size[0]//2
        x_end = x + mask_size[1]
        y_end = y + mask_size[0]
        self.mask[x:x_end, y:y_end] = mask
    
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
        self.orientation = (mean.astype(np.float32).flatten().tolist(), eigenvectors.astype(np.float32).flatten().tolist(), eigenvalues.astype(np.float32).flatten().tolist())
        return self.orientation
    

        
