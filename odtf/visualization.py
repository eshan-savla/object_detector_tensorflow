#!/usr/bin/env python3

import numpy as np
import cv2


class Visualization:

    def __init__(self,
                 font: int = cv2.FONT_HERSHEY_SIMPLEX,
                 font_scale: int = 1,
                 font_thickness: int = 2,
                 roi_color: tuple = (0, 0, 255),
                 detection_color: tuple = (0, 255, 0),
                 other_color: tuple = (255, 0, 0)):

        self.font = font
        self.font_scale = font_scale
        self.font_thickness = font_thickness
        self.roi_color = roi_color
        self.detection_color = detection_color
        self.other_color = other_color

    def draw_detections(self,
                        image: np.ndarray,
                        detections: list,
                        roi: list = None):

        if roi is not None:
            rect = self._roi2rect(roi)
            self._draw_rect(image, roi, self.roi_color)

        # Use for referenceing
        # image[image.shape[0] // 2 - 2:image.shape[0] // 2 + 3,
        #       image.shape[1] // 2 - 2:image.shape[1] // 2 + 3,
        #       :] = (0, 0, 255)

        for index, detection in enumerate(detections):
            color = (self.detection_color
                     if index == 0 else
                     (0, 255, 0))

            if detection["class_id"] == 1 or detection["class_id"] == 2 or detection["class_id"] == 6:
                color = self.other_color

            rect = self._roi2rect(detection["bounding_box"])
            text = f"{detection['class_name']} ({int(detection['probability'] * 100)} %)"

            self._draw_rect(image,
                            rect,
                            color,
                            text)

            cv2.circle(image,
                       (int(detection["center"][0]),
                        int(detection["center"][1])),
                       5,
                       (0, 255, 0),
                       2)

            if detection["mask"] is not None:
                image = self._draw_mask(image,
                                        detection["mask"],
                                        rect,
                                        color)

            if detection["eigens"] is not None:
                image = self.draw_orientation_v2(image, detection["eigens"])#detection["center"])
                
                # image = self.draw_minRectArea(image, dete)
                
        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    def _roi2rect(self, roi: list):

        return [(roi[1], roi[0]),
                (roi[1], roi[2]),
                (roi[3], roi[2]),
                (roi[3], roi[0])]

    def _draw_rect(self,
                   image: np.ndarray,
                   rect: list,
                   color=(255, 255, 255),
                   text: str = "",
                   mask: np.ndarray = None,
                   thickness: int = 2):

        try:
            image = cv2.drawContours(image,
                                     np.array([rect]),
                                     -1,
                                     color,
                                     thickness)

            if text:
                position = (rect[0][0] + 5,
                            rect[0][1] - 5)

                self._draw_text(image,
                                text,
                                position,
                                color)

        except Exception as e:
            print(e)

        return image

    def _draw_text(self,
                   image: np.ndarray,
                   text: str,
                   position: tuple,
                   color: tuple = (255, 255, 255)):

        point1 = (position[0] + 160, position[1])
        point2 = (position[0], position[1] - 35)

        cv2.rectangle(image, point1, point2, color, -1)

        cv2.putText(image,
                    text,
                    position,
                    self.font,
                    self.font_scale,
                    (255, 255, 255),
                    self.font_thickness)

    def _draw_mask(self,
                   image: np.ndarray,
                   mask: np.ndarray,
                   rect: list,
                   color=(255, 255, 255),
                   alpha=.2):

        try:
            # size = (rect[2][0] - rect[0][0],
            #         rect[2][1] - rect[0][1])

            # mask = cv2.resize(mask,
            #                   size)

            # mask_binary = np.zeros(mask.shape)
            # mask_binary[mask > .5] = 1
            mask_binary = mask

            mask = np.stack((mask_binary * color[0],
                             mask_binary * color[1],
                             mask_binary * color[2]),
                            axis=2)

            # image_mask = np.zeros(image.shape,
            #                       dtype=np.uint8)

            # position = (rect[0][0],
            #             rect[0][1])

            # image_mask[position[1]: position[1] + size[1],
                    #    position[0]: position[0] + size[0]] = mask
            image_mask = mask

            cv2.addWeighted(image_mask,
                            alpha,
                            image,
                            1,
                            0,
                            image)

        except Exception as e:
            print(e)

        return image
    
    def draw_orientation(self, image: np.ndarray, orientation: list, center: list):
        mean = orientation[0]
        eigenvectors = orientation[1]
        eigenvalues = orientation[2]
        center_pts = (int(center[0]), int(center[1]))
        mean = (int(mean[0]), int(mean[1]))
        cv2.circle(image, center_pts, 5, (0, 255, 0), 2)
        cv2.circle(image, center_pts, 5, (0, 0, 255), 2)
        for i in range(2):
            cv2.line(image, mean, (int(mean[0] + eigenvectors[i*2]*eigenvalues[i]*0.01), int(mean[1] + eigenvectors[i*2+1]*eigenvalues[i]*0.01)), (255, 0, 0), 2)
        return image
    
    
    def draw_orientation_v2(self, image: np.ndarray, orientation: list):
        print('XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXx')
        center = orientation[0]     
        print(f'Center : {center}')
        size = orientation[1]
        angle_rad = orientation[2][0] # extract the angle in radians
        
        angle = np.rad2deg(angle_rad)
        
        # Reconstruct rotated rectangle 
        rect = (tuple(center), tuple(size), angle)
        box = cv2.boxPoints(rect)
        box = np.int0(box)    
        
        # Draw the rotated rectangle
        cv2.polylines(image, [box], isClosed=True, color=(255, 0, 0), thickness=2)
                        
        # Calculate the longer side of the box
        side1 = np.linalg.norm(box[0] - box[1])
        side2 = np.linalg.norm(box[1] - box[2])
        longer_side = max(side1, side2)

        # Calculate the vector in the direction of the longer side
        vector = (box[1] - box[0]) if side1 == longer_side else (box[2] - box[1])

        vector = vector / np.linalg.norm(vector)

        # Calculate the center point of the minAreaRect
        center_pts = (int(center[0]), int(center[1]))

        # Calculate the end point of the vector
        end_point = (int(center[0] + longer_side * 0.6 * vector[0]),
                    int(center[1] + longer_side * 0.6 * vector[1]))

        # Draw rotated rectangle and longer vector
        cv2.polylines(image, [box], isClosed=True, color=(255, 140, 0), thickness=3)
        cv2.arrowedLine(image, center_pts, end_point, (255, 0, 0), 2)
        
        # Calculate the perpendicular vector (90 degrees counterclockwise)
        perp_vector = np.array([-vector[1], vector[0]])
        # Calculate the end point of the perpendicular vector
        perp_end_point = (int(center[0] + longer_side * 0.3 * perp_vector[0]),
                          int(center[1] + longer_side * 0.3 * perp_vector[1]))

        # Draw the perpendicular vector
        cv2.arrowedLine(image, center_pts, perp_end_point, (0, 255, 0), 2)
        # Draw the center point
        cv2.circle(image, center_pts, 5, (0, 255, 0), 2)
        cv2.circle(image, center_pts, 5, (0, 0, 255), 2)

        # Print the orientation
        orientation = orientation[2]  # angle
        print(f'Orientation : {orientation}')
        
        
        
        # 'Backlog'
        # rect = cv2.minAreaRect(c)
        # box = cv2.boxPoints(rect)
        # box = np.int0(box)
        # 
        # center, size, angle = rect
        # self.orientation = (list(center), list(size), [angle_rad])
        

        return image