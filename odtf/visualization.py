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
                     (0, 0, 255))

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
            size = (rect[2][0] - rect[0][0],
                    rect[2][1] - rect[0][1])

            mask = cv2.resize(mask,
                              size)

            # mask_binary = np.zeros(mask.shape)
            # mask_binary[mask > .5] = 1
            mask_binary = mask

            mask = np.stack((mask_binary * color[0],
                             mask_binary * color[1],
                             mask_binary * color[2]),
                            axis=2)

            image_mask = np.zeros(image.shape,
                                  dtype=np.uint8)

            position = (rect[0][0],
                        rect[0][1])

            image_mask[position[1]: position[1] + size[1],
                       position[0]: position[0] + size[0]] = mask

            cv2.addWeighted(image_mask,
                            alpha,
                            image,
                            1,
                            0,
                            image)

        except Exception as e:
            print(e)

        return image
