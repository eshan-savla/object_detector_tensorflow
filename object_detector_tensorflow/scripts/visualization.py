import numpy as np
import cv2


class Visualization:

    def __init__(self,
                 font: int = cv2.FONT_HERSHEY_SIMPLEX,
                 font_scale: int = 1,
                 font_thickness: int = 2):

        self.font = font
        self.font_scale = font_scale
        self.font_thickness = font_thickness

        self.roi_color = (0, 255, 0)
        self.detection_color = (255, 0, 0)

    def draw_detections(self,
                        image: np.ndarray,
                        detections: list,
                        roi: list = None):

        if roi is not None:
            rect = self._roi2rect(roi)

            self._draw_rect(image, roi, self.roi_color)

        for detection in detections:
            rect = self._roi2rect(detection["bounding_box"])
            text = f"{detection['class_name']} ({int(detection['probability'] * 100)} %)"

            self._draw_rect(image, rect, self.detection_color, text)

        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    def _roi2rect(self, roi: list):

        return [(roi[1], roi[0]),
                (roi[1], roi[0] + roi[2]),
                (roi[1] + roi[3], roi[0] + roi[2]),
                (roi[1] + roi[3], roi[0])]

    def _draw_rect(self,
                   image: np.ndarray,
                   rect: list,
                   color=(255, 255, 255),
                   text: str = "",
                   thickness: int = 2):

        try:
            image = cv2.drawContours(image,
                                     np.array([rect]),
                                     -1,
                                     color,
                                     thickness)

            if text:
                position = (rect[0][0] + 5, rect[0][1] - 5)

                self._draw_text(image, text, position, color)

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
