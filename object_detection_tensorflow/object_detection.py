import os

import numpy as np
import tensorflow as tf

from sensor_msgs.msg import RegionOfInterest

from object_detection_tensorflow.msg import Detection


class ObjectDetection:

    def __init__(self,
                 saved_model_path: str,
                 label_map_path: str = None,
                 max_gpu_memory_fraction: float = 1):

        self.saved_model_path = saved_model_path
        self.label_map_path = (label_map_path
                               if label_map_path is not None else None)

        self.graph = None
        self.session = None
        self.labels = None

        self.max_gpu_memory_fraction = max(min(max_gpu_memory_fraction, 0), 1)

        self._load_label_map()
        self._load_saved_model()

    def _load_label_map(self):

        self.labels = ["background"]

        if not self.label_map_path:
            print("No label map provided. Using default class names.")

        else:
            with open(self.label_map_path) as label_map:
                self.labels.extend(label_map.read().splitlines())

    def _load_saved_model(self):

        self.model = tf.saved_model.load(self.saved_model_path)

        print("Starting initialization inference.")
        # self._run_model(np.zeros([1, 960, 1280, 3], dtype=np.uint8))
        print("Finished initialization inference.")

    def _run_model(self, image: np.ndarray):

        detection_dict = self.model(image)

        num = detection_dict['num_detections'][0].numpy().astype(np.int32)
        classes = detection_dict['detection_classes'][0].numpy().astype(np.uint8)
        boxes = detection_dict['detection_boxes'][0].numpy()
        scores = detection_dict['detection_scores'][0].numpy()

        detections = []

        for index in range(num):
            class_name = (self.labels[classes[index]]
                          if classes[index] < len(self.labels)
                          else f"Class {classes[index]}")

            detections.append({
                "class_id": classes[index],
                "class_name": class_name,
                "probability": scores[index],
                "bounding_box": boxes[index],
            })

        return detections

    def run(self,
            image: np.ndarray,
            roi: RegionOfInterest = None):

        box_scale = image.shape[:2]
        box_offset = (0, 0)

        if roi is not None:
            image = image[roi.y_offset:roi.y_offset + roi.height,
                          roi.x_offset:roi.x_offset + roi.width]

            box_offset = (roi.y_offset, roi.x_offset)

        raw_detections = self._run_model(image)

        for detection in raw_detections:
            box = np.multiply(detection["bounding_box"],
                              box_scale).astype(np.uint16)

            bounding_box = {
                "y_offset": int(box[0] * box_scale[0] + box_offset[0]),
                "x_offset": int(box[1] * box_scale[0] + box_offset[1]),
                "height": int((box[2] - box[0]) * box_scale[0]),
                "width": int((box[3] - box[1]) * box_scale[0])
            }

            detection["bounding_box"] = bounding_box

        return raw_detections

        # detections = []

        # for detection in raw_detections:
        #     box = np.multiply(detection["bounding_box"],
        #                       box_scale).astype(np.uint16)

        #     bounding_box = RegionOfInterest(
        #         y_offset=int(box[0] * box_scale[0] + box_offset[0]),
        #         x_offset=int(box[1] * box_scale[0] + box_offset[1]),
        #         height=int((box[2] - box[0]) * box_scale[0]),
        #         width=int((box[3] - box[1]) * box_scale[0]))

        #     detections.append(Detection(
        #         class_id=detection["class_id"],
        #         class_name=detection["class_name"],
        #         probability=detection["probability"],
        #         bounding_box=bounding_box))

        # return detections
