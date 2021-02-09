import os

import numpy as np
import tensorflow as tf


class ObjectDetection:

    def __init__(self,
                 saved_model_path: str,
                 label_map_path: str = None,
                 max_gpu_memory_fraction: float = 1.0):

        self.saved_model_path = saved_model_path
        self.label_map_path = label_map_path

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
        #self.model = tf.saved_model.load(self.saved_model_path)
        #self.model = tf.compat.v2.saved_model.load(self.saved_model_path)
        self.model = tf.compat.v1.saved_model.loader.load(self.saved_model_path)

        print("Starting initialization inference.")
        # self._run_model(np.zeros([960, 1280, 3], dtype=np.uint8))
        print("Finished initialization inference.")

    def _run_model(self, image: np.ndarray):

        detection_dict = self.model([image])

        num = detection_dict['num_detections'][0].numpy().astype(np.int32)
        classes = detection_dict['detection_classes'][0].numpy().astype(
            np.uint8)
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
            roi: list = None):

        box_scale = image.shape[:2]
        box_offset = (0, 0)

        if roi is not None:
            image = image[roi[0]:roi[2],
                          roi[1]:roi[3]]

            box_offset = (roi[0], roi[1])

        raw_detections = self._run_model(image)

        for detection in raw_detections:
            detection["class_id"] = int(detection["class_id"])
            detection["probability"] = float(detection["probability"])

            box = [int(detection["bounding_box"][0] * box_scale[0] + box_offset[0]),
                   int(detection["bounding_box"][1] * box_scale[1] + box_offset[1]),
                   int(detection["bounding_box"][2] * box_scale[0] + box_offset[0]),
                   int(detection["bounding_box"][3] * box_scale[1] + box_offset[1])]

            detection["bounding_box"] = box

        return raw_detections
