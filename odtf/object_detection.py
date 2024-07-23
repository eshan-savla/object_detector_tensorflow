#!/usr/bin/env python3

import os
from time import time

import numpy as np
import tensorflow as tf
from odtf.Orientation import Orientation


class ObjectDetection:

    def __init__(self,
                 saved_model_path: str,
                 label_map_path: str = None,
                 max_gpu_memory_fraction: float = 1.0,
                 logger=None):

        self.saved_model_path = saved_model_path
        self.label_map_path = label_map_path

        self.graph = None
        self.session = None
        self.labels = None

        self.max_gpu_memory_fraction = max(min(max_gpu_memory_fraction, 0), 1)

        self._logger = logger

        self._load_label_map()
        self._load_saved_model()

    def _load_label_map(self):

        self.labels = ["background"]

        if not self.label_map_path:
            print("No label map provided. Using default class names.")

        elif not os.path.exists(self.label_map_path):
            print((f"Label map path does not exist: {self.label_map_path}. "
                   "Using default class names."))

        else:
            with open(self.label_map_path) as label_map:
                self.labels.extend(label_map.read().splitlines())

    def _load_saved_model(self):

        self.model = tf.saved_model.load(self.saved_model_path)

        print("Starting initialization inference.")
        self._run_model(np.zeros([960, 1280, 3], dtype=np.uint8))
        print("Finished initialization inference.")

    def _run_model(self, image: np.ndarray):

        detection_dict = self.model(tf.expand_dims(image, 0))

        num = detection_dict["num_detections"][0].numpy().astype(np.int32)
        classes = detection_dict["detection_classes"][0].numpy().astype(
            np.uint8)
        boxes = detection_dict["detection_boxes"][0].numpy()
        scores = detection_dict["detection_scores"][0].numpy()
        masks = None

        if "detection_masks" in detection_dict:
            masks = detection_dict["detection_masks"][0].numpy()

        detections = []

        for index in range(num):
            class_name = (self.labels[classes[index]]
                          if classes[index] < len(self.labels)
                          else f"Class {classes[index]}")

            mask = None if masks is None else masks[index]

            detections.append({
                "class_id": classes[index],
                "class_name": class_name,
                "probability": scores[index],
                "bounding_box": boxes[index],
                "mask": mask
            })

        return detections

    def run(self,
            image: np.ndarray,
            roi: list = None,
            threshold: float = 0.5):

        box_scale = image.shape[:2]
        box_offset = (0, 0)

        if roi is not None:
            image = image[roi[0]:roi[2],
                          roi[1]:roi[3]]

            box_offset = (roi[0], roi[1])
        ori_obj = Orientation(image)
        start_time = time()

        detections = self._run_model(image)

        if self._logger is not None:
            self._logger.info(f"Inference took {time() - start_time:.3f} s")

        for detection in detections:
            detection["class_id"] = int(detection["class_id"])
            detection["probability"] = float(detection["probability"])
            detection["mask"] = np.asarray(detection["mask"] > threshold,dtype=np.uint8) if detection["mask"] is not None else None
            detection["bounding_box"] = [int(detection["bounding_box"][0] * box_scale[0] + box_offset[0]),
                                         int(detection["bounding_box"][1] * box_scale[1] + box_offset[1]),
                                         int(detection["bounding_box"][2] * box_scale[0] + box_offset[0]),
                                         int(detection["bounding_box"][3] * box_scale[1] + box_offset[1])]

            # x, y = np.argwhere(detection["mask"] > 0).sum(0) / detection["mask"].size
            # x, y = (x / detection["mask"].shape[1] * box_scale[1] + box_offset[1],
            #         y / detection["mask"].shape[0] * box_scale[0] + box_offset[0])
            x, y = (detection["bounding_box"][1] + (detection["bounding_box"][3] - detection["bounding_box"][1]) / 2,
                    detection["bounding_box"][0] + (detection["bounding_box"][2] - detection["bounding_box"][0]) / 2)

            detection["center"] = [float(x), float(y)]
            ori_obj.set_mask(detection["mask"], tuple(detection["bounding_box"]),(int(x), int(y)))
            detection["orientation"] = ori_obj.compute_orientation()

        return detections
