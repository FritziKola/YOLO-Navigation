#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import Image
import cv2
from sensor_msgs.msg import Image
from rospy.numpy_msg import numpy_msg
import threading
import colorsys
import random
from std_msgs.msg import String


class rumex_detection:
    talker = rospy.Publisher("/test_detect", String, queue_size=10)
    def __init__(self):
        # Subscribe to camera topic
        rospy.Subscriber("/realsense/color/image_raw", numpy_msg(Image), self.callback_camera_feed)

        # Load YOLOv4 model
        #net_read = cv2.dnn.readNetFromDarknet("/home/ubuntu/catkin_ws/src/jackal_test/src/yolo/yolov4-coco.cfg", "/home/ubuntu/catkin_ws/src/jackal_test/src/yolo/yolov4-coco.weights") #normal config
        net_read = cv2.dnn.readNetFromDarknet("/home/ubuntu/catkin_ws/src/jackal_test/src/yolo/yolov4-rumex.cfg", "/home/ubuntu/catkin_ws/src/jackal_test/src/yolo/yolov4-field.weights")
        #net_read = cv2.dnn.readNet(cfg.YOLOV4_WEIGHTS, cfg.YOLOV4_MODEL_CFG)
        self.net = cv2.dnn_DetectionModel(net_read)
        self.net.setInputParams(scale=1 / 255, size=(416, 416), swapRB=True)
        # Load pred_classes
        self.classes_dict = self.read_class_names("/home/ubuntu/catkin_ws/src/jackal_test/src/yolo/rumex.names")
        self.classes_colors = self.generate_colors_for_classes(self.classes_dict)

        # Init tracker
        self.new_rumex_positions = list()
        #self.nav = navigation()

        # Start YOLO prediction thread
        yolo_pred_thread = threading.Thread(target=self.make_predictions)
        yolo_pred_thread.start()


    def callback_camera_feed(self, img_msg):
        self.img = np.frombuffer(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)
        #self.img = cv2.cvtColor(img_cam, cv2.COLOR_RGB2BGR)


    def make_predictions(self):
        while not rospy.is_shutdown():
            frame = self.img.copy()
            # Make predictions on image
            class_ids, scores, boxes = self.net.detect(frame, confThreshold=0.4, nmsThreshold=0.25)
            #class_ids, scores, boxes = self.only_detect_one_class(class_ids, scores, boxes)
            if len(boxes)>= 1:
                self.talker.publish("success")
            else:
                self.talker.publish("no")
            # Update tracker
            self.img_pred = frame
            # Draw predictions on the frame
            self.img_pred = self.draw_predictions(frame, class_ids, scores, boxes, self.classes_dict, self.classes_colors)
            #self.img_pred = self.draw_predictions_tracker(frame, self.classes_dict, self.classes_colors)
            #cv2.waitKey(1000)

    def only_detect_one_class(self, class_ids_old, scores_old, boxes_old):
        #print("old",class_ids_old, scores_old, boxes_old)
        class_ids = list()
        scores = list()
        boxes = list()
        for label, score, bbox in zip(class_ids_old, scores_old, boxes_old):
            if label == 32:
                print(label)
                class_ids.append(label)
                scores.append(score)
                boxes.append(bbox)
        #print("new",class_ids, scores, boxes)
        return class_ids, scores, boxes


    def draw_predictions(self, image, class_ids, scores, boxes, classes_names, classes_colors):
        img_height, img_width, _ = image.shape

        # draw every valid bbox with class name, score, object nr
        for label, score, bbox in zip(class_ids, scores, boxes):
            color = classes_colors[label]
            bbox_color = [color[2], color[1], color[0]]
            bbox_text = classes_names[label] + ": " + str(score)[:4]

            bbox_p1 = (bbox[0], bbox[1])  # top left corner (w,h)
            bbox_p2 = (bbox[0] + bbox[2], bbox[1] + bbox[3])  # bottom right corner (w,h)
            image = self.draw_bbox(image, bbox_p1, bbox_p2, bbox_text, bbox_color)

        return image

    def draw_predictions_tracker(self, image, classes_names, classes_colors):
        img_height, img_width, _ = image.shape

        for i, obj in list(self.tracker.objects.items()):
            if obj.disappeared == 0:

                # Draw bbox rect # BGR for cv2 methods
                #bbox_color = [classes_colors[obj.class_idx][2], classes_colors[obj.class_idx][1], classes_colors[obj.class_idx][0]]
                bbox_color = [classes_colors[0][2], classes_colors[0][1], classes_colors[0][0]]
                label = classes_names[obj.class_idx]
                # Prepare label text
                if obj.id_tracking == None:
                    # tracking_id = "x"+str(obj.frame_counter)
                    tracking_id = ""
                else:
                    tracking_id = str(obj.id_tracking+1)
                if tracking_id == "":
                    bbox_text = label + ": " + str("%.2f" % round(obj.score, 2))
                else:
                    bbox_text = label + ": " + str(
                        "%.2f" % round(obj.score, 2)) + "  ID: " + str(
                        tracking_id)

                image = self.draw_bbox(image, (obj.bbox[0], obj.bbox[1]), (obj.bbox[2], obj.bbox[3]), bbox_text, bbox_color)

        return image

    def draw_bbox(self, image, bbox_p1, bbox_p2, bbox_text, bbox_color):
        img_height, img_width, _ = image.shape

        # bbox_thick = int(0.6 * (img_height + img_width) / 600)
        # text_thick = 1 + round(int(0.6 * (img_height + img_width) / 600), 0)
        # fontScale = 1 + round(int(0.6 * (img_height + img_width) / 600), 0)
        bbox_thick = int(round(0.003 * img_height, 0))
        text_thick = int(round(0.002 * img_height, 0))
        font_scale = img_height * 0.001
        text_size = cv2.getTextSize(bbox_text, 0, font_scale, text_thick)[0]

        # Draw bbox
        cv2.rectangle(image, bbox_p1, bbox_p2, bbox_color, bbox_thick)

        # Draw filled rect (background of pred. label text and score)
        label_left_corner = (
        bbox_p1[0] - int(bbox_thick / 2), bbox_p1[1])  # the lower left and upper right corner of the rect
        label_right_corner = (label_left_corner[0] + text_size[0] + 5, label_left_corner[1] - text_size[1] - 3)
        label_left_corner, label_right_corner = self.correct_label_if_outside_img((img_height, img_width), label_left_corner,
                                                                             label_right_corner)
        cv2.rectangle(image, label_left_corner, ((label_right_corner[0]), (label_right_corner[1])), bbox_color,
                      -1)  # filled

        # Write text on filled rect
        cv2.putText(image, bbox_text, (label_left_corner[0], (label_left_corner[1] - 2)), cv2.FONT_HERSHEY_DUPLEX,
                    font_scale, (0, 0, 0), text_thick, lineType=cv2.LINE_AA)

        return image

    def correct_label_if_outside_img(self, img_shape, label_lower_left_corner, label_upper_right_corner):
        img_height, img_width = img_shape
        offset_width = 0
        offset_hight = 0

        # label outside of img on the right img edge
        if label_upper_right_corner[0] > img_width:
            offset_width = img_width - label_upper_right_corner[0]

        # label outside of img on the upper img edge
        if label_upper_right_corner[1] < 0:
            offset_hight = abs(label_upper_right_corner[1])

        label_lower_left_corner = (label_lower_left_corner[0] + offset_width, label_lower_left_corner[1] + offset_hight)
        label_upper_right_corner = (
        label_upper_right_corner[0] + offset_width, label_upper_right_corner[1] + offset_hight)
        return label_lower_left_corner, label_upper_right_corner

    def read_class_names(self, class_file_name):
        # copied from https://github.com/hunglc007/tensorflow-yolov4-tflite/blob/master/core/utils.py
        names = {}
        data = open(class_file_name, 'r')
        for ID, name in enumerate(data):
            names[ID] = name.strip('\n')
        return names

    def generate_colors_for_classes(self, classes):
        # based on https://github.com/hunglc007/tensorflow-yolov4-tflite/blob/master/core/utils.py
        num_classes = len(classes)
        hsv_tuples = [(1.0 * x / num_classes, 1., 1.) for x in range(num_classes)]
        colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
        colors = list(map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)), colors))
        random.seed(0)
        random.shuffle(colors)
        random.seed(None)
        return colors

    def update_tracker(self, image_shape, pred_class_ids, pred_scores, pred_boxes):
        img_height, img_width, _ = image_shape

        boxes = []
        scores = []
        class_ids = []

        for class_id, score, bbox in zip(pred_class_ids, pred_scores, pred_boxes):
            class_ids.append(class_id)
            scores.append(score)
            x_1 = bbox[0] # top left corner w
            y_1 = bbox[1] # top left corner h
            x_2 = bbox[0] + bbox[2] # bottom right corner w
            y_2 = bbox[1] + bbox[3] # bottom right corner h
            box = np.array([x_1, y_1, x_2, y_2])
            boxes.append(box.astype("int"))

        self.tracker.update(boxes, scores, class_ids)

        self.save_plant_pos()

    def save_plant_pos(self):
        if self.tracker.flag_plant:
            print("save_plant_pos")
            self.new_rumex_positions.append([100, 100])
            print("save", [100, 100])
            self.tracker.flag_plant = False

    def get_new_rumex_positions(self):
        temp_pos = self.new_rumex_positions
        self.new_rumex_positions = list()
        print("get", temp_pos)
        return temp_pos