#!/usr/bin/env python

import os
import sys
import cv2
import numpy as np
try:
    import tensorflow as tf
except ImportError:
    print("unable to import TensorFlow. Is it installed?")
    print("  sudo apt install python-pip")
    print("  sudo pip install tensorflow")
    sys.exit(1)

# ROS related imports
import rospy
from std_msgs.msg import String, Header, Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point

# Object detection module imports
import object_detection
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util

######### Set model here ############
MODEL_NAME = 'digits_recognition/up_down/third_training/model21936' #model130385'
# By default models are stored in data/models/
MODEL_PATH = os.path.join(os.path.dirname(sys.path[0]),'data','models' , MODEL_NAME)
# Path to frozen detection graph. This is the actual model that is used for the object detection.
PATH_TO_CKPT = MODEL_PATH + '/frozen_inference_graph.pb'
######### Set the label map file here ###########
LABEL_NAME = 'up_down_label_map.pbtxt' #'digits_label_map.pbtxt'# 'mscoco_label_map.pbtxt'
# By default label maps are stored in data/labels/
PATH_TO_LABELS = os.path.join(os.path.dirname(sys.path[0]),'data','labels', LABEL_NAME)
######### Set the number of classes here #########
NUM_CLASSES = 12


enable_target_button = False
target_button = Int8()


detection_graph = tf.Graph()
with detection_graph.as_default():
  od_graph_def = tf.GraphDef()
  with tf.gfile.GFile(PATH_TO_CKPT, 'rb') as fid:
    serialized_graph = fid.read()
    od_graph_def.ParseFromString(serialized_graph)
    tf.import_graph_def(od_graph_def, name='')

## Loading label map
# Label maps map indices to category names, so that when our convolution network predicts `5`,
# we know that this corresponds to `airplane`.  Here we use internal utility functions,
# but anything that returns a dictionary mapping integers to appropriate string labels would be fine
label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
category_index = label_map_util.create_category_index(categories)

# Setting the GPU options to use fraction of gpu that has been set
config = tf.ConfigProto()
# config.gpu_options.per_process_gpu_memory_fraction = GPU_FRACTION

# Detection
attempts = 2
current_attempts = 0
with detection_graph.as_default():
    # with tf.Session(graph=detection_graph,config=config) as sess:
    sess = tf.Session()
    #with tf.Session() as sess:
    class detector:

      def __init__(self):
        self.image_pub = rospy.Publisher("detected_objects",Image, queue_size=1)
        # self.object_pub = rospy.Publisher("objects", Detection2DArray, queue_size=1)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_cb, queue_size=1)#, buff_size=2**24)
        self.button_sub = rospy.Subscriber("/target_button", Int8, self.button_cb, queue_size=1)
        self.point_pub = rospy.Publisher("target_point", Point, queue_size=1)
        self.button_repeat_pub = rospy.Publisher("/target_button", Int8, queue_size=1)

      def button_cb(self, data):
        global enable_target_button
        enable_target_button = True
        global target_button
        target_button = data
        print("target button recieved: ", target_button)
        

      def image_cb(self, data):
        global enable_target_button
        # rospy.loginfo("Image callback")
        if(enable_target_button):
          # recibe comando
          # objArray = Detection2DArray()
          enable_target_button = False
          global target_button
          print("Entra a imagen")
          try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
          except CvBridgeError as e:
            print(e)
          image=cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

          # the array based representation of the image will be used later in order to prepare the
          # result image with boxes and labels on it.
          image_np = np.asarray(image)

          # print("check 1")
          # Expand dimensions since the model expects images to have shape: [1, None, None, 3]
          image_np_expanded = np.expand_dims(image_np, axis=0)
          image_tensor = detection_graph.get_tensor_by_name('image_tensor:0')
          # print("check 2")
          # Each box represents a part of the image where a particular object was detected.
          boxes = detection_graph.get_tensor_by_name('detection_boxes:0')

          # print("check 3")
          # Each score represent how level of confidence for each of the objects.
          # Score is shown on the result image, together with the class label.
          scores = detection_graph.get_tensor_by_name('detection_scores:0')
          classes = detection_graph.get_tensor_by_name('detection_classes:0')
          num_detections = detection_graph.get_tensor_by_name('num_detections:0')
          # print("check 4")
          (boxes, scores, classes, num_detections) = sess.run([boxes, scores, classes, num_detections],
              feed_dict={image_tensor: image_np_expanded})
          # print("check 5")

          
          objects=vis_util.visualize_boxes_and_labels_on_image_array(
              image,
              np.squeeze(boxes),
              np.squeeze(classes).astype(np.int32),
              np.squeeze(scores),
              category_index,
              max_boxes_to_draw=10,
              min_score_thresh=.4,
              use_normalized_coordinates=True,
              line_thickness=2)
          
          

          # objArray.detections =[]
          # objArray.header=data.header
          
          
          [image_rows, image_cols, image_ch] = image.shape
          
          button_found = False
          center_point = Point()
          for i in range(len(objects)):
            if(target_button.data == objects[i][0]):
              print("Button found")
              button_found = True
              [ymin, xmin, ymax, xmax] = objects[i][2]
              print("xmin: ", xmin)
              print("ymin: ", ymin)
              print("xmax: ", xmax)
              print("ymax: ", ymax)

              xmin *= image_cols
              xmax *= image_cols
              ymin *= image_rows
              ymax *= image_rows

              
              center_point.x = int((xmax + xmin)/2)
              center_point.y = int((ymax + ymin)/2)
              center_point.z = 0

              # center_x = int((xmax + xmin)/2)
              # center_y = int((ymax + ymin)/2)

              break

          img=cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
          image_out = Image()
          try:
            image_out = self.bridge.cv2_to_imgmsg(img,"bgr8")
          except CvBridgeError as e:
            print(e)
          image_out.header = data.header
          self.image_pub.publish(image_out)

          global attempts, current_attempts

          if(button_found):
            # publish point
            current_attempts = 0
            self.point_pub.publish(center_point)
          else:
            current_attempts += 1
            if(current_attempts < attempts):
              # print("Next attempt")
              # rospy.sleep(10.)
              print("Find button again")
              
              rospy.sleep(10.0)
              # self.button_repeat_pub.publish(target_button)
              enable_target_button = True
              rospy.sleep(2.0)
              
            else:
              print("Button not found")
              current_attempts = 0          

      def object_predict(self,object_data, header, image_np,image):
        image_height,image_width,channels = image.shape
        obj=Detection2D()
        obj_hypothesis= ObjectHypothesisWithPose()

        object_id=object_data[0]
        object_score=object_data[1]
        dimensions=object_data[2]

        obj.header=header
        obj_hypothesis.id = object_id
        obj_hypothesis.score = object_score
        obj.results.append(obj_hypothesis)
        obj.bbox.size_y = int((dimensions[2]-dimensions[0])*image_height)
        obj.bbox.size_x = int((dimensions[3]-dimensions[1] )*image_width)
        obj.bbox.center.x = int((dimensions[1] + dimensions [3])*image_height/2)
        obj.bbox.center.y = int((dimensions[0] + dimensions[2])*image_width/2)

        return obj

def main(args):
  rospy.init_node('detector_node')
  obj=detector()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("ShutDown")
  cv2.destroyAllWindows()

if __name__=='__main__':
  main(sys.argv)


