from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point
# from depthai_ros_msgs.msg import SpatialDetectionArray, SpatialDetection
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from message_filters import ApproximateTimeSynchronizer, Subscriber

import cv2
import cv_bridge

import numpy as np   
import os
import time


### ros packaged imports
from duck2_msgs.msg import FollowMeHumanList, FollowMeHuman, FollowMeGesture

from tiny_gestures.GestureTracker import KalmanGestureTracker, KalmanObject

#############################################################################################                  
class Colors:
    # Ultralytics color palette https://ultralytics.com/
    def __init__(self):
        # hex = matplotlib.colors.TABLEAU_COLORS.values()
        hex = ('FF3838', 'FF9D97', 'FF701F', 'FFB21D', 'CFD231', '48F90A', '92CC17', '3DDB86', '1A9334', '00D4BB',
               '2C99A8', '00C2FF', '344593', '6473FF', '0018EC', '8438FF', '520085', 'CB38FF', 'FF95C8', 'FF37C7')
        self.palette = [self.hex2rgb('#' + c) for c in hex]
        self.n = len(self.palette)
    def __call__(self, i, bgr=False):
        c = self.palette[int(i) % self.n]
        return (c[2], c[1], c[0]) if bgr else c
    @staticmethod
    def hex2rgb(h):  # rgb order (PIL)
        return tuple(int(h[1 + i:1 + i + 2], 16) for i in (0, 2, 4))
colors = Colors()  # create instance of colorgen
#############################################################################################                  
class OakDDetection:
    def __init__(self, cls, classname, center, size, conf, xyz, header):
        self.cls = cls
        self.classname = classname
        self.center = center
        self.size_x, self.size_y = size
        self.conf =  conf
        self.x, self.y, self.z = xyz
        self.header = header
        ### bounding box
        self.xmin, self.ymin, self.xmax, self.ymax = calculateRectEdges(center, size)
    def __repr__(self) -> str:
        return "%s (%s, %s) : %s" % (self.center, self.size_x, self.size_y, self.conf)
#############################################################################################
class Annotator:
    font = cv2.FONT_HERSHEY_TRIPLEX
    size = 0.75
    # textcolor = (0,255,0)
    @staticmethod
    def drawDetection(rgb_img, m:OakDDetection, extraStr=""):
        label = f'{m.cls} {m.classname} {m.conf:.2f}'
        if extraStr:
            label += " | %s" % extraStr
        classColor = colors(m.cls, True)
        x,y,z = (m.x, m.y, m.z)
        ### annotate rgb image rect and depth info
        center = m.center
        # centerx = center[0]
        # centery = center[1]
        p1, p2 = (int(m.xmin), int(m.ymin)), (int(m.xmax), int(m.ymax))
        cv2.rectangle(rgb_img, p1, p2, classColor, thickness=1, lineType=cv2.LINE_AA)
        texts = [
            label, 
            f"{m.conf:.2f}",
            f"X: {x:.1f}m",
            f"Y: {y:.1f}m",
            f"Z: {z:.1f}m",
        ]
        for text, ycoordoffset in zip(texts, [-2, 20, 40, 60, 80]):
            cv2.putText(rgb_img, text, (p1[0], p1[1] + ycoordoffset), 0, Annotator.size, classColor, thickness=2, lineType=cv2.LINE_AA)
        return rgb_img
    @staticmethod
    def drawTracks(rgb, objectDict):
        rgb_img = rgb.copy()
        obj : KalmanObject
        ### also draw hypotheses information on the image
        lines = ["TrackedObjetcs: %s" % len(objectDict.items())]
        for id, obj in objectDict.items():
            ratio = obj.calculateRatio()
            rgb_img = Annotator.drawDetection(rgb_img, obj, extraStr=f'{ratio:.2f}')
            lines.append(" - %s: %s" % (id, obj.classname))
        ### draw multiline
        for i, line in enumerate(lines):
            cv2.putText(rgb_img, line, (10, 30 + 30*i), 0, 1, (0,0,0), thickness=2, lineType=cv2.LINE_AA)
        return rgb_img
#############################################################################################                  
def centerFromRect(rect):
    p1, p2 = (int(rect[0]), int(rect[1])), (int(rect[2]), int(rect[3]))
    center = ((p1[0] + p2[0]) // 2, (p1[1] + p2[1]) // 2)
    return center
def calculateRectEdges(center, size):
    center_x, center_y = center
    width, height = size
    xmin = center_x - width / 2
    ymin = center_y - height / 2
    xmax = center_x + width / 2
    ymax = center_y + height / 2
    return xmin, ymin, xmax, ymax
#############################################################################################
class DetectionRosNode(Node):
    def __init__(self):
        super().__init__('inference_ros_node')
        #################################################
        ### Declare and get parameters
        # self.declare_parameter('model_config', 'config.yml')
        self.declare_parameter('visualize', False)
        self.declare_parameter('rgb_topic', '/color/image')
        self.declare_parameter('depth_topic', '/stereo/depth')
        self.declare_parameter('nn_topic', '/color/yolov4_Spatial_detections')
        self.declare_parameter('detection_score_treshold', 0.3)   
        self.declare_parameter('tracking_association_dist_threshold', 1.0)   
        # configPath = self.get_parameter('model_config').get_parameter_value().string_value
        self.visualize = self.get_parameter('visualize').get_parameter_value().bool_value
        rgb_topic = self.get_parameter('rgb_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        nn_topic = self.get_parameter('nn_topic').get_parameter_value().string_value
        self.detection_score_treshold = self.get_parameter('detection_score_treshold').get_parameter_value().double_value
        self.tracking_association_dist_threshold = self.get_parameter('tracking_association_dist_threshold').get_parameter_value().double_value
        #################################################
        ### Create publishers/subscribers for the two image topics
        self.human_gesture_pub = self.create_publisher(FollowMeHumanList, '/gestures', 1)
        if self.visualize:
            self.rgb_pub = self.create_publisher(Image, '~/rgb/labeled', 1)
            self.depth_pub = self.create_publisher(Image, '~/depth/colored', 1)
            self.tracks_pub = self.create_publisher(Image, '~/tracks/colored', 1)
        self.rgb_sub = Subscriber(self, Image, rgb_topic)
        self.depth_sub = Subscriber(self, Image, depth_topic)
        self.nn_sub = Subscriber(self, Detection3DArray, nn_topic)
        # self.nn_sub = self.create_subscription(TrackDetection2DArray, nn_topic, self.nn_callback, 1)
        ### Create the ApproximateTimeSynchronizer
        self.ts = ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub, self.nn_sub],
            queue_size=10,
            slop=0.1
        )
        ### Register the callback to be called when both images are received
        self.ts.registerCallback(self.image_callback)
        #################################################
        ### misc
        self.bridge = cv_bridge.CvBridge()
        self.tracker = KalmanGestureTracker(distance_threshold=self.tracking_association_dist_threshold) 
    def destroy_node(self):
        self.inference.stop()
        super().destroy_node()    
    def image_callback(self, rgb_msg, depth_msg, nn_msg):
        # self.get_logger().debug('Synchronized images received')
        rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
        if self.visualize:
            # target_height, target_width, c = rgb.shape[:3]
            # depth = cv2.resize(depth, (target_width, target_height))
            depth = cv2.convertScaleAbs(depth, alpha=(255.0/65535.0))
            ### Convert the single-channel 8-bit image to a 3-channel 8-bit image by replicating the gray values across the 3 channels
            depth = cv2.cvtColor(depth, cv2.COLOR_GRAY2BGR)
            if np.all(depth == 0):
                min_depth = 0  # Set a default minimum depth value when all elements are zero
            else:
                min_depth = np.percentile(depth[depth != 0], 1)
            max_depth = np.percentile(depth, 99)
            depthFrameColor = np.interp(depth, (min_depth, max_depth), (0, 255)).astype(np.uint8)
            depth = cv2.applyColorMap(depthFrameColor, cv2.COLORMAP_HOT)
        ### calculate proper rectangles inside rgb and depth from neural network output of oak
        detections = self.parseNNMsg(nn_msg)
        ### track gestures
        self.trackGestures(detections)
        ### debug draw detections
        if self.visualize:
            rgb_tracked = rgb.copy()
            for detec in detections:
                rgb = Annotator.drawDetection(rgb, detec)
            rgb_tracked = Annotator.drawTracks(rgb_tracked, self.tracker.get_all_objects(filter = lambda o: o.lifecycles > 10))
        ### publish output images containing last available model output
        if self.visualize:
            rgb_img_msg = self.bridge.cv2_to_imgmsg(rgb, "bgr8")
            depth_img_msg = self.bridge.cv2_to_imgmsg(depth, "bgr8")
            track_img_msg = self.bridge.cv2_to_imgmsg(rgb_tracked, "bgr8")
            self.rgb_pub.publish(rgb_img_msg)
            self.depth_pub.publish(depth_img_msg)
            self.tracks_pub.publish(track_img_msg)
        self.publishgestures()
    def parseNNMsg(self, msg : Detection3DArray) -> list[OakDDetection]:
        ### parse detections
        classes = ["ok","stop","palm","fist","like","peace","peace_inv"]
        extracted = []
        header = msg.header
        detection:Detection3D
        for i, detection in enumerate(msg.detections):
            ### HACK
            ### for some reason, depthai-ros/depthai_bridge/src/SpatialDetectionConverter.cpp -> SpatialDetectionConverter::toRosVisionMsg()
            ### does not support multi hypotheses, it only has result size of 1
            ### so whatever we do, WE get the first entry
            ##### OLD
            ### find highest score in result hypothesis
            # highestResult : ObjectHypothesisWithPose
            # highestResult = max(detection.results, key=lambda obj: obj.hypothesis.score)
            highestResult = detection.results[0]
            if highestResult.hypothesis.score > self.detection_score_treshold:
                id = int(highestResult.hypothesis.class_id)
                # print(i, id, highestResult.pose.pose.position)
                extracted.append(OakDDetection(id, classes[id],
                    (detection.bbox.center.position.x, detection.bbox.center.position.y),
                    (detection.bbox.size.x, detection.bbox.size.y), highestResult.hypothesis.score, 
                    (highestResult.pose.pose.position.x, highestResult.pose.pose.position.y, highestResult.pose.pose.position.z),
                    header)
                )
        return extracted
    def trackGestures(self, matches):
        ### create extra association func for differentiating classes
        extraAssociationFunc = lambda old, new: old.cls == new.cls
        ### update tracker with new measurements
        self.tracker.update(matches, f=extraAssociationFunc)
    def publishgestures(self):
        humans = []
        obj : KalmanObject
        for id, obj in self.tracker.get_all_objects().items():
            ratio = obj.calculateRatio()
            ### define publish criteria for gestures
            # if obj.lifecycles > 20 and ratio > 0.5:
            header = Header(stamp=self.get_clock().now().to_msg(), frame_id=obj.header.frame_id)
            gesture = FollowMeGesture(id=obj.cls, name=obj.classname, confidence=obj.conf)
            position = Point(x=obj.x, y=obj.y, z=obj.z)
            pose = Pose(position=position)
            poseStamped = PoseStamped(header=header, pose=pose)
            human = FollowMeHuman(header=obj.header, 
                                    is_tracked=True, tracked_time=obj.trackedTime, confidence=ratio, 
                                    pose=poseStamped, gesture=gesture)
            humans.append(human)
        msg = FollowMeHumanList(humans=humans)
        self.human_gesture_pub.publish(msg)

