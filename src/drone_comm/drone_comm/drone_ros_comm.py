#!/usr/bin/env python3

import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

from drone_detection_interface.msg import DronePid, DroneStatus, HailoDetection
from drone_detection_interface.srv import ActiveStatus, SpeedStatus, StartSomething, GainPid, MovementType
from std_msgs.msg import Int64

from typing import Callable
from functools import partial

class Battery(Node): # Ready
    def __init__(self, get_battery_method: Callable[[], tuple[bool, int]]):
        super().__init__("get_battery_publisher")
        self.cb = MutuallyExclusiveCallbackGroup()
        self.get_battery_method = get_battery_method
        self.publisher_ = self.create_publisher(Int64, "get_battery", 10, callback_group=self.cb)
        self.timer_ = self.create_timer(5.0, self.battery_callback, callback_group=self.cb)
        
    def battery_callback(self):
        battery = Int64()
        verify, battery.data = self.get_battery_method()

        if verify:
            self.publisher_.publish(battery)

class GeneralDataStatus(Node): 
    def __init__(self, get_status_method):
        super().__init__("get_status_publisher")
        self.cb = MutuallyExclusiveCallbackGroup()
        self.get_status_method = get_status_method
        self.publisher_ = self.create_publisher(DroneStatus, "get_status", 10, callback_group=self.cb)
        self.timer_ = self.create_timer(3.5, self.status_callback, callback_group = self.cb)
        self.t_start = True
        self.get_logger().info("Get Status Publisher has been activated")
    
    def status_callback(self):
        verify, status_list = self.get_status_method()

        if verify:
            status = DroneStatus()
            status.pitch = status_list[0]
            status.roll = status_list[1]
            status.yaw = status_list [2]
            status.templ = status_list [3]
            status.temph = status_list [4]
            status.tof = status_list [5]
            status.h = status_list [6]
            status.bat = status_list [7]
            status.baro = status_list [8]
            status.time = status_list [9]
            self.publisher_.publish(status)

    def start_timer(self):
        if not self.t_start:
            self.timer_ = self.create_timer(3.5, self.status_callback, callback_group = self.cb)
            self.t_start = True
            self.get_logger().info(f"get status publisher timer has been started")

    def stop_timer(self):
        if self.t_start:
            self.destroy_timer(self.t_start)
            self.t_start = False
            self.get_logger().info(f"get status publisher timer has been stoped")

class GetPid(Node): # create 5 nodes
    def __init__(self, get_pid_method, node_name, topic_name, identity):
        super().__init__(node_name)
        self.cb = ReentrantCallbackGroup()
        self.get_pid_method = get_pid_method
        self.identity = identity
        self.publisher_ = self.create_publisher(DronePid, topic_name, 10, callback_group = self.cb)
        #self.timer_ = self.create_timer(0.6, self.get_pid_callback)
        self.t_start = False
        #self.get_logger().info(f"{identity} Has Been Started")

    def get_pid_callback(self):
        msg = DronePid()
        verify, data = self.get_pid_method(self.identity)
        if verify:
            msg.system_variable, msg.setpoint, msg.error, msg.output = data
            self.publisher_.publish(msg)
        #self.get_logger().info(f"\n{msg.system_variable} \n{msg.setpoint} \n{msg.error} \n{msg.output}")
    
    def start_timer(self):
        if not self.t_start:
            self.timer_ = self.create_timer(0.6, self.get_pid_callback)
            self.t_start = True
            self.get_logger().info(f"{self.identity} timer has been started")    
    def stop_timer(self):
        if self.t_start:
            self.destroy_timer(self.t_start)
            self.t_start = False
            self.get_logger().info(f"{self.identity} timer has been stoped")

class GetImg(Node): # Ready
    def __init__(self, get_frame):
        super().__init__("get_img_publisher")
        self.get_frame = get_frame
        self.bridge_object = CvBridge()
        self.cb = MutuallyExclusiveCallbackGroup()
        self.publisher_ = self.create_publisher(Image, "camara_img", 20, callback_group=self.cb)
        self.timer_ = self.create_timer(0.1, self.camara_callback)
        self.timer_running = True

    def camara_callback(self):
        success, frame = self.get_frame()
        if success:
            img_msg = self.bridge_object.cv2_to_imgmsg(frame)
            self.publisher_.publish(img_msg)
    
    def stop_timer(self):
        if self.timer_running:
            self.destroy_timer(self.timer_)
            self.timer_running = False
    
    def start_timer(self):
        if not self.timer_running:
            self.timer_ = self.create_timer(0.1, self.camara_callback, callback_group=self.cb)
            self.timer_running = True

class Bbox(Node):
    def __init__(self, send_bbox):
        super().__init__("bbox_subscriber")
        self.send_bbox = send_bbox
        self.cb = MutuallyExclusiveCallbackGroup()
        self.subscriber = self.create_subscription(HailoDetection, 
                                                   "get_bbox", 
                                                   self.get_bbox, 
                                                   10, callback_group=self.cb)
    
    def get_bbox(self, msg:HailoDetection):
        tuple_bbox = (
                msg.x_min,
                msg.y_min,
                msg.x_max,
                msg.y_max,
                msg.width,
                msg.height
                )
        bool_detection = msg.detection
        self.send_bbox(tuple_bbox, bool_detection)
        

# ************************************************** #
class SpeedData(Node):
    def __init__(self, speed_data):
        super().__init__("speed_server")
        self.cb = ReentrantCallbackGroup()
        self.speed_data = speed_data
        self.server_ = self.create_service(SpeedStatus, "speed_data", self.speed_data_callback, callback_group = self.cb)

    def speed_data_callback(self, request: SpeedStatus.Request, response: SpeedStatus.Response):
        response.verify = self.speed_data(request.speed)
        return response

class StartData(Node):
    def __init__(self, start_data_method):
        super().__init__("start_data_server")
        self.cb = MutuallyExclusiveCallbackGroup()
        self.start_data_method = start_data_method
        self.server_ = self.create_service(StartSomething, "start_data", self.start_data_callback, callback_group = self.cb)
        self._logger.info("start data server has been started")
        
    def start_data_callback(self, request: StartSomething.Request, response: StartSomething.Response):
        response.verify = self.start_data_method(request.start)
        return response

class DroneMotion(Node):
    def __init__(self, motion_method):
        super().__init__("drone_motion_server")
        self.cb = MutuallyExclusiveCallbackGroup()
        self.motion_method = motion_method
        self.server_ = self.create_service(MovementType, "drone_motion", self.motion_callback, callback_group = self.cb)

    def motion_callback(self, request:MovementType.Request, response:MovementType.Response):
        response.verify = self.motion_method(request.movement, request.activation)
        return response

class PidGainServer(Node):
    def __init__(self, gain_method: Callable[[str, float], bool], node_name:str, srv_name:str):
        super().__init__(node_name)
        self.cb = ReentrantCallbackGroup()
        self.gain_method = gain_method
        self.server_ = self.create_service(GainPid, srv_name, self.pid_gain_callback, callback_group = self.cb)
        self.get_logger().info(f"{node_name} has been started")

    def pid_gain_callback(self, request: GainPid.Request, response: GainPid.Response) -> GainPid.Response:
        try:
            response.verify = self.gain_method(request.identity, request.gain)
        except Exception as e:
            self.get_logger().error(f"Gain method failed: {e}")
            response.verify = False  # o algún valor por defecto
        return response
###
class StartDetection(Node):
    def __init__(self):
        super().__init__("start_data_det_clt")
        self.cb_grup = ReentrantCallbackGroup()
        self.client_ = self.create_client(StartSomething, "start_data_detection", callback_group = self.cb_grup)
        self.get_logger().info("Start Data Detection Client has been started")

    def call_start_detection(self, data):
        while not self.client_.wait_for_service(0.5):
            self.get_logger().warn("Waiting for start_data_detection server...")
        
        #Using the interfac for request
        request = StartSomething.Request()
        request.start = data

        future = self.client_.call_async(request)
        future.add_done_callback(
                    partial (self.callback_start_detection, request=request))
    
    def callback_start_detection(self, future, request):
        self.get_logger().info(f'{future.result().verify}')
