#!/usr/bin/env python3

from rclpy.node import Node
import rclpy
from std_msgs.msg import Int64
from drone_detection_interface.msg import DronePid, DroneStatus
from drone_detection_interface.srv import SpeedStatus, StartSomething, ActiveStatus, GainPid, MovementType
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from typing import Callable
from functools import partial

# Topics
class GetBattery(Node): # Ready
    def __init__(self, change_battery_method: Callable[[int], None]):
        super().__init__("get_battery_subscriber") 
        # Creating a suscription node
        self.change_battery_method = change_battery_method
        self.cb_group = MutuallyExclusiveCallbackGroup()
        self.subscriber_ = self.create_subscription(Int64, 
                                                    "get_battery", 
                                                    self.callback_get_battery, 
                                                    10, callback_group = self.cb_group)
        self.get_logger().info("get battery subscriber node activated")

    def callback_get_battery(self, msg: Int64):
        #self.get_logger().info(f'{msg.data}')
        self.change_battery_method(msg.data)

class GetStatus(Node):
    def __init__(self, change_status_method: Callable[[int], None]):
        super().__init__("get_status_suscriber") 
        self.change_status_method = change_status_method
        self.cb_group = MutuallyExclusiveCallbackGroup()
        self.subscriber_ = self.create_subscription(DroneStatus, "get_status", self.callback_get_status, 10, callback_group = self.cb_group)
        self.get_logger().info("get status subscriber node activated")

    def callback_get_status(self, msg: DroneStatus):
        msg_list = [
            msg.pitch+"°",
            msg.roll+"°",
            msg.yaw+"°",
            msg.templ+"°C",
            msg.temph+"°C",
            msg.tof+"cm",
            msg.h+"cm",
            msg.bat+"%",
            msg.baro+"m",
            msg.time+"s"
        ]

        self.change_status_method(msg_list)

class GetPid(Node):
    def __init__(self, update_graph: Callable[[], None], node_name, topic_name, identity):
        super().__init__(node_name)
        self.update_graph = update_graph
        self.identity = identity
        self.cb_group = MutuallyExclusiveCallbackGroup()
        self.subscriber_ = self.create_subscription(DronePid, topic_name, self.callback_update_graph, 10, callback_group = self.cb_group)
        self.get_logger().info("get pid subscriber node activated")

    def callback_update_graph(self, msg: DronePid):
        self.update_graph(self.identity, (msg.system_variable, msg.setpoint, msg.error, msg.output))

class SubscriberGetImageNode (Node):
    def __init__(self, show_img):
        super().__init__('get_image_subscriber')
        self.show = show_img
        self.bridge_object_ = CvBridge()
        self.cb_group = ReentrantCallbackGroup()
        self.subscription_ = self.create_subscription(Image, 'camara_img',self.listener_callback, 20, callback_group=self.cb_group)
        self.get_logger().info("Subscriber Get Image Has Been Started")

    def listener_callback (self, image_message):
        img = self.bridge_object_.imgmsg_to_cv2(image_message)
        self.show(img)  

# Server (client)
class DroneSpeedClient(Node):
    def __init__(self):
        super().__init__("speed_client")
        self.cb_group = ReentrantCallbackGroup()
        self.client_ = self.create_client(SpeedStatus, "speed_data", callback_group = self.cb_group)
    
    def call_speed_sever(self, speed):
        while not self.client_.wait_for_service(0.5):
            self.get_logger().warn("Waiting for speed_server...")
        
        # Using the interface for request
        request = SpeedStatus.Request()
        request.speed = speed

        future = self.client_.call_async(request)
        future.add_done_callback(
            partial(self.callback_speed, request=request))

    def callback_speed(self, future, request):
        self.get_logger().info(f'{future.result().verify}')

# 
class StartData(Node):
    def __init__(self):
        super().__init__("start_data_client")
        self.cb_grup = ReentrantCallbackGroup()
        self.client_ = self.create_client(StartSomething, "start_data", callback_group = self.cb_grup)
        self.get_logger().info("Start Data Client has been started")

    def call_start_data(self, data):
        while not self.client_.wait_for_service(0.5):
            self.get_logger().warn("Waiting for start_data server...")
        
        #Using the interfac for request
        request = StartSomething.Request()
        request.start = data

        future = self.client_.call_async(request)
        future.add_done_callback(
            partial (self.callback_start_data, request=request))
    
    def callback_start_data(self, future, request):
        self.get_logger().info(f'{future.result().verify}')

class DroneMotion(Node):
    def __init__(self):
        super().__init__("drone_motion_client")
        self.cb_group = ReentrantCallbackGroup()
        self.client_ = self.create_client(MovementType, "drone_motion", callback_group = self.cb_group)
    
    def call_drone_motion(self, motion, activation):
        while not self.client_.wait_for_service(0.5):
            self.get_logger().warn("Waiting for speed_server...")
        
        request = MovementType.Request()
        request.movement = motion 
        request.activation = activation

        future = self.client_.call_async(request)
        future.add_done_callback(
            partial (self.callback_drone_motion, request=request))
    
    def callback_drone_motion(self, future, request):
        self.get_logger().info(f'{future.result().verify}')

class PidGainClient(Node):
    def __init__(self, node_name, srv_name):
        super().__init__(node_name)
        self.srv_name = srv_name
        self.cb = ReentrantCallbackGroup()
        self.client_ = self.create_client(GainPid, srv_name, callback_group=self.cb)
        self.get_logger().info(f"{node_name} has been started")
    
    def call_pid_gain(self, identity, gain):
        while not self.client_.wait_for_service(timeout_sec=0.5):
            if not rclpy.ok():
                self.get_logger().error("Interrupted while waiting for the service.")
                return
            self.get_logger().warn(f"Waiting for {self.srv_name}...")
        
        request = GainPid.Request()
        request.gain = gain
        request.identity = identity
        
        future = self.client_.call_async(request)
        future.add_done_callback(
            partial(self.callback_pid_gain, request=request))
    
    def callback_pid_gain(self, future, request):
        try:
            response = future.result()
            self.get_logger().info(f'Response: {response.verify}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

######
