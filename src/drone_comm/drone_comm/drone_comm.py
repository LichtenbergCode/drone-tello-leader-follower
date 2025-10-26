#!/usr/bin/env python3
import cv2
import numpy as np
import math
from djitellopy import tello
import subprocess
import threading
from typing import Callable, Union
import logging
from colorlog import ColoredFormatter

from time import sleep
from drone_comm.drone_ros_comm import *
from drone_comm.drone_pid import *
#from drone_comm.tello_comm_socket import Tello

from rclpy.executors import MultiThreadedExecutor
import rclpy

class  DroneClass:
    def __init__(self, ros_args):
        self.ros_args = ros_args

        self.start_camara_drone = False # Start the video streaming ros2 communication
        self.start_graph = False
        self.connect_pid = False
        self.activated = False # It starts when the drone is activated
        self.master = False # For master
        self.slave = True # For Slave
        self.add_up = False
        self.add_down = False
        self.flying = False
        
        self.start_camara_drone = True
        self.start_graph = False

        self.speed = 50 # This is the speed for keyboard events
        self.battery = 100
        self.temperature = 50.0
        self.output = 0 # This is the altitude speed when the drone is connected to the PID 
        self.lr, self.fb, self.up, self.yv = 0, 0, 0, 0

        # logger
        formatter = ColoredFormatter(
            "%(log_color)s%(levelname)s:%(name)s:%(message)s",
            log_colors={
                "DEBUG": "cyan",
                "INFO": "green",
                "WARNING": "yellow",
                "ERROR": "red",
                "CRITICAL": "bold_red",
            }
        )
        handler = logging.StreamHandler()
        handler.setFormatter(formatter)

        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.ERROR)
        self.logger.addHandler(handler)

        #BBOX VARS
        self.x_min = 0
        self.y_min = 0
        self.x_max = 0
        self.y_min = 0
        self.height = 0
        self.width = 0
        self.bbox_activated = False
        self.detection = False # 
        self.slave_pid_activation = False # Activated when pid slave starts 
        self.cx = 0
        self.cy = 0
        self.area = 0

        #THROTTLE SLAVE
        self.throttle_slave_activation = False
        self.throttle_slave_activation_external = False
        self.throttle_slave_proportional_gain = 1.4
        self.throttle_slave_integral_gain = 0.6
        self.throttle_slave_derivative_gain = 0.4
        self.throttle_slave_system_variable = 0.0
        self.throttle_slave_setpoint = 320
        self.throttle_slave_pid = Pid(self.get_gui_parameters, 
                                    self.get_gui_setpoint, 25, -25,
                                    "ThrottleSlave")

        #YAW SLAVE
        self.yaw_slave_activation = False
        self.yaw_slave_activation_external = False
        self.yaw_slave_proportional_gain = 1.4
        self.yaw_slave_integral_gain = 0.4
        self.yaw_slave_derivative_gain = 0.1
        self.yaw_slave_system_variable = 0.0
        self.yaw_slave_setpoint = 320
        self.yaw_slave_pid = Pid(self.get_gui_parameters,
                                self.get_gui_setpoint, 30, -30,
                                "YawSlave")
        #ROLL SLAVE
        self.roll_slave_activation = False
        self.roll_slave_activation_external = False
        self.roll_slave_proportional_gain = 1.4
        self.roll_slave_integral_gain = 0.6
        self.roll_slave_derivative_gain = 0.01
        self.roll_slave_system_variable = 0.0
        self.roll_slave_setpoint = 320
        self.roll_slave_pid = Pid(self.get_gui_parameters,
                                self.get_gui_setpoint, 20, -20,
                                "RollSlave")
        #PITCH SLAVE
        self.pitch_slave_activation = False
        self.pitch_slave_activation_external = False
        self.pitch_slave_proportional_gain = 1.2
        self.pitch_slave_integral_gain = 0.4
        self.pitch_slave_derivative_gain = 0.1
        self.pitch_slave_system_variable = 0.0
        self.pitch_slave_setpoint = 7125
        self.pitch_slave_pid = Pid(self.get_gui_parameters,
                                self.get_gui_setpoint, 25, -25,
                                "PitchSlave")

        #THROTTLE MASTER
        self.throttle_master_activation = False
        self.throttle_master_activation_external = False
        self.throttle_master_proportional_gain = 1.6
        self.throttle_master_integral_gain = 0.6
        self.throttle_master_derivative_gain = 0.4
        self.throttle_master_system_variable = 10
        self.throttle_master_setpoint = 20
        self.throttle_master_pid = Pid(self.get_gui_parameters,
                                    self.get_gui_setpoint, 60, -60,
                                    "ThrottleMaster")

        self.drone = tello.Tello()
        self.ros2_communication()

    def ros2_communication(self):
        #############
        # ros2 node's creation
        #############
        rclpy.init(args=self.ros_args)

        # Topics
        self.call_battery = Battery(self.get_battery)
        self.call_data_status = GeneralDataStatus(self.get_status)
        # detection topic sub
        self.call_bbox = Bbox(self.get_bbox)

        # START TOPICS FOR PID
        self.call_get_pid_throttle_slave = GetPid(self.get_pid_data, 
                                            "throttle_slave_pid_publisher",
                                            "throttle_slave",
                                            "ThrottleSlave")
        self.call_get_pid_yaw_slave = GetPid(self.get_pid_data,
                                            "yaw_slave_pid_publisher",
                                            "yaw_slave",
                                            "YawSlave")
        self.call_get_pid_roll_slave = GetPid(self.get_pid_data,
                                            "roll_slave_pid_publisher",
                                            "roll_slave",
                                            "RollSlave")
        self.call_get_pid_pitch_slave = GetPid(self.get_pid_data,
                                            "pitch_slave_pid_publisher",
                                            "pitch_slave",
                                            "PitchSlave")
        self.call_get_pid_throttle_master = GetPid(self.get_pid_data,
                                            "throttle_master_pid_publisher",
                                            "throttle_master",
                                            "ThrottleMaster")
        #self.call_get_img = GetImg(self.camara_img)


        # Srv
        self.call_speed_data = SpeedData(self.speed_data)
        self.call_start_data = StartData(self.activate)
        self.call_drone_motion = DroneMotion(self.drone_motion)
        
        # Clt
        self.call_start_detection = StartDetection()

        
        ###
        self.call_proportional_pid = PidGainServer(self.change_proportional_gain, 
                                                "proportional_server", 
                                                "pid_proportional")
        self.call_integral_pid = PidGainServer(self.change_integral_gain, 
                                                "integral_server", 
                                                "pid_integral")
        self.call_derivative_pid = PidGainServer(self.change_derivative_gain, 
                                                "derivative_server", 
                                                "pid_derivative") 
        
        # Executors
        executor = MultiThreadedExecutor()

        #Adding Topics
        executor.add_node(self.call_battery)
        executor.add_node(self.call_data_status)
        executor.add_node(self.call_get_pid_throttle_slave)
        executor.add_node(self.call_get_pid_yaw_slave)
        executor.add_node(self.call_get_pid_roll_slave)
        executor.add_node(self.call_get_pid_pitch_slave)
        executor.add_node(self.call_get_pid_throttle_master)
        executor.add_node(self.call_bbox)
        #executor.add_node(self.call_get_img)
        ###
        
        #Adding srvs
        executor.add_node(self.call_speed_data)
        executor.add_node(self.call_start_data)
        executor.add_node(self.call_drone_motion)
        ###
        executor.add_node(self.call_proportional_pid)
        executor.add_node(self.call_integral_pid)
        executor.add_node(self.call_derivative_pid)

        #Adding clts
        executor.add_node(self.call_start_detection)

        executor.spin()
        rclpy.shutdown()

    # For the topics
    def get_battery(self)->tuple[bool, int]:
        ##########
        # Ros2 Communication
        # returns the battery state
        ##########
        if self.activated:
            try: 
                #battery = self.drone.bat
                battery = self.drone.get_battery()
                self.logger.info("Sending battery")
                return True, battery
            except Exception as e:
                self.logger.exception("Failed to get battery") 
                return False, 0 #dafault value
        else: 
            #self.logger.warning("Failed to get battery")
            #self.logger.warning("The drone is not connected")
            return False, 0
    
    def get_status(self)->tuple[bool, list[str]]: 
        #########
        # ROS2 Commnication
        # returns the sensor drone data
        #########
        if self.activated:
            try:
                state = self.drone.get_current_state()
                state_values =  list(state.values())
                state_values = [str(data) for data in state.values()]
                del state_values[3:6]
                del state_values[10:]
                self.logger.info("Sending current sensors state")
                return True, state_values
            except Exception as e:
                self.logger.exception("Failed to get state")
                return False, None
        else:
            self.logger.warning("Failed to get current sensors state")
            self.logger.warning("The drone is not connected or " \
            "the variable: self.start_camara_drone is not activated")
            return False, None
    
    def get_bbox(self, bbox_tuple: tuple[float], detection: bool)-> None:
        #############
        # ROS2 Communication 
        # Get the Bbox to control the drone in slave mode
        #############
        a, b, c, d, e, f = bbox_tuple
        self.x_min = a # min coordenate in y axis 
        self.y_min = b
        
        self.x_max = c
        self.y_max = d
        
        self.width = e # bbox height
        self.height = f # bbox width
        #(cx, cy): centro de la BBox (cx = x + w/2, cy = y + h/2).
        self.cx = 640 - (self.x_min+self.width//2)
        print(f"cx: {self.cx}")
        self.cy = (self.y_min+self.height//2) + 80 
        print(f"cy:{self.cy}")
        self.area = self.width*self.height
        #print(f"{bbox_tuple}")
        self.detection = detection
        print(f"{detection}")
    
    def get_pid_data(self, identity:str) -> tuple[Union[int, float], float, float, float]:
        ##########
        # ROS2 Communication
        
        #returns: 
            # *system variable
            # *setpoint
            # *error
            # *output
        ##########
        match identity: 
            case "ThrottleSlave":
                if self.activated:
                    data_tuple = self.throttle_slave_pid.get_data()
                    send_data = (float(data) for data in data_tuple)
                    #send_data = (10.0,20.0,25.0,30.0)
                    return True, send_data
                else:
                    return False, None
            case "YawSlave":
                if self.activated:
                    data_tuple = self.yaw_slave_pid.get_data()
                    send_data = (float(data) for data in data_tuple)
                    #send_data = (10.0,20.0,25.0,30.0)
                    return True, send_data
                else:
                    return False, None
            case "RollSlave":
                if self.activated:
                    data_tuple = self.roll_slave_pid.get_data()
                    send_data = (float(data) for data in data_tuple)
                    #send_data = (10.0,20.0,25.0,30.0)
                    return True, send_data
                else:
                    return False, None
            case "PitchSlave":
                if self.activated:
                    data_tuple = self.pitch_slave_pid.get_data()
                    send_data = (float(data) for data in data_tuple)
                    #send_data = (10.0,20.0,25.0,30.0)
                    return True, send_data
                else: 
                    return False, None
            case "ThrottleMaster":
                #data_tuple = self.throttle_master_pid.get_data()
                #if self.start_graph and self.master and self.throttle_master_activation:
                if self.activated:
                    data_tuple = self.throttle_master_pid.get_data()
                    send_data = (float(data) for data in data_tuple)
                    #send_data = (10.0,20.0,25.0,30.0)
                    #self.logger.info("Sending ThrottleMaster Data")
                    return True, send_data
                else:
                    self.logger.warning("ThrottleMaster Cannot send the data")
                    return False, None
        
    def camara_img(self) -> tuple[bool, np.ndarray]: # Deactivate
        ##################
        # Ros2 communication
        # returns the state of the drone camara
        ###################
        if self.activated and self.start_camara_drone:
            try:
                self.frame = self.drone.get_frame_read().frame
                scale_percent = 50
                width = int(self.frame.shape[1]* scale_percent/80)
                height = int(self.frame.shape[0]*scale_percent/80)
                self.frame = cv2.resize(self.frame, (width, height), interpolation=cv2.INTER_AREA)
                self.frame = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
                #self.logger.info("Sending frame")
                return True, self.frame
            except Exception as e:
                #self.logger.exception("Failed to send drone frame")
                return False, None
        else:
            #self.logger.warning("Failed to send drone frame")
            #self.logger.warning("The drone is not connected or " \
            #"the variable: self.start_camara_drone is not activated")
            return False, None
    
    # For ROS2 services
    def speed_data(self, speed:int) -> bool:
        #######################
        # Ros2 communication
        # change the speed
        #####################
        self.speed = speed
        self.logger.info("Sending speed data")
        return True

    def activate(self, active:str) -> bool: # for start data ros2 service
        ####################
        # Ros2 communication
        # Activates or deactivates funcionalities:
        # *Connect 
        #       * Starts communication with tello
        #       * Starts video_streaming
        #       * Starts bbox communication
        #       * Starts detection
        # *Master
        # *Slave
        # *CamaraImg
        #       * Starts video streaming in the pipeline
        #       * Deactivate graph
        # *UpdateGraph
        #       * Start Graph
        #       * Deactivate video streaming
        # *TkOff
        # *Land
        # *ThrottleSlave
        # *YawSlave
        # *RollSlave
        # *PitchSlave
        # *ThrottleMaster
        # *ThrottleSlaveExternal
        # *YawSlaveExternal
        # *RollSlaveExternal
        # *PitchSlaveExternal
        # *ThrottleMasterExternal
        # *Exit
        #####################################
        # activates and deactivates functionalities in detection node
        # StartDetection -> Activated in Connect
        # StartVideo -> Activated/Deactivated in CamaraImg
        # StartBbox -> Activated/Deactivated with:
        #               * self.call_start_detection
        #               * Connect
        #               * Master
        #               * Slave           
        ####################
        if active == "Connect":
            try:
                # To verify that that the radio Wi-Fi es activated
                subprocess.run(['nmcli', 'radio', 'wifi', 'on'])
                subprocess.run(['nmcli', 'device', 'wifi', 'rescan'])
                sleep(2)
                subprocess.run(['nmcli', '-t', '-f', 'SSID,BSSID', 'device', 'wifi', 'list'], text = True)
                connect = subprocess.run(['nmcli', 'device', 'wifi', 'connect','TELLO-9A6F11', ], text = True, capture_output=True)
                sleep(2)
                if connect.returncode != 0:
                    self.logger.error(f"Failed to connect to Tello: {connect.stderr}")
                    return False
                try:
                    self.drone.connect()
                    self.drone.streamon()
                except Exception as e:
                    self.logger.exception("Djitellopy Error Comm")
                    return False
                
                self.call_start_detection.call_start_detection("StartDetection")
                self.call_start_detection.call_start_detection("StartBbox")
                self.call_start_detection.call_start_detection("StartVideo")
                self.call_data_status.start_timer()
                
                self.activated = True
                self.logger.info("Succesfully communication")
                return True
            ########
            except Exception as e:
                self.logger.exception("Unseccesfully communication")
                return False
            
        match active:
            case "Master":
                try:
                    self.master = True
                    self.slave = False
                    # ...
                    self.call_get_pid_throttle_slave.stop_timer()
                    self.call_get_pid_yaw_slave.stop_timer()
                    self.call_get_pid_pitch_slave.stop_timer()
                    self.call_get_pid_roll_slave.stop_timer()
                    #------
                    self.call_data_status.start_timer()
                    #self.call_get_img.start_timer()
                    # ...
                    self.logger.info("Master mode activated")
                    return True
                except Exception as e:
                    self.logger.exception("Master mode Error")
                    return False
            case "Slave":
                try:
                    self.slave = True
                    self.master = False
                    # ...
                    self.call_get_pid_throttle_master.stop_timer()
                    self.call_data_status.start_timer()
                    #self.call_get_img.start_timer()
                    # ...
                    self.logger.info("Slave mode activated")
                    return True
                except Exception as e:
                    self.logger.exception("Slave mode Error")
            case "CamaraImg":
                try:
                    self.start_camara_drone = True
                    self.start_graph = False
                    # ...
                    if self.slave:
                        if not self.throttle_slave_activation_external:
                            self.call_get_pid_throttle_slave.stop_timer()
                        if not self.yaw_slave_activation_external:
                            self.call_get_pid_yaw_slave.stop_timer()
                        if not self.pitch_slave_activation_external:
                            self.call_get_pid_pitch_slave.stop_timer()
                        if not self.roll_slave_activation_external:
                            self.call_get_pid_roll_slave.stop_timer()
                    if self.master:
                        if not self.throttle_master_activation_external:
                            self.call_get_pid_throttle_master.stop_timer()
                    
                    self.call_data_status.start_timer()
                    #self.call_get_img.start_timer()
                    self.call_start_detection.call_start_detection("StartVideo")
                    # ...
                    self.logger.info("CamaraImg mode activated")
                    return True
                except Exception as e:
                    self.logger.exception("CamaraImg Error")
                    return False
            case "UpdateGraph":
                try:
                    self.start_camara_drone = False
                    self.start_graph = True
                    # ...
                    if self.slave:
                        self.call_get_pid_throttle_slave.start_timer()
                        self.call_get_pid_yaw_slave.start_timer()
                        self.call_get_pid_pitch_slave.start_timer()
                        self.call_get_pid_roll_slave.start_timer()
                    if self.master:
                        self.call_get_pid_throttle_master.start_timer()
                    self.call_data_status.stop_timer()
                    #self.call_get_img.start_timer()
                    self.call_start_detection.call_start_detection("StopVideo")
                    # ...
                    self.logger.info("UpdateGraph mode activated")
                    return True
                except Exception as e:
                    self.logger.exception("UpdateGraph Error")
                    return False
            case "TkOff":
                try:
                    if self.activated:
                        self.drone.takeoff()
                        self.logger.info("Taking Off Succesfully")
                        return True
                    else: 
                        self.logger.warning("TkOff Error")
                        self.logger.warning("The drone is not connected")
                        return False
                except Exception as e:
                    self.logger.exception("TkOff Error")
                    return False
            case "Land":
                try:
                    if self.drone.is_flying:
                        self.drone.land()
                        self.logger.info("Landing Succesfully")
                        return True
                    else:
                        self.logger.warning("Land Error")
                        self.logger.warning("The drone is not connected")
                        return False
                except Exception as e:
                    self.logger.exception("Land Error")
                    return False
            case "ThrottleSlave":
                print("Hello since throttleSlave")
                if self.throttle_slave_activation:
                    # .....
                    #
                    # .....
                    self.throttle_slave_activation = False
                    self.logger.info("ThrottleSlave Unactivated")
                else:
                    # .....
                    #
                    # .....
                    self.throttle_slave_activation = True
                    if not self.slave_pid_activation:
                        self.slave_pid_activation = True
                        th2 = threading.Thread(target = self.start_pid_slave, daemon= True)
                        th2.start()
                    self.logger.info("ThrottleSlave Activated")
                return True
            case "YawSlave":
                if self.yaw_slave_activation:
                    # .....
                    #
                    # .....
                    self.yaw_slave_activation = False
                    self.logger.info("YawSlave Unactivated")
                else: 
                    self.yaw_slave_activation = True
                    if not self.slave_pid_activation:
                        self.slave_pid_activation = True
                        th2 = threading.Thread(target = self.start_pid_slave, daemon= True)
                        th2.start()
                    self.logger.info("YawSlave Activated")
                return True
            case "RollSlave":
                print("Hello since RollSlave")
                if self.roll_slave_activation:
                    # .....
                    #
                    # .....
                    self.roll_slave_activation = False
                    self.logger.info("RollSlave Unactivated")
                else:
                    self.roll_slave_activation = True
                    if not self.slave_pid_activation:
                        self.slave_pid_activation = True
                        th2 = threading.Thread(target = self.start_pid_slave, daemon= True)
                        th2.start()
                    self.logger.info("RollSlave Activated")
                return True
            case "PitchSlave":
                if self.pitch_slave_activation:
                    # .....
                    #
                    # .....
                    self.pitch_slave_activation = False
                    self.logger.info("PitchSlave Unactivated")
                else:
                    # .....
                    # 
                    # .....
                    self.pitch_slave_activation = True
                    if not self.slave_pid_activation:
                        self.slave_pid_activation = True
                        th2 = threading.Thread(target = self.start_pid_slave, daemon= True)
                        th2.start()
                    self.logger.info("PitchSlave Activated")
                return True
            case "ThrottleMaster":
                if self.throttle_master_activation:
                    # .....
                    #
                    # .....
                    self.throttle_master_activation = False
                    self.logger.info("ThrottleMaster Unactivated")
                else:
                    self.throttle_master_activation = True
                    # .....
                    th1 = threading.Thread(target = self.start_pid_master, daemon= True)
                    th1.start()
                    # .....
                    self.logger.info("ThrottleMaster Activated")
                return True
            case "ThrottleSlaveExternal":
                if self.throttle_slave_activation_external:
                    self.throttle_slave_activation_external = False
                    self.logger.info("ThrottleSlaveExternal Unactivated")
                else:
                    self.throttle_slave_activation_external = False
                    self.logger.info("ThrottleSlaveExternal Activated")
                return True
            case "YawSlaveExternal":
                if self.yaw_slave_activation_external:
                    self.yaw_slave_activation_external = False
                    self.logger.info("YawSlaveExternal Unactivated")
                else:
                    self.yaw_slave_activation_external = True
                    self.logger.info("YawSlaveExternal Activated")
                    return True
            case "RollSlaveExternal":
                if self.roll_slave_activation_external:
                    self.roll_slave_activation_external = False
                    self.logger.info("RollSlaveExternal Unactivated")
                else:
                    self.roll_slave_activation_external = True
                    self.logger.info("RollSlaveExternal Activated")
                    return True
            case "PitchSlaveExternal":
                if self.pitch_slave_activation_external:
                    self.pitch_slave_activation_external = False
                    self.logger.info("PitchSlaveExternal Unactivated")
                else:
                    self.pitch_slave_activation_external = True
                    self.logger.info("PitchSlaveExternal Activated")
            case "ThrottleMasterExternal":
                if self.throttle_master_activation_external:
                    self.throttle_master_activation_external = False
                    self.logger.info("ThrottleMasterExternal Unactivated")
                else:
                    self.throttle_master_activation_external = True
                    self.logger.info("ThrottleMasterExternal Activated")
            case "Exit":
                try:
                    rclpy.shutdown()
                    self.logger.info("Exit Succesfull")
                    return True
                except Exception as e:
                    self.logger.exception("Exit Error")
                    return False

    def z_axis_distance(self, identity):
        if self.add_up:
            if identity == "UP":
                self.throttle_master_setpoint += self.speed*0.3
            elif identity == "DOWN":
                self.throttle_master_setpoint -= self.speed*0.3
            print(f"sv_m:{self.throttle_master_setpoint}")
            time.sleep(0.3)
    
    def drone_motion(self, motion: str, activation: bool) -> bool:
        ###################
        # motion = type of movement
        # * FORWARD
        # * BACKWARD
        # * LEFT
        # * RIGHT
        # * TURNL
        # * TURNR
        # * UP
        # * DOWN
        # activation = activate or deactivate the movement

        if self.master:
            match motion:
                case "FORWARD":
                    if activation:
                        self.fb = self.speed
                        self.logger.info("FORWARD")
                    else:
                        self.fb = 0
                        self.logger.info("FORWARD STOP")
                case "BACKWARD":
                    if activation:
                        self.fb = -self.speed
                        self.logger.info("BACKWARD")
                    else:
                        self.fb = 0
                        self.logger.info("BACKWARD STOP")
                case "LEFT":
                    if activation:
                        self.lr = -self.speed
                        self.logger.info("LEFT")
                    else:
                        self.lr = 0
                        self.logger.info("LEFT STOP")
                case "RIGHT":
                    if activation:
                        self.lr = self.speed
                        self.logger.info("RIGHT")
                    else:
                        self.lr = 0
                        self.logger.info("RIGHT STOP")
                case "TURNL":
                    if activation:
                        self.yv = -self.speed
                        self.logger.info("TURNL")
                    else:
                        self.yv = 0
                        self.logger.info("TURNL STOP")
                case "TURNR":
                    if activation:
                        self.yv = self.speed
                        self.logger.info("TURNR")
                    else:
                        self.yv = 0
                        self.logger.info("TURNR STOP")
                case "UP":
                    if self.throttle_master_activation:
                        if not self.add_down and activation:
                            self.add_up = True
                            th = threading.Thread(target = self.z_axis_distance, args=("UP",) ,daemon=True)
                            th.start()
                            self.logger.info("UP")
                            return True
                        else:
                            self.add_up = False
                            self.logger.info("UP Stop")
                            return True
                    else:
                        if activation:
                            self.up = self.speed
                            self.logger.info("UP")
                        else:
                            self.up = 0
                            self.logger.info("UP Stop")
                        
                case "DOWN":
                    if self.throttle_master_activation:
                        if activation and not self.add_up:
                            self.add_up = True
                            th = threading.Thread(target = self.z_axis_distance, args=("DOWN",) ,daemon=True)
                            th.start()
                            self.logger.info("DOWN")
                            return True
                        else:
                            self.add_up = False
                            self.logger.info("DOWN STOP")
                            return True
                    else:
                        if activation:
                            self.up = -self.speed
                            self.logger.info("DOWN")
                        else: 
                            self.up = 0
                            self.logger.info("DOWN STOP")
            response = self.drone_motion_send()
            return response
        else:
            self.logger.warning("Not master mode activated")
            return False
    
    def start_pid_master(self):
        while self.throttle_master_activation:
            self.up = self.throttle_master_pid.sequence()
            self.drone_motion_send()
            time.sleep(0.2)
        self.throttle_master_pid.restart()
    
    def start_pid_slave(self):
        i = 0
        while True:
            self.fb = 0 #-> pitch
            self.yv = 0 #-> yaw
            self.up = 0 #-> throttle
            self.lr = 0 #-> roll
            # self.drone.send_rc_control(self.lr, self.fb, self.up, self.yv)

            if self.detection:
                i = 0
                if self.throttle_slave_activation:
                    self.up  = self.throttle_slave_pid.sequence()
                else: self.throttle_slave_pid.restart()

                if self.yaw_slave_activation:
                    self.yv = self.yaw_slave_pid.sequence()
                else: self.yaw_slave_pid.restart()
                
                if self.roll_slave_activation:
                    self.lr = self.roll_slave_pid.sequence()
                else: self.roll_slave_pid.restart()
                
                if self.pitch_slave_activation:
                    self.fb = self.pitch_slave_pid.sequence()
                else: self.pitch_slave_pid.restart()
            
            else: 
                print ("No detection")
                if self.throttle_slave_activation: 
                    self.throttle_slave_pid.restart()
                
                if self.yaw_slave_activation:
                    self.yaw_slave_pid.restart()
                
                if self.roll_slave_activation:
                    self.roll_slave_pid.restart()
                
                if self.pitch_slave_activation:
                    self.pitch_slave_pid.restart()
                

                #################
                # map  sequence #
                #self.fb = 5
                #if i < 6: 
                #    self.yv = 30
                #else: 
                #    self.yv = -30
                #if i == 10: 
                #    i = 0
                
                
            self.drone_motion_send()
            a = not self.throttle_slave_activation
            b = not self.yaw_slave_activation
            c = not self.roll_slave_activation
            d = not self.pitch_slave_activation
            if a and b and c and d:
                self.slave_pid_activation = False
                break
            time.sleep(0.2) 
    
    def drone_motion_send(self)->False:
        if self.drone.is_flying:
            try: 
                self.drone.send_rc_control(self.lr, self.fb, self.up, self.yv)
                self.logger.info("Drone motion send succesfully")
                return True
            except Exception as e: 
                self.logger.exception("Drone motion error")
                return False
        else:
            self.logger.warning("The drone is not flying")
            return False
    
    def change_proportional_gain(self, identity, gain):
        match identity:
            case "ThrottleSlave":
                self.throttle_slave_proportional_gain = gain
                return True
            case "YawSlave":
                self.yaw_slave_proportional_gain = gain
                return True
            case "RollSlave":
                self.roll_slave_proportional_gain = gain
                return True
            case "PitchSlave":
                self.pitch_slave_proportional_gain = gain
                return True
            case "ThrottleMaster":
                self.throttle_master_proportional_gain = gain
                return True
            case _:
                return False

    def change_integral_gain(self, identity, gain):
        match identity:
            case "ThrottleSlave":
                self.throttle_slave_integral_gain = gain
                return True
            case "YawSlave":
                self.yaw_slave_integral_gain = gain
                return True
            case "RollSlave":
                self.roll_slave_integral_gain = gain
                return True
            case "PitchSlave":
                self.pitch_slave_integral_gain = gain
                return True
            case "ThrottleMaster":
                self.throttle_master_integral_gain = gain
                return True
            case _: return False

    def change_derivative_gain(self, identity, gain):
        match identity:
            case "ThrottleSlave":
                self.throttle_slave_derivative_gain = gain
                return True
            case "YawSlave":
                self.yaw_slave_derivative_gain = gain
                return True
            case "RollSlave":
                self.roll_slave_derivative_gain = gain
                return True
            case "PitchSlave":
                self.pitch_slave_derivative_gain = gain
                return True
            case "ThrottleMaster":
                self.throttle_master_derivative_gain = gain
                return True
            case _: return False
            
    def get_gui_parameters(self, identity):
        
        match identity:
            case "ThrottleSlave":
                self.throttle_slave_system_variable = self.cy
                throttle_slave_t = (self.throttle_slave_proportional_gain,
                                    self.throttle_slave_integral_gain,
                                    self.throttle_slave_derivative_gain,
                                    self.throttle_slave_system_variable)
                return throttle_slave_t
            case "YawSlave":
                self.yaw_slave_system_variable = self.cx
                yaw_slave_t = (self.yaw_slave_proportional_gain,
                                self.yaw_slave_integral_gain,
                                self.yaw_slave_derivative_gain,
                                self.yaw_slave_system_variable)
                return yaw_slave_t
            case "RollSlave":
                self.roll_slave_system_variable = self.cx
                roll_slave_t = (self.roll_slave_proportional_gain,
                                self.roll_slave_integral_gain,
                                self.roll_slave_derivative_gain,
                                self.roll_slave_system_variable)
                return roll_slave_t
            case "PitchSlave":
                self.pitch_slave_system_variable = self.area
                pitch_slave_t = (self.pitch_slave_proportional_gain,
                                self.pitch_slave_derivative_gain,
                                self.pitch_slave_derivative_gain,
                                self.pitch_slave_system_variable)
                return pitch_slave_t
            case "ThrottleMaster":
                # Getting the system variable
                try:
                    self.throttle_master_system_variable = self.drone.get_distance_tof()
                    self.logger.info(f"{identity} Succesfully")
                    self.logger.info("Get distance tof Succesfully")
                except Exception as e:
                    self.throttle_master_system_variable = 10
                    self.logger.exception("Get distance Tof Unsuccesfully")
    
                thtrottle_master_t = (self.throttle_master_proportional_gain,
                                    self.throttle_master_integral_gain,
                                    self.throttle_master_derivative_gain,
                                    self.throttle_master_system_variable)
                return thtrottle_master_t
    
    def get_gui_setpoint(self,identity):
        if self.activated:
            match identity:
                case "ThrottleSlave":
                    return self.throttle_slave_setpoint
                case "YawSlave":
                    return self.yaw_slave_setpoint
                case "RollSlave":
                    return self.roll_slave_setpoint
                case "PitchSlave":
                    return self.pitch_slave_setpoint
                case "ThrottleMaster":
                    return self.throttle_master_setpoint

def main(args = None):
    DroneClass(args)