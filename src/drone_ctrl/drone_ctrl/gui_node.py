#!/usr/bin/env python3
import customtkinter as ctk
import threading
from pynput.keyboard import Key, Listener, KeyCode
from PIL import Image as ImagePIL
from PIL import ImageTk, ImageOps
from time import sleep

import rclpy
from rclpy.executors import MultiThreadedExecutor 

from drone_ctrl.segmentation import *
from drone_ctrl.segmented_panels import *
from drone_ctrl.settings import *
from drone_ctrl.upframe import *
from drone_ctrl.gui_ros_comm import *
import numpy as np
import os

class Window(ctk.CTk):
    def __init__(self, ros_args):
        super().__init__(fg_color=DARKGRAY)
        ctk.set_appearance_mode('dark')
        self.geometry('920x480')
        self.title('PID Drone Control')
        #self.minsize(980, 560)
        #self.maxsize(1000, 600)
        self.resizable(False, False)
        self.protocol("WM_DELETE_WINDOW", self.close_protocol)

        # ros2
        ###############################
        self.ros_args = ros_args
        # layout 
        self.rowconfigure(0, weight = 5, uniform = 'a')
        self.rowconfigure(1, weight = 35, uniform = 'a')
        self.rowconfigure(2, weight = 1, uniform = 'a')
        self.columnconfigure(0, weight = 6, uniform = 'a')
        self.columnconfigure(1, weight = 30, uniform = 'a')
        self.columnconfigure(2, weight = 5, uniform = 'a')

        # creation
        self.init_parameters()
        self.creation()
        self.check_command()
        
        #threads
        th1 = threading.Thread(target = self.keyboard_method, daemon = True)
        th2 = threading.Thread(target = self.comm_ros, daemon = True)
        th1.start()
        th2.start()

        #self.ros_srv_send_data('Slave')
        self.img_capture = False
        #
        self.update()
        self.mainloop()

    def comm_ros(self): #ros2 communication
        rclpy.init(args = self.ros_args)
        executor = MultiThreadedExecutor()
        # tpcs
        self.ros_comm_battery = GetBattery(self.ros_tp_battery)
        self.ros_comm_get_status = GetStatus(self.ros_tp_get_status)
        #self.ros_comm_get_pid = GetPid()
        self.ros_comm_get_throttle_slave = GetPid(self.update_graph, 
                                                "throttle_slave_pid_subscriber",
                                                "throttle_slave",
                                                "ThrottleSlave")
        self.ros_comm_get_yaw_slave = GetPid(self.update_graph,
                                                "yaw_slave_pid_subscriber",
                                                "yaw_slave",
                                                "YawSlave")
        self.ros_comm_get_roll_slave = GetPid(self.update_graph,
                                                "roll_slave_pid_subscriber",
                                                "roll_slave",
                                                "RollSlave")
        self.ros_comm_get_pitch_slave = GetPid(self.update_graph,
                                                "pitch_slave_pid_subscriber",
                                                "pitch_slave",
                                                "PitchSlave")
        self.ros_comm_get_throttle_master = GetPid(self.update_graph,
                                                "throttle_master_pid_subscriber",
                                                "throttle_master",
                                                "ThrottleMaster")
        
        self.ros_comm_get_img = SubscriberGetImageNode(self.ros_tp_get_frame)
        
        # srvs
        self.ros_comm_speed_client = DroneSpeedClient() 
        # Change the speed in the master mode
        
        self.ros_comm_start_data_client = StartData()
        # Starts something
        #   * Start communication
        #   *...
        
        self.ros_comm_motion_client = DroneMotion()
        # Send drone movements in master mode
        #   * ...
        #########
        self.ros_comm_pid_gain_client_proportional = PidGainClient(
            "proportional_client",
            "pid_proportional")
        self.ros_comm_pid_gain_client_integral = PidGainClient(
            "integral_client",
            "pid_integral"
        )
        self.ros_comm_pid_gain_client_derivative = PidGainClient(
            "derivative_client",
            "pid_derivative"
        )
        # Send gain: 
        #   *proportional
        #   *integral
        #   *derivative
        ############################### 

        # Adding subscribers:
        executor.add_node(self.ros_comm_battery)
        executor.add_node(self.ros_comm_get_status)
        executor.add_node(self.ros_comm_get_img)
        #---
        executor.add_node(self.ros_comm_get_throttle_slave)
        executor.add_node(self.ros_comm_get_yaw_slave)
        executor.add_node(self.ros_comm_get_roll_slave)
        executor.add_node(self.ros_comm_get_pitch_slave)
        executor.add_node(self.ros_comm_get_throttle_master)
        
        # Adding clients:
        executor.add_node(self.ros_comm_speed_client)
        executor.add_node(self.ros_comm_start_data_client)
        executor.add_node(self.ros_comm_motion_client)
        # ---
        executor.add_node(self.ros_comm_pid_gain_client_proportional)
        executor.add_node(self.ros_comm_pid_gain_client_integral)
        executor.add_node(self.ros_comm_pid_gain_client_derivative)

        executor.spin()
        rclpy.shutdown()
    
    # Methods for subscribers
    def ros_tp_battery(self, data):
        
        battery = data/100
        self.progress_battery.set(value = battery)
        
        if data > 40 and self.green_battery:
            self.up_frame.change_color_battery  = PROGRESSCOLOR
            self.green_battery = False
            self.yellow_battery = True
            self.red_battery = True
        elif data < 40 and data > 17 and self.yellow_battery:
            self.up_frame.change_color_battery = YELLOWBATTERY
            self.green_battery = True
            self.yellow_battery = False
            self.red_battery = True
        elif data < 17 and self.red_battery:
            self.up_frame.change_color_battery = REDBATTERY
            self.green_battery = True
            self.yellow_battery = True
            self.red_battery = False 
        
    def ros_tp_get_status(self, data_list:list):
        self.labels_data = [data for data in data_list]
        self.check_command()

    def update_graph(self, identity:str, data:tuple[float]):
        match identity:
            case "ThrottleSlave":
                self.graph1.change_data(data)
                if self.creation_button_activated_throttle:
                    self.external_window_throttle.graph.change_data(data)
                    #(msg.system_variable, msg.setpoint, msg.error, msg.output)
                    self.throttle_drone_labels["system_variable"].set(f"var:{data[0]}")
                    self.throttle_drone_labels["setpoint"].set(f"sp:{data[1]}")
                    self.throttle_drone_labels["error"].set(f"error:{data[2]}")
                    self.throttle_drone_labels["output"].set(f"output:{data[3]}")
                    
                return True
            case "YawSlave":
                self.graph2.change_data(data)
                if self.creation_button_activated_yaw:
                    self.external_window_yaw.graph.change_data(data)
                    self.yaw_drone_labels["system_variable"].set(f"var:{data[0]}")
                    self.yaw_drone_labels["setpoint"].set(f"sp:{data[1]}")
                    self.yaw_drone_labels["error"].set(f"error:{data[2]}")
                    self.yaw_drone_labels["output"].set(f"output:{data[3]}")

                return True
            case "RollSlave":
                self.graph3.change_data(data)
                if self.creation_button_activated_roll:
                    self.external_window_roll.graph.change_data(data)
                    self.roll_drone_labels["system_variable"].set(f"var:{data[0]}")
                    self.roll_drone_labels["setpoint"].set(f"sp:{data[1]}")
                    self.roll_drone_labels["error"].set(f"error:{data[2]}")
                    self.roll_drone_labels["output"].set(f"output:{data[3]}")

                return True
            case "PitchSlave":
                self.graph4.change_data(data)
                if self.creation_button_activated_pitch:
                    self.external_window_pitch.graph.change_data(data)
                    self.pitch_drone_labels["system_variable"].set(f"var:{data[0]}")
                    self.pitch_drone_labels["setpoint"].set(f"sp:{data[1]}")
                    self.pitch_drone_labels["error"].set(f"error:{data[2]}")
                    self.pitch_drone_labels["output"].set(f"output:{data[3]}")

                return True
            case "ThrottleMaster":
                self.graph1_master.change_data(data)
                if self.creation_button_activated_throttle_master:
                    self.external_window_throttle_master.graph.change_data(data)
                    self.throttle_master_drone_labels["system_variable"].set(f"var:{data[0]}")
                    self.throttle_master_drone_labels["setpoint"].set(f"sp:{data[1]}")
                    self.throttle_master_drone_labels["error"].set(f"error:{data[2]}")
                    self.throttle_master_drone_labels["output"].set(f"output:{data[3]}")
                
                return True
    def update(self):
        if self.img_capture:
            img = ImagePIL.fromarray(self.frame)
            self.photo = ImageTk.PhotoImage(image = img)
            self.label_video.label_video_img(self.photo) 
        self.after(15, self.update)
    
    def ros_tp_get_frame(self, frame):
        self.frame = frame
        self.img_capture = True

    # Methods for the servers
    def ros_srv_send_speed(self, data:int) -> None:
        self.ros_comm_speed_client.call_speed_sever(data)

    def ros_srv_send_data(self, data:str) -> None: # Used for start something
        self.ros_comm_start_data_client.call_start_data(data)
        #Connect -> Starts the drone communication

    def ros_srv_send_movement(self, movement_data:str, bool_data:bool):
        self.ros_comm_motion_client.call_drone_motion(movement_data, bool_data)

    def ros_srv_proportional(self, identity, gain):
        gain = float(gain)
        match identity:
            case "THROTTLE":
                self.ros_comm_pid_gain_client_proportional.call_pid_gain("ThrottleSlave", gain)
            case "YAW":
                self.ros_comm_pid_gain_client_proportional.call_pid_gain("YawSlave", gain)
            case "ROLL":
                self.ros_comm_pid_gain_client_proportional.call_pid_gain("RollSlave", gain)
            case "PITCH":
                self.ros_comm_pid_gain_client_proportional.call_pid_gain("PitchSlave", gain)
            case "THROTTLEM":
                self.ros_comm_pid_gain_client_proportional.call_pid_gain("ThrottleMaster", gain)
    
    def ros_srv_integral(self, identity, gain):
        match identity:
            case "THROTTLE":
                self.ros_comm_pid_gain_client_integral.call_pid_gain("ThrottleSlave", gain)
            case "YAW":
                self.ros_comm_pid_gain_client_integral.call_pid_gain("YawSlave", gain)
            case "ROLL":
                self.ros_comm_pid_gain_client_integral.call_pid_gain("RollSlave", gain)
            case "PITCH":
                self.ros_comm_pid_gain_client_integral.call_pid_gain("PitchSlave", gain)
            case "THROTTLEM":
                self.ros_comm_pid_gain_client_integral.call_pid_gain("ThrottleMaster", gain)
    
    def ros_srv_derivative(self, identity, gain):
        match identity:
            case "THROTTLE":
                self.ros_comm_pid_gain_client_derivative.call_pid_gain("ThrottleSlave", gain)
            case "YAW":
                self.ros_comm_pid_gain_client_derivative.call_pid_gain("YawSlave", gain)
            case "ROLL":
                self.ros_comm_pid_gain_client_derivative.call_pid_gain("RollSlave", gain)
            case "PITCH":
                self.ros_comm_pid_gain_client_derivative.call_pid_gain("PitchSlave", gain)
            case "THROTTLEM":
                self.ros_comm_pid_gain_client_derivative.call_pid_gain("ThrottleMaster", gain)

    ##########################################
    def close_protocol(self)-> None:
        self.destroy()

    def init_parameters(self) -> None:
        
        self.throttle_drone = {
            'proportional' : ctk.DoubleVar(value = DEFAULT_PROPORTIONAL_VALUE_THROTTLE),
            'derivative' : ctk.DoubleVar(value = DEFAULT_DERIVATIVE_VALUE_THROTTLE),
            'integral' : ctk.DoubleVar(value = DEFAULT_INTEGRAL_VALUE_THROTTLE),
            'system_variable': ctk.IntVar(value = DEFAULT_THROTTLE) # DELETE LATER
        }

        self.throttle_master_drone = {
            'proportional' : ctk.DoubleVar(value = DEFAULT_PROPORTIONAL_VALUE_THROTTLE_MASTER),
            'derivative' : ctk.DoubleVar(value = DEFAULT_DERIVATIVE_VALUE_THROTTLE_MASTER),
            'integral' : ctk.DoubleVar(value = DEFAULT_INTEGRAL_VALUE_THROTTLE_MASTER),
            'system_variable' : ctk.IntVar(value = DEFAULT_THROTTLE_MASTER)
        }
        self.yaw_drone = {
            'proportional' : ctk.DoubleVar(value = DEFAULT_PROPORTIONAL_VALUE_YAW),
            'derivative' : ctk.DoubleVar(value = DEFAULT_DERIVATIVE_VALUE_YAW),
            'integral' : ctk.DoubleVar(value = DEFAULT_INTEGRAL_VALUE_YAW),
            'system_variable' : ctk.IntVar(value = DEFAULT_YAW) # DELETE LATER
        }

        self.roll_drone = {
            'proportional' : ctk.DoubleVar(value = DEFAULT_PROPORTIONAL_VALUE_ROLL),
            'derivative' : ctk.DoubleVar(value = DEFAULT_DERIVATIVE_VALUE_ROLL),
            'integral': ctk.DoubleVar(value = DEFAULT_INTEGRAL_VALUE_ROLL),
            'system_variable' : ctk.IntVar(value = DEFAULT_ROLL) # DELETE LATER
        }

        self.pitch_drone = {
            'proportional' : ctk.DoubleVar(value = DEFAULT_PROPORTIONAL_VALUE_PITCH),
            'derivative' : ctk.DoubleVar(value = DEFAULT_DERIVATIVE_VALUE_PITCH),
            'integral' : ctk.DoubleVar(value = DEFAULT_INTEGRAL_VALUE_PITCH),
            'system_variable' : ctk.IntVar(value = DEFAULT_PITCH) # DELETE LATER
        }

        #
        self.throttle_drone_labels = {
            'system_variable' : ctk.StringVar(value = "var: "),
            'output' : ctk.StringVar(value = "output: "),
            'setpoint' : ctk.StringVar(value = "sp: "),
            "error" : ctk.StringVar(value = "error: ")
        }
        self.throttle_master_drone_labels = {
            'system_variable' : ctk.StringVar(value = "var: "),
            'output' : ctk.StringVar(value = "output: "),
            'setpoint' : ctk.StringVar(value = "sp: "),
            "error" : ctk.StringVar(value = "error: ")
        }
        self.yaw_drone_labels = {
            'system_variable' : ctk.StringVar(value = "var: "),
            'output' : ctk.StringVar(value = "output: "),
            'setpoint' : ctk.StringVar(value = "sp: "),
            'error' : ctk.StringVar(value = "error: ")
        }
        self.roll_drone_labels = {
            'system_variable' : ctk.StringVar(value="var: "),
            'output' : ctk.StringVar(value="output: "),
            'setpoint' : ctk.StringVar(value="setpoint: "),
            'error' : ctk.StringVar(value="error: ")
        }
        self.pitch_drone_labels = {
            'system_variable' : ctk.StringVar(value="var: "),
            'output' : ctk.StringVar(value="output:"),
            'setpoint' : ctk.StringVar(value="setpoint: "),
            'error' : ctk.StringVar(value="error: ")
        }
        ########################
        self.labels_dict = {
            'pitch' : ctk.BooleanVar(value = False),
            'roll' : ctk.BooleanVar(value = False),
            'yaw' : ctk.BooleanVar(value = False),
            'templ' : ctk.BooleanVar(value = False),
            'temph' : ctk.BooleanVar(value = False),
            'tof' : ctk.BooleanVar(value = True),
            'h' : ctk.BooleanVar(value = False),
            'bat' : ctk.BooleanVar(value = True),
            'baro' : ctk.BooleanVar(value = False),
            'time' : ctk.BooleanVar(value = True)
        }
        self.labels_name = ("pitch", 'roll', 'yaw', 'templ', 'temph', 'tof', 'h', 'bat', 'baro', 'time')
        self.labels_data = ["0" for _ in self.labels_name]

        # Progress bar variable battery
        self.progress_battery = ctk.DoubleVar(value = PROGRESSDEFAULT)
        self.radiobutton_var = ctk.StringVar(value = DEFAULTCONFIGURATION) # IT CAN TAKE THE ARGUMENT OF SLAVE OR MASTER/ DEFAULT: SLAVE
        self.switch_start_var = ctk.BooleanVar(value = STARTDEFAULTCONFIGURATION)# DEFAULT: FALSE
        self.speed_slider = ctk.IntVar(value = DEFAULT_SPEED)
        self.segmented_buttons_commands = (self.button_drone, self.button_graph)
        self.pitch_roll_list = ('FORWARD', 'BACKWARD', 'LEFT', 'RIGHT')
        self.yaw_throttle_list = ('UP', 'DOWN', 'TURNL', 'TURNR')
        self.resources_pitch_roll = ('./src/drone_ctrl/drone_ctrl/Resources/forward.png', './src/drone_ctrl/drone_ctrl/Resources/left.png', './src/drone_ctrl/drone_ctrl/Resources/right.png', './src/drone_ctrl/drone_ctrl/Resources/backward.png')
        self.resources_yaw_throttle = ('./src/drone_ctrl/drone_ctrl/Resources/up.png', './src/drone_ctrl/drone_ctrl/Resources/one.png', './src/drone_ctrl/drone_ctrl/Resources/second.png', './src/drone_ctrl/drone_ctrl/Resources/down.png')
        self.resources_segmentation = ('./src/drone_ctrl/drone_ctrl/Resources/drone_E1E1E1.png', './src/drone_ctrl/drone_ctrl/Resources/graph_E1E1E1.png')
        
        self.var_graph = ('THROTTLE', 'YAW', 'ROLL', 'PITCH', 'ACTIVATE', 'THROTTLEM')
        self.commands_button_graph = (self.button_command_graph_system_variable,  
                                    self.button_command_graph_setpoint, 
                                    self.button_command_graph_error, 
                                    self.button_command_graph_output, 
                                    self.button_command_graph_external_graphic,
                                    self.button_command_graph_activate)
        #tello.get_current_state()
        self.vcmd = (self.register(self.validate_entry), '%P')

        # Default Battery colors Flags
        self.green_battery = True
        self.yellow_battery = False
        self.red_battery = False

        self.master_variable = False
        self.new_window_created = False
        self.value_keyboard_method = True

        self.creation_button_activated_throttle = False
        self.creation_button_activated_throttle_master = False
        self.creation_button_activated_yaw = False
        self.creation_button_activated_roll = False
        self.creation_button_activated_pitch = False

        self.activate_throttle = True
        self.activate_throttle_master = True
        self.activate_yaw = True
        self.activate_roll = True
        self.activate_pitch = True

        self.button_graph_id = 0
        self.button_drone_id = 0
        
    def validate_entry(self, input_value) -> bool:
        if input_value == "":
            return True
        try:
            float(input_value)
            return True
        except ValueError: 
            return False

    def changing_color(self) -> None: # Used to change the color in the icon graph and the icon drone
        self.segments_frame.change_drone_color()
        self.segments_frame.change_graph_color()

    def creation(self) -> None:
        self.segments_frame = Segmentation(self, self.segmented_buttons_commands, self.resources_segmentation, self.tkoff_land_command)
        self.labels_frame = Labels(self, self.labels_name, self.labels_dict, self.check_command, 'Start', self.switch_start_var, self.switch_command)
        del self.labels_frame.switch_frame  # It makes appears the switch
        #self.switch_start_var.set(False)
        self.button_drone()

        self.up_frame = UpFrame(self, self.progress_battery)
        self.left_frame = UpLeftFrame(self, self.left_frame_method) # Change the fnc later
        self.right_frame = UpRightFrame(self, ['MASTER', 'SLAVE'], self.radiobutton_var, self.radio_button_command)# Change the fnc later

    def left_frame_method(self)->None:
        try:
            self.ros_srv_send_data("Connect")
        except Exception as e: 
            print(e)

    def button_drone(self) -> None:
        match self.button_drone_id:
            case 0:
                self.segments_frame.change_drone_color()
                try: self.frame_graph.grid_forget()
                except: pass
                try: self.frame_graph_graph1_master.grid_forget()
                except: pass

                self.frame_drone = ctk.CTkFrame(self, fg_color = DARKGRAY, bg_color = DARKGRAY)
                self.frame_drone.grid(row = 1, column = 1, sticky = 'nswe')
                self.frame_drone.rowconfigure(0, weight = 1, uniform = 'a')     #
                self.frame_drone.columnconfigure(0, weight = 5, uniform = 'a')  #LeftButtons
                self.frame_drone.columnconfigure(1, weight = 17, uniform = 'a') #VideoStreaming
                #self.frame_drone.columnconfigure(2, weight = 6, uniform = 'a')  #StatusLabels
                self.frame_drone_video = ctk.CTkFrame(self.frame_drone, fg_color = DARKGRAY)
                self.frame_drone_video.grid(row = 0, column = 1, sticky = 'nswe')
                self.label_video = LabelVideo(self.frame_drone_video)

                self.frame_sensor_indicator = ctk.CTkFrame(self.frame_drone, fg_color = DARKGRAY)
                self.frame_sensor_indicator.grid(row = 0, column = 0, sticky = 'nswe')
                self.textbox_indicators = TexBoxSensorIndicator(self.frame_sensor_indicator)

                if self.button_graph_id == 0:
                    self.button_drone_id = 2
                else:
                    self.button_drone_id = 2
                    self.button_graph_id = 1

            case 1:
                self.changing_color()
    
                try: self.frame_graph_graph1_master.grid_forget()
                except: pass
                try: self.frame_graph.grid_forget()
                except: pass

                self.frame_drone.grid(row = 1, column = 1, sticky = 'nswe')

                if self.button_graph_id == 0:
                    self.button_drone_id = 2
                else:
                    self.button_drone_id = 2
                    self.button_graph_id = 1
                self.ros_srv_send_data("CamaraImg")

    def button_graph(self) -> None:
        match self.button_graph_id: 
            case 0:
                # Creating Graph Frame
                color = DARKGRAY
                self.frame_graph = ctk.CTkFrame(self, fg_color = DARKGRAY)
                #self.frame_graph.grid(row = 1, column = 1, sticky = 'nswe')
                self.frame_graph.rowconfigure(0, weight = 1, uniform = 'a')
                self.frame_graph.rowconfigure(1, weight = 1, uniform = 'a')
                self.frame_graph.columnconfigure(0, weight = 1, uniform = 'a')
                self.frame_graph.columnconfigure(1, weight = 1, uniform = 'a')
                # For altitude Master
                self.frame_graph_graph1_master = ctk.CTkFrame(self, fg_color = color)
                #self.frame_graph_graph1_master.grid(row = 1, column = 1, sticky = 'nswe') 
                self.graph1_master = GraphPanel(self.frame_graph_graph1_master, 
                                                text = self.var_graph[5],
                                                external_update_graph = True)
                self.graph_buttons1_master = GraphButtons(self.graph1_master, self.var_graph[5], self.commands_button_graph,True)
                self.button_command_graph_system_variable('THROTTLEM')
                self.button_command_graph_setpoint('THROTTLEM')
                # For altitude Slave

                self.frame_graph_graph1 = ctk.CTkFrame(self.frame_graph, fg_color = color, bg_color = color)
                self.frame_graph_graph1.grid(row =0, column = 0, sticky = 'nswe') 
                self.graph1 = GraphPanel(self.frame_graph_graph1, 
                                        text = self.var_graph[0],
                                        external_update_graph = True)
                self.graph_buttons1 = GraphButtons(self.graph1, self.var_graph[0], self.commands_button_graph, True)

                self.button_command_graph_system_variable('THROTTLE')
                self.button_command_graph_setpoint('THROTTLE')

                # For YAW

                self.frame_graph_graph2 = ctk.CTkFrame(self.frame_graph, fg_color = color)
                self.frame_graph_graph2.grid(row =0, column = 1, sticky  = 'nswe')
                self.graph2 = GraphPanel(self.frame_graph_graph2, 
                                        text = self.var_graph[1],
                                        external_update_graph = True)
                self.graph_buttons2 = GraphButtons(self.graph2, self.var_graph[1], self.commands_button_graph, True)
                self.button_command_graph_system_variable('YAW')
                self.button_command_graph_setpoint('YAW')

                # For ROLL
                self.frame_graph_graph3 = ctk.CTkFrame(self.frame_graph, fg_color = color)
                self.frame_graph_graph3.grid(row =1, column = 0, sticky = 'nswe')
                self.graph3 = GraphPanel(self.frame_graph_graph3, 
                                        text = self.var_graph[2],
                                        external_update_graph = True)
                self.graph_buttons3 = GraphButtons(self.graph3, self.var_graph[2], self.commands_button_graph, True)
                self.button_command_graph_system_variable('ROLL')
                self.button_command_graph_setpoint('ROLL')

                # For PITCH
                self.frame_graph_graph4 = ctk.CTkFrame(self.frame_graph, fg_color = color)
                self.frame_graph_graph4.grid(row =1, column = 1, sticky = 'nswe')
                self.graph4 = GraphPanel(self.frame_graph_graph4, 
                                        text = self.var_graph[3],
                                        external_update_graph = True)
                self.graph_buttons4 = GraphButtons(self.graph4,self.var_graph[3], self.commands_button_graph, True)
                self.button_command_graph_system_variable('PITCH')
                self.button_command_graph_setpoint('PITCH')

                self.changing_color()
                try: self.frame_drone.grid_forget()
                except: pass
                if self.radiobutton_var.get() == 'MASTER':
                    self.frame_graph_graph1_master.grid(row = 1, column = 1, sticky = 'nswe')
                
                elif self.radiobutton_var.get() == 'SLAVE':
                    self.frame_graph.grid(row = 1, column = 1, sticky = 'nswe')
                
                self.button_graph_id = 2
                self.button_drone_id = 1
                self.ros_srv_send_data("UpdateGraph")
            case 1:
                try: self.frame_drone.grid_forget()
                except: pass
                self.changing_color()

                if self.radiobutton_var.get() == 'MASTER':
                    self.frame_graph_graph1_master.grid(row = 1, column = 1, sticky = 'nswe')
                
                elif self.radiobutton_var.get() == 'SLAVE':
                    self.frame_graph.grid(row = 1, column = 1, sticky = 'nswe')
                
                self.button_graph_id = 2
                self.button_drone_id = 1
                self.ros_srv_send_data("UpdateGraph")

    def new_window_method(self) -> None: # Used in WindowDroneMaster
        self.master_variable = False
    
    def pitch_roll_method(self, variable: str, activation: bool) -> None:
        print(f"var:{variable} / act:{activation}")
        self.ros_srv_send_movement(variable, activation)

    def yaw_throttle_method(self, variable: str, activation: bool) -> None:
        print(f"var:{variable} / act:{activation}")
        self.ros_srv_send_movement(variable, activation)

    def tkoff_land_command(self, variable: str) -> None:
        
        if variable == 'TKOFF':
            self.ros_srv_send_data("TkOff")

        if variable == 'LAND':
            print("Send Land")
            self.ros_srv_send_data("Land")
    
    def radio_button_command(self) -> None:
        variable = self.radiobutton_var.get()
        if variable == 'MASTER':
            #self.switch_start_var.set(False)
            #del self.labels_frame.switch_frame # Disappears the switch
            self.button_drone() # Calls to drone frame
            if not self.master_variable:
                self.drone_control_window = WindowDroneMaster(self, self.new_window_method, 
                                                            self.pitch_roll_list, 
                                                            self.yaw_throttle_list, 
                                                            self.resources_pitch_roll, 
                                                            self.resources_yaw_throttle,
                                                            self.pitch_roll_method,
                                                            self.yaw_throttle_method,
                                                            self.speed_slider,
                                                            self.ros_srv_send_speed
                                                            )
                self.master_variable = True
            #self.reset_parameters('THROTTLE')
            self.ros_srv_send_data("Master")
        elif variable == 'SLAVE':
            #self.ros_srv_send_data("Slave")
            #self.labels_frame.switch_frame # It makes appears the switch
            #self.switch_start_var.set(False)
            self.button_drone() # Calls to drone frame
            if self.master_variable:
                del self.drone_control_window.window
                self.master_variable = False
            self.ros_srv_send_data("Slave")
        
        if self.creation_button_activated_throttle:
            self.button_command_graph_external_graphic('THROTTLE')
        elif self.creation_button_activated_throttle_master:
            self.button_command_graph_external_graphic('THROTTLEM')
        elif self.creation_button_activated_yaw:
            self.button_command_graph_external_graphic('YAW')
        elif self.creation_button_activated_roll:
            self.button_command_graph_external_graphic('ROLL')
        elif self.creation_button_activated_pitch:
            self.button_command_graph_external_graphic('PITCH')

    def check_command(self) -> None:
        str_variable = ""
        j = 0
        for i in self.labels_name:
            if self.labels_dict[i].get():
                str_variable += str(i)+": "+self.labels_data[j]+"\n"
            j+=1
        self.textbox_indicators.update_sensors(str_variable)

    def battery_command(self) -> None:

        battery = self.progress_battery.get()
        color = self.up_frame.battery_color

        if battery > 0.5 and color != PROGRESSCOLOR:
            self.up_frame.battery_color = PROGRESSCOLOR
        
        elif battery > 0.2 and battery < 0.5 and color != YELLOWBATTERY:
            self.up_frame.battery_color = YELLOWBATTERY

        elif battery < 0.2  and color != REDBATTERY:
            self.up_frame.battery_color = REDBATTERY
    
    def switch_command(self, *args) -> None:
        #activation_list_key = ("THROTTLE", "YAW", "ROLL", "PITCH")
        # Error: redundant code can be simplified with two for loops
        # change that later
        key = (self.activate_throttle, self.activate_yaw, self.activate_roll, self.activate_pitch)
        if not self.switch_start_var.get():
            activate_variables = {
                (False, False, False, False): lambda: [self.button_command_graph_activate(activation) for activation in ("THROTTLE", "YAW", "ROLL", "PITCH")],
                (True, False, False, False): lambda: [self.button_command_graph_activate(activation) for activation in ("YAW", "ROLL", "PITCH")], 
                (False, True, False, False): lambda: [self.button_command_graph_activate(activation) for activation in ("THROTTLE", "ROLL", "PITCH")],
                (False, False, True, False): lambda: [self.button_command_graph_activate(activation) for activation in ("THROTTLE", "YAW", "PITCH")],
                (False, False, False, True): lambda: [self.button_command_graph_activate(activation) for activation in ("THROTTLE", "YAW", "ROLL")],
                (True, True, False, False): lambda: [self.button_command_graph_activate(activation) for activation in ("ROLL", "PITCH")],
                (True, False, True, False): lambda: [self.button_command_graph_activate(activation) for activation in ("YAW", "PITCH")],
                (True, False, False, True): lambda:[self.button_command_graph_activate(activation) for activation in ("YAW", "ROLL")],
                (False, True, True, False): lambda: [self.button_command_graph_activate(activation) for activation in ("THROTTLE", "PITCH")],
                (False, True, False, True): lambda: [self.button_command_graph_activate(activation) for activation in ("THROTTLE", "ROLL")],
                (False, False, True, True): lambda: [self.button_command_graph_activate(activation) for activation in ("THROTTLE", "YAW")],
                (False, True, True, True): lambda: self.button_command_graph_activate("THROTTLE"),
                (True, False, True, True): lambda: self.button_command_graph_activate("YAW"),
                (True, True, False, True): lambda: self.button_command_graph_activate("ROLL"),
                (True, True, True, False): lambda: self.button_command_graph_activate("PITCH"),
                (True, True, True, True): lambda: self.button_command_graph_activate("NONE")
            }
            if key in activate_variables:
                activate_variables[key]()
        else:
        #activation_list_key = ("THROTTLE", "YAW", "ROLL", "PITCH")
            deactivation_variables = {
                (False, False, False, False): lambda: self.button_command_graph_activate("NONE"),
                (True, False, False, False): lambda: self.button_command_graph_activate("THROTTLE"),
                (False, True, False, False): lambda: self.button_command_graph_activate("YAW"),
                (False, False, True, False): lambda: self.button_command_graph_activate("ROLL"),
                (False, False, False, True): lambda: self.button_command_graph_activate("PITCH"),
                (True, True, False, False): lambda: [self.button_command_graph_activate(activation) for activation in ("THROTTLE", "YAW")],
                (True, False, True, False): lambda: [self.button_command_graph_activate(activation) for activation in ("THROTTLE", "ROLL")],
                (True, False, False, True): lambda: [self.button_command_graph_activate(activation) for activation in ("THROTTLE", "PITCH")],
                (False, True, True, False): lambda: [self.button_command_graph_activate(activation) for activation in ("YAW", "ROLL")],
                (False, True, False, True): lambda: [self.button_command_graph_activate(activation) for activation in ("YAW", "PITCH")],
                (False, False, True, True): lambda: [self.button_command_graph_activate(activation) for activation in ("ROLL", "PITCH")],
                (False, True, True, True): lambda: [self.button_command_graph_activate(activation) for activation in ("YAW", "ROLL", "PITCH")],
                (True, False, True, True): lambda: [self.button_command_graph_activate(activation) for activation in ("THROTTLE", "ROLL", "PITCH")],
                (True, True, False, True): lambda: [self.button_command_graph_activate(activation) for activation in ("THROTTLE", "YAW", "PITCH")],
                (True, True, True, False): lambda: [self.button_command_graph_activate(activation) for activation in ("THROTTLE", "YAW", "ROLL")],
                (True, True, True, True): lambda: [self.button_command_graph_activate(activation) for activation in ("THROTTLE", "YAW", "ROLL", "PITCH")]}
            if key in deactivation_variables:
                deactivation_variables[key]()
    def button_command_graph_system_variable(self, graph: str) -> None:
        match graph:
            case 'THROTTLE':
                self.graph_buttons1.change_color('DATA')
                self.graph1.data_method()
                if self.creation_button_activated_throttle:
                    self.external_window_throttle.change_color('DATA')
                    self.external_window_throttle.show_plot('DATA')
            case 'THROTTLEM':
                self.graph_buttons1_master.change_color('DATA')
                self.graph1_master.data_method()
                if self.creation_button_activated_throttle_master:
                    self.external_window_throttle_master.change_color('DATA')
                    self.external_window_throttle_master.show_plot('DATA')
            case 'YAW':
                self.graph_buttons2.change_color('DATA')
                self.graph2.data_method()
                if self.creation_button_activated_yaw:
                    self.external_window_yaw.change_color('DATA')
                    self.external_window_yaw.show_plot('DATA')
            case 'ROLL':
                self.graph_buttons3.change_color('DATA')
                self.graph3.data_method()

                if self.creation_button_activated_roll:
                    self.external_window_roll.change_color('DATA')
                    self.external_window_roll.show_plot('DATA')
            case 'PITCH':
                self.graph_buttons4.change_color('DATA')
                self.graph4.data_method()

                if self.creation_button_activated_pitch:
                    self.external_window_pitch.change_color('DATA')
                    self.external_window_pitch.show_plot('DATA')
    
    def button_command_graph_setpoint(self, graph: str) -> None :
        match graph:
            case 'THROTTLE':
                self.graph_buttons1.change_color('SETPOINT')
                self.graph1.setpoint_method()
                if self.creation_button_activated_throttle:
                    self.external_window_throttle.change_color('SETPOINT')
                    self.external_window_throttle.show_plot('SETPOINT')
            case 'THROTTLEM':
                self.graph_buttons1_master.change_color('SETPOINT')
                self.graph1_master.setpoint_method()
                if self.creation_button_activated_throttle_master:
                    self.external_window_throttle_master.change_color('SETPOINT')
                    self.external_window_throttle_master.show_plot('SETPOINT')
            case 'YAW':
                self.graph_buttons2.change_color('SETPOINT')
                self.graph2.setpoint_method()
                if self.creation_button_activated_yaw:
                    self.external_window_throttle.change_color('SETPOINT')
                    self.external_window_throttle.show_plot('SETPOINT')
            case 'ROLL':
                self.graph_buttons3.change_color('SETPOINT')
                self.graph3.setpoint_method()
                if self.creation_button_activated_roll:
                    self.external_window_roll.change_color('SETPOINT')
                    self.external_window_roll.show_plot('SETPOINT')
            case 'PITCH':
                self.graph_buttons4.change_color('SETPOINT')
                self.graph4.setpoint_method()
                if self.creation_button_activated_pitch:
                    self.external_window_pitch.change_color('SETPOINT')
                    self.external_window_pitch.show_plot('SETPOINT')

    def button_command_graph_error(self, graph: str) -> None:
        match graph:
            case 'THROTTLE':
                self.graph_buttons1.change_color('ERROR')
                self.graph1.error_method()
                if self.creation_button_activated_throttle:
                    self.external_window_throttle.change_color('ERROR')
                    self.external_window_throttle.show_plot('ERROR')
            case 'THROTTLEM':
                self.graph_buttons1_master.change_color('ERROR')
                self.graph1_master.error_method()
                if self.creation_button_activated_throttle_master:
                    self.external_window_throttle_master.change_color('ERROR')
                    self.external_window_throttle_master.show_plot('ERROR')
            case 'YAW':
                self.graph_buttons2.change_color('ERROR')
                self.graph2.error_method()
                if self.creation_button_activated_yaw:
                    self.external_window_yaw.change_color('ERROR')
                    self.external_window_yaw.show_plot('ERROR')
            case 'ROLL':
                self.graph_buttons3.change_color('ERROR')
                self.graph3.error_method()
                if self.creation_button_activated_roll:
                    self.external_window_roll.change_color('ERROR')
                    self.external_window_roll.show_plot('ERROR')
            case 'PITCH':
                self.graph_buttons4.change_color('ERROR')
                self.graph4.error_method()
                if self.creation_button_activated_pitch:
                    self.external_window_pitch.change_color('ERROR')
                    self.external_window_pitch.show_plot('ERROR')

    def button_command_graph_output(self, graph: str) -> None:
        match graph:
            case 'THROTTLE':
                self.graph_buttons1.change_color('OUTPUT')
                self.graph1.output_method()
                if self.creation_button_activated_throttle:
                    self.external_window_throttle.change_color('OUTPUT')
                    self.external_window_throttle.show_plot('OUTPUT')
            case 'THROTTLEM':
                self.graph_buttons1_master.change_color('OUTPUT')
                self.graph1_master.output_method()
                if self.creation_button_activated_throttle_master:
                    self.external_window_throttle_master.change_color('OUTPUT')
                    self.external_window_throttle_master.show_plot('OUTPUT')
            case 'YAW':
                self.graph_buttons2.change_color('OUTPUT')
                self.graph2.output_method()
                if self.creation_button_activated_yaw:
                    self.external_window_yaw.change_color('OUTPUT')
                    self.external_window_yaw.show_plot('OUTPUT')
            case 'ROLL':
                self.graph_buttons3.change_color('OUTPUT')
                self.graph3.output_method()
                if self.creation_button_activated_roll:
                    self.external_window_roll.change_color('OUTPUT')
                    self.external_window_roll.show_plot('OUTPUT')
            case 'PITCH':
                self.graph_buttons4.change_color('OUTPUT')
                self.graph4.output_method()
                if self.creation_button_activated_pitch:
                    self.external_window_pitch.change_color('OUTPUT')
                    self.external_window_pitch.show_plot('OUTPUT')

    def button_command_graph_activate(self, graph:str)->None:
        match graph:
            case 'THROTTLE':
                self.activate_throttle = not self.activate_throttle
                self.graph_buttons1.change_color('ACTIVATE')
                self.ros_srv_send_data("ThrottleSlave")
                if self.creation_button_activated_throttle:
                    self.external_window_throttle.change_color('ACTIVATE')
            case 'THROTTLEM':
                self.activate_throttle_master = not self.activate_throttle_master
                self.graph_buttons1_master.change_color('ACTIVATE')
                self.ros_srv_send_data("ThrottleMaster")
                if self.creation_button_activated_throttle_master:
                    self.external_window_throttle_master.change_color('ACTIVATE')
            case 'YAW':
                self.activate_yaw = not self.activate_yaw
                self.graph_buttons2.change_color('ACTIVATE')
                self.ros_srv_send_data("YawSlave")
                if self.creation_button_activated_yaw:
                    self.external_window_yaw.change_color('ACTIVATE')
            case 'ROLL':
                self.activate_roll = not self.activate_roll
                self.graph_buttons3.change_color('ACTIVATE')
                self.ros_srv_send_data("RollSlave")
                if self.creation_button_activated_roll:
                    self.external_window_roll.change_color('ACTIVATE')
            case 'PITCH':
                self.activate_pitch = not self.activate_pitch
                self.graph_buttons4.change_color('ACTIVATE')
                self.ros_srv_send_data("PitchSlave")
                if self.creation_button_activated_pitch:
                    self.external_window_pitch.change_color('ACTIVATE')
            case _:
                print("ANY ARGUMENT GIVEN :0") 

    def button_command_graph_external_graphic(self, graph: str) -> None:
        match graph:
            case 'THROTTLE':
                if not self.creation_button_activated_pitch and not self.creation_button_activated_roll and not self.creation_button_activated_yaw and not self.creation_button_activated_throttle_master:
                    if not self.creation_button_activated_throttle:
                        self.graph_buttons1.change_color('CREATION')
                        self.graph1.external_graph_method()
                        self.external_window_throttle = WindowGraph(self, self.graph_buttons1.get_buttons_activated(),
                                                                    ('DATA', 'SETPOINT', 'ERROR', 'OUTPUT', 'ACTIVATE','CREATION'),
                                                                    'THROTTLE',
                                                                    self.commands_button_graph,
                                                                    self.throttle_drone,
                                                                    self.vcmd,
                                                                    self.reset_parameters,
                                                                    self.ros_srv_proportional,
                                                                    self.ros_srv_integral,
                                                                    self.ros_srv_derivative,
                                                                    self.throttle_drone_labels,
                                                                    update_externally = True
                                                                    )
                        self.creation_button_activated_throttle = True
                    else:
                        self.graph_buttons1.change_color('CREATION')
                        self.graph1.external_graph_method()
                        self.creation_button_activated_throttle = False
                        self.external_window_throttle.new_window.destroy()
            
            case 'THROTTLEM':
                if not self.creation_button_activated_pitch and not self.creation_button_activated_roll and not self.creation_button_activated_yaw and not self.creation_button_activated_throttle:
                    if not self.creation_button_activated_throttle_master:
                        self.graph_buttons1_master.change_color('CREATION')
                        self.graph1_master.external_graph_method()
                        self.external_window_throttle_master = WindowGraph(self, self.graph_buttons1_master.get_buttons_activated(),
                                                                    ('DATA', 'SETPOINT', 'ERROR', 'OUTPUT', 'ACTIVATE','CREATION'),
                                                                    'THROTTLEM',
                                                                    self.commands_button_graph,
                                                                    self.throttle_master_drone,
                                                                    self.vcmd,
                                                                    self.reset_parameters,
                                                                    self.ros_srv_proportional,
                                                                    self.ros_srv_integral,
                                                                    self.ros_srv_derivative,
                                                                    self.throttle_master_drone_labels,
                                                                    update_externally = True
                                                                    )
                        self.creation_button_activated_throttle_master = True
                    else: 
                        self.graph_buttons1_master.change_color('CREATION')
                        self.graph1_master.external_graph_method()
                        self.creation_button_activated_throttle_master = False
                        self.external_window_throttle_master.new_window.destroy()
            case 'YAW':
                if not self.creation_button_activated_throttle and not self.creation_button_activated_pitch and not self.creation_button_activated_roll and not self.creation_button_activated_throttle_master:
                    if not self.creation_button_activated_yaw:
                        self.graph_buttons2.change_color('CREATION')
                        self.graph2.external_graph_method()
                        self.external_window_yaw = WindowGraph(self, self.graph_buttons2.get_buttons_activated(),
                                                                    ('DATA', 'SETPOINT', 'ERROR', 'OUTPUT', 'ACTIVATE', 'CREATION'),
                                                                    'YAW',
                                                                    self.commands_button_graph,
                                                                    self.yaw_drone,
                                                                    self.vcmd,
                                                                    self.reset_parameters,
                                                                    self.ros_srv_proportional,
                                                                    self.ros_srv_integral,
                                                                    self.ros_srv_derivative,
                                                                    self.yaw_drone_labels,
                                                                    update_externally = True)
                        self.creation_button_activated_yaw = True
                    else: 
                        self.graph_buttons2.change_color('CREATION')
                        self.graph2.external_graph_method()
                        self.creation_button_activated_yaw = False
                        self.external_window_yaw.new_window.destroy()
            case 'ROLL':
                if not self.creation_button_activated_throttle and not self.creation_button_activated_pitch and not self.creation_button_activated_yaw and not self.creation_button_activated_throttle_master:
                    if not self.creation_button_activated_roll:
                        self.graph_buttons3.change_color('CREATION')
                        self.graph3.external_graph_method()
                        self.external_window_roll = WindowGraph(self, self.graph_buttons3.get_buttons_activated(),
                                                                    ('DATA', 'SETPOINT', 'ERROR', 'OUTPUT', 'ACTIVATE', 'CREATION'),
                                                                    'ROLL',
                                                                    self.commands_button_graph,
                                                                    self.roll_drone,
                                                                    self.vcmd,
                                                                    self.reset_parameters,
                                                                    self.ros_srv_proportional,
                                                                    self.ros_srv_integral,
                                                                    self.ros_srv_derivative,
                                                                    self.roll_drone_labels,
                                                                    update_externally = True)
                        self.creation_button_activated_roll = True
                    else:
                        self.graph_buttons3.change_color('CREATION')
                        self.graph3.external_graph_method()
                        self.creation_button_activated_roll = False
                        self.external_window_roll.new_window.destroy()
            case 'PITCH':
                if not self.creation_button_activated_throttle and not self.creation_button_activated_roll and not self.creation_button_activated_yaw and not self.creation_button_activated_throttle_master:
                    if not self.creation_button_activated_pitch:
                        self.graph_buttons4.change_color('CREATION')
                        self.graph4.external_graph_method()
                        self.external_window_pitch = WindowGraph(self, self.graph_buttons4.get_buttons_activated(),
                                                                    ('DATA', 'SETPOINT', 'ERROR', 'OUTPUT', 'ACTIVATE', 'CREATION'),
                                                                    'PITCH',
                                                                    self.commands_button_graph,
                                                                    self.pitch_drone,
                                                                    self.vcmd,
                                                                    self.reset_parameters,
                                                                    self.ros_srv_proportional,
                                                                    self.ros_srv_integral,
                                                                    self.ros_srv_derivative,
                                                                    self.pitch_drone_labels,
                                                                    update_externally = True)
                        self.creation_button_activated_pitch = True
                    else:
                        self.graph_buttons4.change_color('CREATION')
                        self.graph4.external_graph_method()
                        self.creation_button_activated_pitch = False
                        self.external_window_pitch.new_window.destroy()

    def reset_parameters(self, variable) -> None:
        match variable: 
            case 'THROTTLE':
                self.throttle_drone['proportional'].set(DEFAULT_PROPORTIONAL_VALUE_THROTTLE)
                self.throttle_drone['derivative'].set(DEFAULT_DERIVATIVE_VALUE_THROTTLE)
                self.throttle_drone['integral'].set(DEFAULT_INTEGRAL_VALUE_THROTTLE)
                self.throttle_drone['system_variable'].set(DEFAULT_THROTTLE)
            case 'THROTTLEM':
                self.throttle_master_drone['proportional'].set(DEFAULT_PROPORTIONAL_VALUE_THROTTLE_MASTER)
                self.throttle_master_drone['derivative'].set(DEFAULT_DERIVATIVE_VALUE_THROTTLE_MASTER)
                self.throttle_master_drone['integral'].set(DEFAULT_INTEGRAL_VALUE_THROTTLE_MASTER)
                self.throttle_master_drone['system_variable'].set(DEFAULT_THROTTLE_MASTER)
            case 'YAW':
                self.yaw_drone['proportional'].set(DEFAULT_PROPORTIONAL_VALUE_YAW)
                self.yaw_drone['derivative'].set(DEFAULT_DERIVATIVE_VALUE_YAW)
                self.yaw_drone['integral'].set(DEFAULT_INTEGRAL_VALUE_YAW)
                self.yaw_drone['system_variable'].set(DEFAULT_YAW)
            case 'ROLL':
                self.roll_drone['proportional'].set(DEFAULT_PROPORTIONAL_VALUE_ROLL)
                self.roll_drone['derivative'].set(DEFAULT_DERIVATIVE_VALUE_ROLL)
                self.roll_drone['integral'].set(DEFAULT_INTEGRAL_VALUE_ROLL)
                self.roll_drone['system_variable'].set(DEFAULT_ROLL)
            case 'PITCH':
                self.pitch_drone['proportional'].set(DEFAULT_PROPORTIONAL_VALUE_PITCH)
                self.pitch_drone['derivative'].set(DEFAULT_DERIVATIVE_VALUE_PITCH)
                self.pitch_drone['integral'].set(DEFAULT_INTEGRAL_VALUE_PITCH)
                self.pitch_drone['system_variable'].set(DEFAULT_PITCH)

    def keyboard_method(self) -> None:

        def on_press(key):
            #global KEYBOARDEVENTS
            try: 
                if key == Key.up:
                    self.ros_srv_send_movement('FORWARD',True)
                    print('FORWARD')
                elif key == Key.down:
                    self.ros_srv_send_movement('BACKWARD', True)
                    print('BACKWARD')
                elif key == Key.left:
                    self.ros_srv_send_movement('LEFT', True)
                    print('LEFT')
                elif key == Key.right:
                    self.ros_srv_send_movement('RIGHT', True)
                    print('RIGHT')
                elif isinstance(key, KeyCode):
                    if key.char == 'a':
                        self.ros_srv_send_movement('TURNL', True)
                        print('TURNL')
                    elif key.char == 'd':
                        self.ros_srv_send_movement('TURNR', True)
                        print('TURNR')
                    elif key.char == 'w':
                        self.ros_srv_send_movement('UP', True)
                        print('UP')
                    elif key.char == 's':
                        self.ros_srv_send_movement('DOWN', True)
                        print('DOWN')
                    elif key.char == 'i':
                        self.ros_srv_send_data('TkOff')
                        print('TKOFF')
                    elif key.char == 'q':
                        self.ros_srv_send_data('Land')
                        print('LAND')
            except Exception as e: 
                print(f'[on_press] Error: {e}')
        def on_release(key):
            try: 
                if key == Key.up:
                    self.ros_srv_send_movement('FORWARD', False)
                    print('FORWARDF')
                elif key == Key.down:
                    self.ros_srv_send_movement('BACKWARD', False)
                    print('BACKWARDF')
                elif key == Key.left:
                    self.ros_srv_send_movement('LEFT', False)
                    print('LEFTF')
                elif key == Key.right:
                    self.ros_srv_send_movement('RIGHT', False)
                    print('RIGHTF')
                elif isinstance(key, KeyCode):
                    if key.char == 'a':
                        self.ros_srv_send_movement('TURNL', False)
                        print('TURNLF')
                    elif key.char == 'd':
                        self.ros_srv_send_movement("TURNR", False)
                        print('TURNRF')
                    elif key.char == 'w':
                        self.ros_srv_send_movement("UP", False)
                        print('UPF')
                    elif key.char == 's':
                        self.ros_srv_send_movement("DOWN", False)
                        print('DOWNF')
            except Exception as e: 
                print(f'[on_release] Error: {e}')
        with Listener (on_press = on_press, on_release = on_release) as listener:
            listener.join()
def main(args = None):
    Window(args)

if __name__ == '__main__':
    main()