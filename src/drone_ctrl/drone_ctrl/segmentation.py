#!/usr/bin/env python3

import customtkinter as ctk 
from customtkinter import CTkImage
from drone_ctrl.settings import *
from drone_ctrl.elements import *
from PIL import Image
from typing import Callable, List, Dict, Any

class Segmentation:
    def __init__(self, parent: ctk.CTkFrame, segmented_buttons: Tuple[Callable[[], None]], imgs: Tuple[str], tkoff_land_command: Callable[[], None]) -> None:
        
        self.drone_image_direction = imgs[0]
        self.graph_image_direction = imgs[1]

        self.image_drone = Image.open(self.drone_image_direction)#.resize((65, 65))
        self.image_graph = Image.open(self.graph_image_direction)

        self.image_drone_tk = CTkImage(self.image_drone, size = (65, 65))
        self.image_graph_tk = CTkImage(self.image_graph, size = (80, 60))

        segmented_frame = ctk.CTkFrame(master = parent, 
                        fg_color = GRAY)
        segmented_frame.grid(column = 0, row = 1,  sticky = 'nswe')
        Buttons(segmented_frame, tkoff_land_command)

        self.drone_button = SegmentedButton(segmented_frame, 0, 1, '', segmented_buttons[0], self.image_drone_tk)
        self.graph_button = SegmentedButton(segmented_frame, 0, 0, '', segmented_buttons[1], self.image_graph_tk)

    def change_drone_color(self):
        self.drone_button.change_colors()
    def change_graph_color(self):
        self.graph_button.change_colors()

class Labels:
    def __init__(self, parent: ctk.CTkFrame, names: Tuple[str], names_dict: Dict[str,ctk.BooleanVar], check_command:Callable[[], None] , switch_name:str, switch_var: ctk.BooleanVar, switch_command:Callable[[], None]) -> None:
        
        labels_frame = ctk.CTkFrame(parent, fg_color = GRAY)
        labels_frame.grid(column = 2, row = 1, sticky = 'nswe')
        [CheckButtonParameter(labels_frame, i, names_dict[i], check_command) for i in names]
        self.switch = Switch(labels_frame, switch_name, switch_var, switch_command)
        
    @property
    def switch_frame(self) -> None:
        self.switch.frame
    
    @switch_frame.deleter
    def switch_frame(self) -> None:
        del self.switch.frame

class WindowDroneMaster:
    def __init__(self, parent:ctk.CTkFrame, 
                window_close_method: Callable[[], None], 
                pitch_roll_list_nms: Tuple[str], 
                yaw_throttle_list_nms: Tuple[str],
                resources_pitch_roll: Tuple[str],
                resources_yaw_throttle: Tuple[str],
                pitch_roll_method: Callable[[],None],
                yaw_throttle_method: Callable[[], None],
                slider_variable: ctk.IntVar,
                slider_method: Callable):

        self.window_close_method = window_close_method

        self.new_window = ctk.CTkToplevel(parent)
        self.new_window.geometry('480x150')
        self.new_window.resizable(False, False)
        self.new_window.title('Drone Control')
        self.new_window.protocol("WM_DELETE_WINDOW", self.close_window_method)
        self.slider_method = slider_method

        self.frame = ctk.CTkFrame(self.new_window, fg_color = DARKGRAY)
        self.frame.pack(fill = 'both', expand = True)
        self.frame.columnconfigure((0,1), weight = 5, uniform = 'a')
        self.frame.rowconfigure(0, weight = 5, uniform = 'a')
        self.frame.rowconfigure(1, weight = 1, uniform = 'a')
        
        self.frame_pitch_roll = ctk.CTkFrame(self.frame, fg_color = DARKGRAY)
        self.frame_pitch_roll.grid(column = 0, row = 0, sticky = 'nswe')
        
        self.frame_yaw_throttle = ctk.CTkFrame(self.frame, fg_color = DARKGRAY)
        self.frame_yaw_throttle.grid(column = 1, row = 0, sticky = 'nswe')
        
        self.frame_slider = ctk.CTkFrame(self.frame, fg_color = DARKGRAY)
        self.frame_slider.grid(column = 0, row = 1, columnspan = 2, sticky = 'nswe')

        FrameControl(self.frame_pitch_roll, 
                    'BLUE', 
                    resources_pitch_roll, 
                    pitch_roll_list_nms, 
                    pitch_roll_method, 
                    DARKGRAY)
        FrameControl(self.frame_yaw_throttle, 
                    'ORANGE', 
                    resources_yaw_throttle, 
                    yaw_throttle_list_nms, 
                    yaw_throttle_method, 
                    DARKGRAY)
        SpeedSlider(self.frame_slider, 
                    slider_variable, 
                    DARKGRAY, 
                    self.slider_method)
    
    @property
    def window(self) -> str:
        return 'Drone Control Window'

    @window.deleter
    def window(self) -> None:
        self.new_window.destroy()
    
    def close_window_method(self):
        self.window_close_method()
        self.new_window.destroy()
        