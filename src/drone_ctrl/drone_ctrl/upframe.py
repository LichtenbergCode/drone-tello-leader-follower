#!/usr/bin/env python3
import customtkinter as ctk
from customtkinter import CTkImage
from drone_ctrl.settings import *
from drone_ctrl.elements import *
from typing import Callable

class UpFrame(ctk.CTkFrame):
    def __init__(self, parent:ctk.CTkFrame, flt_variable: ctk.DoubleVar):
        super().__init__(master = parent, 
                        fg_color = GRAY, 
                        corner_radius = 0)
        
        self.grid(column = 1, row = 0, sticky = 'nswe')
        self.battery = Battery(self, flt_variable)
    
    @property
    def battery_color(self) -> str:
        return self.battery.color_progress

    @battery_color.setter
    def change_color_battery(self, color: str) -> None:
        self.battery.color_progress = color

class UpLeftFrame(ctk.CTkFrame):
    def __init__(self, parent:ctk.CTkFrame, button_function:Callable[[],None]):
        super().__init__(master = parent, 
                        fg_color = DARKGRAY)
        self.grid(column = 0, row = 0, sticky = 'nswe')

        #image_wifi_tk = ctk.CTkImage()
        image_wifi = Image.open('./src/drone_ctrl/drone_ctrl/Resources/wifi_E1E1E1.png')
        image_wifi_tk = CTkImage(image_wifi, size = (75, 51))

        WifiButton(self, button_function, image_wifi_tk)

class UpRightFrame(ctk.CTkFrame):
    def __init__(self, parent: ctk.CTkFrame, labels: List[str], var: ctk.StringVar,  button_function: Callable[[],None]) -> None:
        super().__init__(master = parent,
                        fg_color = DARKGRAY)
        self.grid(column = 2, row = 0, sticky = 'nswe')
        [RadioButtonDroneConfiguration(self, i, var, button_function) for i in labels]