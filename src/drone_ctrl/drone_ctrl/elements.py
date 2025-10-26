#!/usr/bin/env python3

import customtkinter as ctk 
from customtkinter import CTkImage
from drone_ctrl.settings import *
from typing import Callable, List, Tuple
from PIL import Image, ImageTk

class BasicFrame(ctk.CTkFrame):
    def __init__(self, parent):
        super().__init__(master = parent, fg_color= GRAY)
        self.pack(padx = 5, pady = 5)
    
    @property
    def frame(self) -> None:
        self.pack(padx = 5, pady = 5)

    @frame.deleter
    def frame(self) -> None:
        self.pack_forget()

class SegmentedButton(ctk.CTkButton): #Created to drone and graph buttons
    def __init__(self, parent:ctk.CTkFrame, radius: int, pad: int, text:str, command: Callable[[],None], image:ImageTk.PhotoImage) -> None:
        super().__init__(master = parent, 
                        bg_color = GRAY,
                        fg_color=   GRAY, 
                        corner_radius = radius,
                        #width = 120,
                        height = 69,
                        hover_color = SEGMENTEDPRESSED,
                        text = text,
                        command = command,
                        image = image)
        self.button_id = 0 
        self.pack(pady = pad, expand = False, fill = 'both') # padx = pad,
    
    def change_colors(self):
        if self.button_id == 0:
            self.configure(fg_color = SEGMENTEDPRESSED)
            self.button_id = 1
        else:
            self.configure(fg_color = GRAY)
            self.button_id = 0

class Buttons: #Created to Take Off and Land the drone 
    def __init__(self, parent:ctk.CTkFrame, tkoff_land_command: Callable[[], None]) -> None:
        self.frame_buttons = ctk.CTkFrame(parent, fg_color = GRAY)
        self.frame_buttons.pack(side = 'bottom')
        self.frame_buttons.columnconfigure((0, 1), weight = 1)
        button1 = ctk.CTkButton(self.frame_buttons, 
                                fg_color = SEGMENTEDPRESSED,
                                hover_color = HOVERFINISHBUTTONS, text = 'Tk Off',
                                command = lambda:tkoff_land_command('TKOFF'),
                                corner_radius = 8)
        button1.grid(column = 0, sticky = 'nswe')

        button2 = ctk.CTkButton(self.frame_buttons,
                                fg_color = SEGMENTEDPRESSED,
                                hover_color = HOVERFINISHBUTTONS, text = 'Land',
                                command = lambda:tkoff_land_command('LAND'),
                                corner_radius = 8)
        button2.grid(column = 1, sticky = 'nswe')

class Battery(ctk.CTkFrame):
    def __init__(self, parent: ctk.CTkFrame, flt_variable: ctk.DoubleVar):
        super().__init__(master = parent, fg_color='transparent')
        #self.progress = progress
        
        # Create a progress bar with specific properties
        self.progressbar = ctk.CTkProgressBar(self, width=50, 
                                        height=25,
                                        corner_radius = 0, 
                                        progress_color= PROGRESSCOLOR,
                                        fg_color=BACKPROGRESSCOLOR, 
                                        bg_color=BACKPROGRESSCOLOR,
                                        variable = flt_variable)
        self.progressbar.pack(side = 'left', pady=20)
        frame = ctk.CTkFrame(self, width = 6, height =14, fg_color = BACKPROGRESSCOLOR, bg_color = BACKPROGRESSCOLOR)
        frame.pack(side = 'left', pady = 20)

        self.place(relx = 0.5, rely = 0.05, anchor = 'n')
    
    @property
    def color_progress(self) -> str:
        return self.progressbar.cget("progress_color")

    @color_progress.setter
    def color_progress(self, color: str) -> None:
        self.progressbar.configure(progress_color = color)

class WifiButton(ctk.CTkButton):
    def __init__(self, parent, button_function:Callable[[],None], image: CTkImage):
        super().__init__(
                        parent, 
                        bg_color= GRAY, 
                        fg_color= DARKGRAY, 
                        hover_color = BLUE, 
                        #corner_radius = 20,
                        command = button_function,
                        text = '',
                        image = image)
        
        self.pack(expand = True, fill = 'both')
        self.button_id = True
        
    def change_color(self) -> None:
        if self.button_id:
            self.configure(fg_color = BLUE)
            self.button_id = True

class CheckButtonParameter:
    def __init__(self, parent: ctk.CTkFrame, label: str, bool_variable: ctk.BooleanVar, command: Callable[[], None]):
        self.parent = parent
        self.label = label
        self.checkbox = ctk.CTkCheckBox(parent, text = self.label, 
                                        variable = bool_variable, 
                                        hover_color = SEGMENTEDPRESSED , 
                                        fg_color = SEGMENTEDPRESSED, 
                                        checkmark_color = WHITE, 
                                        command = lambda: command())
        self.checkbox.pack(padx = 15, pady = 2)

class RadioButtonDroneConfiguration:
    def __init__(self, parent: ctk.CTkFrame, label:str, string_var:ctk.StringVar, command = Callable[[], None]):
        self.radio = ctk.CTkRadioButton(parent, text = label.capitalize(), variable = string_var, value = label, command = command, hover_color=BLUE, fg_color = BLUE)
        self.radio.pack(padx = 15, pady = 4)

class FrameControl(ctk.CTkFrame):
    def __init__(self, 
            root:ctk.CTkFrame, 
            button_style: str, 
            imgs: Tuple[str],
            nms: Tuple[str],
            command_press: Callable[[],None],
            frame_fg_color = str):
        super().__init__(root, fg_color = frame_fg_color)

        self.button_style = button_style

        self.path_button1 = imgs[0]
        self.path_button2 = imgs[1]
        self.path_button3 = imgs[2]
        self.path_button4 = imgs[3]

        self.button_nm_1 = nms[0]
        self.button_nm_2 = nms[1]
        self.button_nm_3 = nms[2]
        self.button_nm_4 = nms[3]

        self.command_button_pressed = command_press
        
        self.command_not_info = 'NONE'
        self.send_info = 'NONE'

        self.columnconfigure((0, 4), weight = 2, uniform='a')
        self.columnconfigure(2, weight = 9, uniform = 'a')
        self.columnconfigure((1,3), weight = 7, uniform = 'a')
        self.rowconfigure((0, 5), weight = 1, uniform = 'a')
        self.rowconfigure((1, 4), weight = 3, uniform = 'a')
        self.rowconfigure((2, 3), weight = 4, uniform = 'a')

        self.images()
        self.style()
        self.buttons()
        self.events()
        self.pack()

    def images(self):
        
        # Images # 
        image_first_button = Image.open(self.path_button1).resize((70, 70))
        image_second_button = Image.open(self.path_button2).resize((70, 70))
        image_third_button = Image.open(self.path_button3).resize((70, 70))
        image_fourth_button = Image.open(self.path_button4).resize((70, 70))

        self.image_first_button_tk = CTkImage(image_first_button, size = (40, 40))
        self.image_second_button_tk = CTkImage(image_second_button, size = (40, 40))
        self.image_third_button_tk = CTkImage(image_third_button, size = (40, 40))
        self.image_fourth_button_tk = CTkImage(image_fourth_button, size = (40, 40))
    
    def style(self):
        # Creating styles OwO
        self.corner_radius = 0.0
        if self.button_style == 'BLUE':
            self.color = BLUE
            self.hover_color = LIGHTBLUE
        if self.button_style == 'ORANGE':
            self.color = SEGMENTEDPRESSED
            self.hover_color = LIGHTORANGE

    def buttons(self) -> None:
        self.button_1 = ctk.CTkButton(self, 
                                    corner_radius = self.corner_radius, 
                                    fg_color = self.color, 
                                    hover_color = self.hover_color, 
                                    image = self.image_first_button_tk, 
                                    text = "")  
        
        self.button_2 = ctk.CTkButton(self, 
                                    corner_radius = self.corner_radius, 
                                    fg_color = self.color, 
                                    hover_color = self.hover_color, 
                                    image = self.image_second_button_tk, 
                                    text = "")
        
        self.button_3 = ctk.CTkButton(self, 
                                    corner_radius = self.corner_radius, 
                                    fg_color = self.color, 
                                    hover_color = self.hover_color, 
                                    image = self.image_third_button_tk, 
                                    text = "")
        
        self.button_4 = ctk.CTkButton(self, 
                                    corner_radius = self.corner_radius, 
                                    fg_color = self.color, 
                                    hover_color = self.hover_color, 
                                    image = self.image_fourth_button_tk, 
                                    text = "")

        self.button_1.grid(column = 2, row = 1, sticky = 'nswe', rowspan = 2, padx=8, pady = 5)
        self.button_2.grid(column = 1, row = 2, sticky = 'nswe', rowspan = 2)
        self.button_3.grid(column = 3, row =  2, sticky = 'nswe', rowspan = 2)
        self.button_4.grid(column = 2, row = 3, sticky = 'nswe', rowspan = 2, padx=8, pady = 5)

    def events(self) -> None:

            # Press
        self.button_1.bind('<ButtonPress-1>', lambda _: self.command_button_pressed(self.button_nm_1, True))
        self.button_2.bind('<ButtonPress-1>', lambda _: self.command_button_pressed(self.button_nm_3, True))
        self.button_3.bind('<ButtonPress-1>', lambda _: self.command_button_pressed(self.button_nm_4, True))
        self.button_4.bind('<ButtonPress-1>', lambda _: self.command_button_pressed(self.button_nm_2, True))
        
            # Release
        self.button_1.bind('<ButtonRelease-1>', lambda _: self.command_button_pressed(self.button_nm_1, False))
        self.button_2.bind('<ButtonRelease-1>', lambda _: self.command_button_pressed(self.button_nm_3, False))
        self.button_3.bind('<ButtonRelease-1>', lambda _: self.command_button_pressed(self.button_nm_4, False))
        self.button_4.bind('<ButtonRelease-1>', lambda _: self.command_button_pressed(self.button_nm_2, False))
    
    @property
    def colors(self) -> Tuple[str, str]:
        return self.color, self.hover_color
    
    @colors.setter
    def colors(self, hover_color: str, button_color:str) -> None:
        self.button_1.configure(fg_color = button_color, hover_color = hover_color)
        self.button_2.configure(fg_color = button_color, hover_color = hover_color)
        self.button_3.configure(fg_color = button_color, hover_color = hover_color)
        self.button_4.configure(fg_color = button_color, hover_color = hover_color)

class Switch(BasicFrame):
    def __init__(self, parent: ctk.CTkFrame, label: str, switch_var: ctk.BooleanVar, switch_command: Callable) -> None:
        super().__init__(parent)
        
        ctk.CTkSwitch(
                    self, 
                    text = label,
                    variable = switch_var ,
                    onvalue = True,
                    offvalue = False,
                    command = switch_command).pack(side = 'bottom', pady = 30)

class SliderPanel(BasicFrame):
    def __init__(self, 
                parent: ctk.CTkFrame , 
                text : str, 
                data_var, vcmd, 
                min_value, max_value,
                send_txt:str,
                gain_command: Callable[[str, float], None] ) -> None:
        super().__init__(parent = parent)

        # layout
        self.rowconfigure((0, 1, 2), weight = 1)
        self.columnconfigure((0,1), weight = 1)
        self.entry_variable = ctk.DoubleVar()
        self.data_var = data_var
        self.send_txt = send_txt
        self.gain_command = gain_command
        self.data_var.trace('w', self.update_text)

        self.label_slider = ctk.CTkLabel(self, text = text)
        self.label_slider.grid(column = 0, row = 0, sticky = 'W', padx = 5)

        self.num_label = ctk.CTkLabel(self, 
                                    text = self.data_var.get(),
                                    text_color = WHITE)
        self.num_label.grid(column = 1, row = 0, sticky = 'E', padx = 5)

        self.slider = ctk.CTkSlider(self, 
                        variable = self.data_var,
                        from_ = min_value,
                        to = max_value,
                        command=self.send_gain,
                        button_color = SEGMENTEDPRESSED,
                        button_hover_color = HOVERFINISHBUTTONS)
        self.slider.grid(row = 1, column = 0, columnspan = 2, sticky = 'EW', padx = 5, pady = 5)

        self.entry = ctk.CTkEntry(self, 
                    textvariable = self.entry_variable,
                    validate = 'key',
                    validatecommand = vcmd)
        self.entry.grid(row = 2, column = 0, columnspan = 1, sticky = 'EW', padx = 4, pady = 5)
        
        self.button =ctk.CTkButton(self, 
                    text = 'Send',
                    width=50,
                    height=15,
                    command = self.send_entry,
                    fg_color = SEGMENTEDPRESSED,
                    hover_color = HOVERFINISHBUTTONS)
        self.button.grid(row = 2, column = 1, columnspan = 1, sticky = 'EW', padx = 2.5, pady = 5)

    def update_text(self, *args):
        try: self.num_label.configure(text = f'{round(self.data_var.get(),2)}')
        except: pass
    
    def send_entry(self):
        if self.entry_variable.get() == "": 
            try: self.data_var.set(0)
            except: pass
        else:
            try: 
                self.data_var.set(self.entry_variable.get())
                self.gain_command(self.send_txt, float(self.data_var.get()))
            except: pass
    
    def send_gain(self, *args):
        self.gain_command(self.send_txt, float(self.data_var.get()))

class Button(ctk.CTkFrame):
        def __init__(self, parent: ctk.CTkFrame, reset_pid: Callable[[str], None], str_reset_param: str,text = 'Reset PID'):
            super().__init__(parent, fg_color = DARKGRAY)
            ctk.CTkButton(
                self, 
                text = text,
                width=100, 
                height=30, 
                fg_color = SEGMENTEDPRESSED,
                hover_color = HOVERFINISHBUTTONS,
                command = lambda: reset_pid(str_reset_param)).pack(padx = 5, pady = 5)
            
            self.pack(pady = 80, fill = 'both')

class LabelsStatusGraph():
    def __init__(self, parent: ctk.CTkFrame, labels_var):
        self.labels_var = labels_var

        self.label_system_variable = ctk.CTkLabel(parent, 
                                                textvariable=self.labels_var["system_variable"], 
                                                font = ("Lucida Console", 18), 
                                                fg_color = DARKGRAY, text_color = WHITE)
        self.label_system_variable.pack(anchor="w", pady = 4, padx = 5)

        self.label_output = ctk.CTkLabel(parent, 
                                        textvariable = self.labels_var["output"], 
                                        font = ("Lucida Console", 18), 
                                        fg_color = DARKGRAY, text_color = WHITE)
        self.label_output.pack(anchor="w", pady = 4, padx = 5)
        
        self.label_setpoint = ctk.CTkLabel(parent, 
                                        textvariable = self.labels_var["setpoint"], 
                                        font = ("Lucida Console", 18), 
                                        fg_color = DARKGRAY, text_color = WHITE)
        self.label_setpoint.pack(anchor="w", pady = 4, padx = 5)

        self.label_error = ctk.CTkLabel(parent, 
                                        textvariable = self.labels_var["error"], 
                                        font = ("Lucida Console", 18), 
                                        fg_color = DARKGRAY, text_color = WHITE)
        self.label_error.pack(anchor="w", pady = 4, padx = 5)
    
    def change_sv_value(self, txt) -> None:
        self.label_system_variable.configure(text = f'var:{txt}')

    def change_out_value(self, txt) -> None:
        self.label_output.configure(text = f'output:{txt}')
    
    def change_sp_value(self, txt) -> None:
        self.label_setpoint.configure(text = f'sp:{txt}')
    
    def change_err_value(self, txt) -> None:
        self.label_error.configure(text = f'error:{txt}')

class SpeedSlider(BasicFrame):
    def __init__(self, 
                parent: ctk.CTkFrame, 
                slider_variable: ctk.IntVar, 
                color: str,
                slider_command_comm):
        super().__init__(parent)
        self.slider_variable = slider_variable
        self.slider_command_comm = slider_command_comm

        self.label = ctk.CTkLabel(self, fg_color=color, 
                                text=f' {self.slider_variable.get()} cm/s',
                                font = ("Lucida Console", 18))
        self.label.pack(side = 'right', fill = 'both')

        self.slider = ctk.CTkSlider(
            self,
            fg_color=color,
            variable=self.slider_variable,
            command= self.slider_command,
            from_ = 0, to = 100,
            width = 350
        )
        self.slider.pack()
        self.pack()
    
    def slider_command(self, *args):
        self.slider_command_comm(int(self.slider_variable.get()))
        self.label.configure(text = f' {self.slider_variable.get()} cm/s')

class SensorIndicator(BasicFrame):
    def __init__(self, parent):
        super().__init__(parent)
        
        self.textbox = ctk.CTkTextbox(master=self, font = ("Lucida Console", 18), fg_color = DARKGRAY)
        self.textbox.pack(expand = True, fill = 'both')
        
        self.pack(expand = True, fill = 'both')

    def update_sensors(self, str_variable: str):
        self.textbox.configure(state = "normal")
        self.textbox.delete("1.0", "end")
        self.textbox.insert("1.0", str_variable)
        self.textbox.configure(state="disabled")