#!/usr/bin/env python3

from matplotlib.figure import Figure
import customtkinter as ctk 
from drone_ctrl.settings import *
from drone_ctrl.elements import *
from typing import Optional, Callable, Union, Dict
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import threading
import time
import numpy as np

class WindowGraph:
    def __init__(self, 
                parent: ctk.CTkFrame, 
                buttons_activated: Tuple[bool], 
                buttons_name: Tuple[str], 
                graph_name: str, 
                graph_commands: Tuple[Callable[[], None]], 
                slider_variables, 
                vcmd, 
                reset: Callable[[str], None],
                proportional_command: Callable[[str, float], None],
                integral_command: Callable[[str, float], None],
                derivative_command: Callable[[str, float], None],
                labels_variable: Dict[str, ctk.StringVar],
                update_externally = False):

        self.buttons_activated = buttons_activated
        self.buttons_name = buttons_name
        self.graph_name = graph_name
        self.graph_commands = graph_commands
        self.slider_variables = slider_variables
        self.vcmd = vcmd
        self.reset = reset
        self.proportional_command = proportional_command
        self.integral_command = integral_command
        self.derivative_command = derivative_command
        self.labels_variable = labels_variable
        self.update_externally = update_externally
        
        self.new_window = ctk.CTkToplevel(parent)
        self.new_window.geometry('980x435')
        self.new_window.resizable(False, False)
        self.new_window.title('EXTERNAL GRAPH')
        self.new_window.protocol("WM_DELETE_WINDOW", self.close_window_method)

        self.frame = ctk.CTkFrame(self.new_window, fg_color = DARKGRAY)
        self.frame.pack(fill = 'both', expand = True)
        self.frame.columnconfigure(0, weight = 1, uniform = 'a') # frame
        self.frame.columnconfigure(1, weight = 16, uniform = 'a') # sliders
        self.frame.columnconfigure(2, weight = 12, uniform = 'a') # labels
        self.frame.columnconfigure(3, weight = 40, uniform = 'a') # graph and buttons
        self.frame.columnconfigure(4, weight = 1, uniform = 'a') # frame
        self.frame.rowconfigure(0, weight = 1, uniform = 'a')# Frame
        self.frame.rowconfigure(1, weight = 25, uniform = 'a') # content
        self.frame.rowconfigure(2, weight = 1, uniform = 'a') # frame

        self.frame_slider = ctk.CTkFrame(self.frame, fg_color = DARKGRAY)
        self.frame_slider.grid(column = 1, row = 1 , sticky = 'nswe')

        self.frame_label = ctk.CTkFrame(self.frame, fg_color = DARKGRAY)
        self.frame_label.grid(column = 2, row = 1, sticky = 'nswe')

        self.frame_graph = ctk.CTkFrame(self.frame, fg_color = DARKGRAY)
        self.frame_graph.grid(column = 3, row = 1, sticky = 'nswe')

        self.create_graph()
        self.create_sliders()
        self.create_labels() 

    def create_graph(self) -> None:
        if not self.update_externally:
            self.graph = GraphPanel(self.frame_graph, self.graph_name)
        else:
            self.graph = GraphPanel(self.frame_graph, self.graph_name, external_update_graph=True)
        
        self.graph_buttons = GraphButtons(self.graph, self.graph_name, self.graph_commands, False)
        for i in range (5):
            if self.buttons_activated[i]:
                self.change_color(self.buttons_name[i])
                self.show_plot(self.buttons_name[i])

    def create_sliders(self) -> None:
        #self.variable_slider = SliderPanel(self.frame_slider, self.graph_name, self.slider_variables['system_variable'], self.vcmd, 0, 350)
        self.proportional_slider = SliderPanel(self.frame_slider, 
                                            'PROPORTIONAL', 
                                            self.slider_variables['proportional'], 
                                            self.vcmd, 0, 2,
                                            self.graph_name,
                                            self.proportional_command)
        
        self.integral_slider = SliderPanel(self.frame_slider, 
                                            'INTEGRAL', 
                                            self.slider_variables['integral'], 
                                            self.vcmd, 0, 2,
                                            self.graph_name,
                                            self.integral_command)
        
        self.derivative_slider = SliderPanel(self.frame_slider, 
                                            'DERIVATIVE', 
                                            self.slider_variables['derivative'], 
                                            self.vcmd, 0, 2,
                                            self.graph_name,
                                            self.derivative_command)

    def create_labels(self) -> None:
        self.labels = LabelsStatusGraph(self.frame_label, self.labels_variable)
        self.reset_button = Button(self.frame_label, self.reset, self.graph_name)

    def change_labels(self, system_var:Union[int, float], setpoint: Union[int, float], error: Union[int, float], output:Union[int, float]):
        self.labels.change_sv_value(system_var)
        self.labels.change_sp_value(setpoint)
        self.labels.change_err_value(error)
        self.labels.change_out_value(output)

    def change_color(self, variable: str) -> None:
        match variable:
            case 'DATA':
                self.graph_buttons.change_color('DATA')
            case 'SETPOINT':
                self.graph_buttons.change_color('SETPOINT')
            case 'ERROR':
                self.graph_buttons.change_color('ERROR')
            case 'OUTPUT':
                self.graph_buttons.change_color('OUTPUT')
            case 'CREATION':
                self.graph_buttons.change_color('CREATION')
            case 'ACTIVATE':
                self.graph_buttons.change_color('ACTIVATE')
            case _:
                print('SOMETHING WRONG')

    def show_plot(self, variable: str) -> None:
        match variable:
            case 'DATA':
                self.graph.data_method()
            case 'SETPOINT':
                self.graph.setpoint_method()
            case 'ERROR':
                self.graph.error_method()
            case 'OUTPUT':
                self.graph.output_method()
            case 'CREATION':
                self.graph.external_graph_method()
            case _: 
                print('Something Wrong')

    def close_window_method(self):
        self.graph_commands[4](self.graph_name)

class LabelVideo(ctk.CTkFrame):
    def __init__(self, parent: ctk.CTkFrame):
        super().__init__(parent, fg_color = DARKGRAY)
        self.label_video = ctk.CTkLabel(self, fg_color = DARKBLUE, text = '', font = ("Lucida Console", 30))
        self.label_video.pack(fill = 'both', expand = True)
        self.pack(expand = True, fill = 'both')
    
    def label_video_img(self, img) -> None:
        self.label_video.configure(image = img)
    
    def label_video_text(self, txt: str, s: int) -> None:
        self.label_video.configure(text = txt, size = int)

class TexBoxSensorIndicator(ctk.CTkFrame):
    def __init__(self, parent: ctk.CTkFrame):
        super().__init__(parent, fg_color = DARKGRAY)
        self.sensor_indicator = SensorIndicator(self)
        self.pack(expand = True, fill = 'both')
    
    def update_sensors(self, str_var: str):
        self.sensor_indicator.update_sensors(str_var)

class GraphPanel(ctk.CTkFrame):
    def __init__(self, master_frame: ctk.CTkFrame, text: str, color = DARKGRAY, commands_variable = False, external_update_graph = False, commands: Optional[Tuple[Callable, None]] = None) -> None:
        super().__init__(master_frame, fg_color  = color) #GRAPHFACECOLOR
        self.rowconfigure(0, weight = 3, uniform = 'a')
        self.columnconfigure(0, weight = 28, uniform = 'a')
        self.columnconfigure(1, weight = 2, uniform = 'a')

        self.text_graph = text
        self.color = color
        self.commands = commands #  important later
        self.commands_variable = commands_variable #important later

        if self.commands:
            self.commands_variable[0]
        
        self.external_update_graph = external_update_graph
        self.data = (6.0, 1.0, 2.0, 3.0)

        self.data_graph_variable = False
        self.data_button_pressed = False

        self.setpoint_graph_variable = False
        self.setpoint_button_pressed = False

        self.error_graph_variable = False
        self.error_button_pressed = False

        self.output_graph_variable = False
        self.output_button_pressed = False

        self.creating_lists()
        self.creating_graph()
        
        self.pack(fill = 'both', expand = True)

    def data_method(self) -> None:
        if self.data_graph_variable: # Press the button
            self.data_graph_variable = False
        else:
            self.data_graph_variable = True 

    def setpoint_method(self) -> None:
        if self.setpoint_graph_variable:
            self.setpoint_graph_variable = False
        else:
            self.setpoint_graph_variable = True

    def error_method(self) -> None:
        if self.error_graph_variable:
            self.error_graph_variable = False
        else:
            self.error_graph_variable = True

    def output_method(self) -> None:
        if self.output_graph_variable:
            self.output_graph_variable = False
        else: 
            self.output_graph_variable = True

    def external_graph_method(self) -> None: #It doesn't do nothing, you can delete that
        pass

    def creating_lists(self) -> None:
        self.last_x = time.time()
        self.actual_x = time.time()
        self.x = []
        self.y = []
        self.y_setpoint = []
        self.y_error = []
        self.y_output = []

    def get_adress_methods(self) -> Callable[[], None]:
        return self.data_method

    def creating_graph(self) -> None:
        # Creating a Matplotlib figure and axes
        self.fig = Figure(figsize=(10, 8), dpi=100, facecolor = self.color) #(10,8)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_facecolor(self.color) #GRAPHBG
        self.ax.spines['left'].set_color(self.color)
        self.ax.spines['left'].set_linewidth(2)
        self.ax.spines['bottom'].set_color(self.color)
        self.ax.spines['top'].set_color(self.color)
        self.ax.spines['right'].set_color(self.color)
        self.ax.set_title(self.text_graph, color = WHITE)
        self.ax.grid(
                    axis = 'y', 
                    visible = True, 
                    alpha = 0.3, 
                    linewidth = 0.8, 
                    #linestyle = (0,(4, 3)), 
                    color = '0')
        self.ax.tick_params(axis='both', colors = WHITE) #WHITE

        # Initial plot setup
        if self.external_update_graph:
            self.x.append(0)
            self.y.append(0)
            self.y_setpoint.append(0)
            self.y_error.append(0)
            self.y_output.append(0)
        else:
            self.x = np.linspace(0, 2 * np.pi, 100)
            self.y = np.sin(self.x)
            self.y_setpoint = np.cos(self.x)
            self.y_error = np.sin(self.x+0.8)
            self.y_output = np.cos(self.x+0.8)
        #self.x.append(0)
        #self.y.append(0)
        #self.y_setpoint.append(0)
        #self.y_error.append(0)
        #self.y_output.append(0)
        ##############
        self.line, = self.ax.plot(self.x, self.y, color = GRAPHBLUE)
        self.setpoint_line, = self.ax.plot(self.x, self.y_setpoint, color = GRAPHGREEN)
        self.error_line, = self.ax.plot(self.x, self.y_error, color = GRAPHRED)
        self.output_line, = self.ax.plot(self.x, self.y_output, color = GRAPHLIGHTBLUE)
        
        # Map Logic
        # (current_value, setpoint, error, output)
        self.change_external_conditions = {
            (False, False, False, False): lambda: self.ax.set_ylim(-5, 5),
            (True, False, False, False) : lambda: self.ax.set_ylim(min(self.y)-10, max(self.y)+10),
            (False, True, False, False) : lambda: self.ax.set_ylim(min(self.y_setpoint)-10, max(self.y_setpoint)+10),
            (False, False, True, False) : lambda: self.ax.set_ylim(min(self.y_error)-10, max(self.y_error)+10),
            (False, False, False, True) : lambda: self.ax.set_ylim(min(self.y_output)-10, max(self.y_output)+10),
            (True, True, False, False) : lambda: self.ax.set_ylim(min(min(self.y), min(self.y_setpoint))-10, max(max(self.y), max(self.y_setpoint))+10), 
            (True, False, True, False) : lambda: self.ax.set_ylim(min(min(self.y), min(self.y_error))-10, max(max(self.y), max(self.y_error))+10),
            (True, False, False, True) : lambda: self.ax.set_ylim(min(min(self.y), min(self.y_output))-10, max(max(self.y), max(self.y_output))+10),
            (False, True, True, False) : lambda: self.ax.set_ylim(min(min(self.y_setpoint), min(self.y_error))-10, max(max(self.y_setpoint), max(self.y_error))+10),
            (False, True, False, True) : lambda: self.ax.set_ylim(min(min(self.y_setpoint), min(self.y_output))-10, max(max(self.y_setpoint), max(self.y_output))+10),
            (False, False, True, True) : lambda: self.ax.set_ylim(min(min(self.y_error), min(self.y_output))-10, max(max(self.y_error), max(self.y_output))+10),
            (True, True, True, False) : lambda: self.ax.set_ylim(min(min(self.y), min(self.y_setpoint), min(self.y_error))-10, max(max(self.y), max(self.y_setpoint), max(self.y_error))+10),
            (True, True, False, True) : lambda: self.ax.set_ylim(min(min(self.y), min(self.y_setpoint), min(self.y_output))-10, max(max(self.y), max(self.y_setpoint), max(self.y_output))+10),
            (True, False, True, True) : lambda: self.ax.set_ylim(min(min(self.y), min(self.y_error), min(self.y_output))-10, max(max(self.y), max(self.y_error), max(self.y_output))+10),
            (False, True, True, True) : lambda: self.ax.set_ylim(min(min(self.y_setpoint), min(self.y_error), min(self.y_output))-10, max(max(self.y_setpoint), max(self.y_error), max(self.y_output))+10),
            (True, True, True, True) : lambda: self.ax.set_ylim(min(min(self.y), min(self.y_setpoint), min(self.y_error), min(self.y_output))-10, max(max(self.y),max(self.y_setpoint), max(self.y_error), max(self.y_output))+10)
        }

        # Create a canvas to display the plot in Tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=self) #We add the master UwU

        self.canvas.get_tk_widget().configure(bg=self.color, highlightthickness=0, bd=0) # info below
        #elimina los bordes visibles que suelen venir por defecto con los widgets de Tkinter (como highlightthickness y bd), 
        # y además iguala el fondo con el color del gráfico, evitando cualquier parpadeo o "línea blanca" al redimensionar.

        self.fig.patch.set_facecolor(self.color)
        self.canvas.get_tk_widget().grid(row = 0, 
                                        column = 0, 
                                        sticky = 'nsew', 
                                        padx = 1, pady = 5)
        self.canvas.draw()
        
        if not self.external_update_graph:
            self.th1 = threading.Thread(target = self.math_functions, daemon = True)
            self.th1.start()
        else:
            self.th1 = threading.Thread(target = self.change_external, daemon = True)
            self.th1.start()
    
    def math_functions(self):
        empty = np.array([])
        while True:

            # Redibujar el gráfico
            if self.data_graph_variable:
                self.y = np.sin(self.x + time.time() % (2 * np.pi))
                self.line.set_xdata(self.x)
                self.line.set_ydata(self.y)
            
            if self.setpoint_graph_variable:
                self.y_setpoint = np.cos(self.x + time.time() % (2 * np.pi))
                self.setpoint_line.set_xdata(self.x)
                self.setpoint_line.set_ydata(self.y_setpoint)
            
            if self.error_graph_variable:
                self.y_error = np.sin(self.x + time.time() % (2 * np.pi) + 0.8)
                self.error_line.set_xdata(self.x)
                self.error_line.set_ydata(self.y_error)
            
            if self.output_graph_variable:
                self.y_output = np.cos(self.x + time.time() % (2 * np.pi) + 0.8) 
                self.output_line.set_xdata(self.x)
                self.output_line.set_ydata(self.y_output)
            
            if not self.data_graph_variable:
                self.line.set_ydata(empty)
                self.line.set_xdata(empty)
            
            if not self.error_graph_variable:
                self.error_line.set_ydata(empty)
                self.error_line.set_xdata(empty)

            if not self.setpoint_graph_variable:
                self.setpoint_line.set_ydata(empty)
                self.setpoint_line.set_xdata(empty)

            if not self.output_graph_variable:
                self.output_line.set_ydata(empty)
                self.output_line.set_xdata(empty)

            self.canvas.draw()
            time.sleep(0.7)

    def change_data(self, data: tuple):
        self.data = data 

    def change_external(self):
        while True:
            empty = []
            self.actual_x = time.time()
            self.x.append(self.actual_x-self.last_x)
            if len(self.x) > 30:
                self.x.pop(0)
            self.y.append(self.data[0])
            if len(self.y) > 30:
                self.y.pop(0)
            self.y_setpoint.append(self.data[1])
            if len(self.y_setpoint) > 30:
                self.y_setpoint.pop(0)
            self.y_error.append(self.data[2])
            if len(self.y_error):
                self.y_error.pop(0)
            self.y_output.append(self.data[3])
            if len(self.y_output):
                self.y_output.pop(0)
            
            if self.data_graph_variable:            
                self.line.set_xdata(self.x)
                self.line.set_ydata(self.y)
            else:
                self.line.set_xdata(empty)
                self.line.set_ydata(empty)

            if self.setpoint_graph_variable:
                self.setpoint_line.set_xdata(self.x)
                self.setpoint_line.set_ydata(self.y_setpoint)
            else:
                self.setpoint_line.set_xdata(empty)
                self.setpoint_line.set_ydata(empty)

            if self.error_graph_variable:
                self.error_line.set_xdata(self.x)
                self.error_line.set_ydata(self.y_error)
            else: 
                self.error_line.set_xdata(empty)
                self.error_line.set_ydata(empty)

            if self.output_graph_variable:
                self.output_line.set_xdata(self.x)
                self.output_line.set_ydata(self.y_output)
            else:
                self.output_line.set_xdata(empty)
                self.output_line.set_ydata(empty)

            # MAP LOGIC
            # (current_value, setpoint, error, output)
            self.ax.set_xlim(min(self.x), max(self.x)+1)
            logic_tuple = (self.data_graph_variable,
                        self.setpoint_graph_variable,
                        self.error_graph_variable,
                        self.output_graph_variable)

            if logic_tuple in self.change_external_conditions:
                self.change_external_conditions[logic_tuple]()

            self.canvas.draw()
            time.sleep(0.7)

class GraphButtons(ctk.CTkFrame):
    def __init__(self, parent_frame: GraphPanel, str_graph_type: str, button_commands: tuple[Callable[[str],None]], creation_button = False):
        super().__init__(parent_frame, fg_color = DARKGRAY)

        self.grid(row = 0, column = 1, sticky = 'nswe', pady = 35)

        self.str_graph_type = str_graph_type

        self.creation_button = creation_button
        
        self.data_button_pressed = False       
        self.setpoint_button_pressed = False
        self.error_button_pressed = False
        self.output_button_pressed = False
        self.creation_button_pressed = False
        self.activate_button_pressed = False

        self.button_commands_data = button_commands[0]
        self.button_commands_sp = button_commands[1]
        self.button_commands_error = button_commands[2]
        self.button_commands_output = button_commands[3]
        self.button_commands_external_graphic = button_commands[4]
        self.button_commands_activate = button_commands[5]
        
        self.creating_buttons()
    
    def creating_buttons(self) -> None:    
        height = 20
        self.data_button = ctk.CTkButton(self, 
                                        command = lambda: self.button_commands_data(self.str_graph_type),
                                        fg_color = FG_DATABUTTON_COLOR,
                                        hover_color = HOVER_DATABUTTON_COLOR,
                                        text = '',
                                        height = height)
        self.data_button.pack(expand = False)
        ToolTip(self.data_button, 'System Variable')
        self.setpoint_button = ctk.CTkButton(self,
                                            command = lambda: self.button_commands_sp(self.str_graph_type),
                                            fg_color = FG_SETPOINTBUTTON_COLOR,
                                            hover_color = HOVER_SETPOINTBUTTON_COLOR,
                                            text = '',
                                            height = height)
        self.setpoint_button.pack(expand = False)
        ToolTip(self.setpoint_button, 'Set Point')
        self.error_button = ctk.CTkButton(self, 
                                        command = lambda: self.button_commands_error(self.str_graph_type),
                                        fg_color = FG_ERRORBUTTON_COLOR,
                                        hover_color = HOVER_ERRORBUTTON_COLOR,
                                        text = '',
                                        height = height)
        self.error_button.pack(expand = False)
        ToolTip(self.error_button, 'Error')
        self.output_button = ctk.CTkButton(self,
                                        command = lambda: self.button_commands_output(self.str_graph_type),
                                        fg_color = FG_OUTBUTTON_COLOR,
                                        hover_color = HOVER_OUTBUTTON_COLOR,
                                        text = '',
                                        height = height)
        self.output_button.pack(expand = False)
        ToolTip(self.output_button, 'System Output')
        self.activate_button = ctk.CTkButton(self,
                                            command = lambda:self.button_commands_activate(self.str_graph_type),
                                            fg_color = FG_ACTIVATEBUTTON_COLOR,
                                            hover_color = HOVER_ACTIVATEBUTTON_COLOR,
                                            text = '',
                                            height = height)
        self.activate_button.pack(expand = False)
        ToolTip(self.activate_button, 'Activate')
        if self.creation_button:
            self.creation_button = ctk.CTkButton(self,
                                            command = lambda: self.button_commands_external_graphic(self.str_graph_type),
                                            fg_color = HOVER_CREATIONBUTTON_COLOR ,
                                            hover_color= FG_CREATIONBUTTON_COLOR,
                                            text = '',
                                            height = height)
            self.creation_button.pack(side = 'bottom', fill = 'both')
            ToolTip(self.creation_button, 'External Graphic')
    
    def change_color(self, button_name: str) -> None:
        # button_name = ('DATA', SETPOINT, ERROR, OUTPUT, CREATION)
        match button_name:
            case 'DATA':
                if self.data_button_pressed: # Press the button
                    self.data_button.configure(fg_color = FG_DATABUTTON_COLOR)
                    self.data_button_pressed = False
                else:
                    self.data_button.configure(fg_color = HOVER_DATABUTTON_COLOR)
                    self.data_button_pressed = True 
    
            case 'SETPOINT' :
                if self.setpoint_button_pressed:
                    self.setpoint_button.configure(fg_color = FG_SETPOINTBUTTON_COLOR)
                    self.setpoint_button_pressed = False
                else:
                    self.setpoint_button.configure(fg_color = HOVER_SETPOINTBUTTON_COLOR)
                    self.setpoint_button_pressed = True
                
            case 'ERROR' :
                if self.error_button_pressed:
                    self.error_button.configure(fg_color = FG_ERRORBUTTON_COLOR)
                    self.error_button_pressed = False
                else:
                    self.error_button.configure(fg_color = HOVER_ERRORBUTTON_COLOR)
                    self.error_button_pressed = True
            
            case 'OUTPUT' : 
                if self.output_button_pressed:
                    self.output_button.configure(fg_color = FG_OUTBUTTON_COLOR)
                    self.output_button_pressed = False
                else: 
                    self.output_button.configure(fg_color = HOVER_OUTBUTTON_COLOR)
                    self.output_button_pressed = True
            
            case 'CREATION':
                if self.creation_button_pressed:
                    self.creation_button.configure(fg_color = HOVER_CREATIONBUTTON_COLOR)
                    self.creation_button_pressed = False
                else: 
                    self.creation_button.configure(fg_color = FG_CREATIONBUTTON_COLOR)
                    self.creation_button_pressed = True
            
            case 'ACTIVATE':
                if self.activate_button_pressed:
                    self.activate_button.configure(fg_color = FG_ACTIVATEBUTTON_COLOR)
                    self.activate_button_pressed = False
                
                else:
                    self.activate_button.configure(fg_color = HOVER_ACTIVATEBUTTON_COLOR)
                    self.activate_button_pressed = True

    def get_buttons_activated(self) -> Tuple[bool]:
        
        active = (self.data_button_pressed, 
                self.setpoint_button_pressed, 
                self.error_button_pressed, 
                self.output_button_pressed,
                self.activate_button_pressed,
                self.creation_button_pressed
                )
        return active

class ToolTip:
    def __init__(self, widget, text=str, delay=0.75):
        self.widget = widget
        self.text = text
        self.delay = delay  # Tiempo en segundos
        self.tooltip_window = None
        self.hovering = False
        self.thread = None

        # Vincular eventos de entrada y salida del ratón
        widget.bind("<Enter>", self.schedule_tooltip)
        widget.bind("<Leave>", self.hide_tooltip)

    def schedule_tooltip(self, event):
        self.hovering = True
        if self.thread is None or not self.thread.is_alive():
            self.thread = threading.Thread(target=self.delayed_show_tooltip, args=(event,), daemon=True)
            self.thread.start()

    def delayed_show_tooltip(self, event):
        time.sleep(self.delay)
        if self.hovering:
            # Mostrar tooltip en el hilo principal usando `after` (solo para manipular UI de forma segura)
            self.widget.after(0, lambda: self.show_tooltip(event))

    def show_tooltip(self, event):
        if self.tooltip_window:  # Evitar mostrar múltiples tooltips
            return

        self.tooltip_window = ctk.CTkToplevel(self.widget)
        self.tooltip_window.wm_overrideredirect(True)
        self.tooltip_window.wm_geometry(f"+{event.x_root + 10}+{event.y_root + 10}")

        label = ctk.CTkLabel(self.tooltip_window, text=self.text, bg_color=GRAY)
        label.pack()

    def hide_tooltip(self, event):
        self.hovering = False
        if self.tooltip_window:
            self.tooltip_window.destroy()
            self.tooltip_window = None