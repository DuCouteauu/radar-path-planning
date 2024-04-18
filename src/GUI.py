import tkinter
from tkinter import *
import matplotlib.pyplot
from matplotlib.backends._backend_tk import NavigationToolbar2Tk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
import config


class GUI:

    def __init__(self):

        self.final_grid = None
        self.can_proceed = True
        self.radar_field_size = config.radar_field_size

        self.frame_distances = []
        self.polar_data = {}

        self.window = Tk()  # The main Tkinter window
        self.window.title('Path Planning of Autonomous Vehicles')  # Setting the title
        self.window.geometry("1500x1000")  # Dimensions of the main window
        self.fig = Figure(figsize=(10, 10), dpi=100)
        self.fig.subfigures(hspace = 0.15)
        self.fig.subplots_adjust(hspace=0.25)

        self.status_initializing = Label(self.window, text=f"Initializing", font='Times 24')
        self.status_initializing.place(x=10,y=10)
        self.status_scanning = Label(self.window, text=f"Scanning", font='Times 24')
        self.status_scanning.place(x=10, y=10)
        self.status_pathing = Label(self.window, text=f"Pathing", font='Times 24')
        self.status_pathing.place(x=10, y=10)
        self.status_uploading = Label(self.window, text=f"Uploading", font='Times 24')
        self.status_uploading.place(x=10, y=10)
        self.status_executing = Label(self.window, text=f"Executing", font='Times 24')
        self.status_executing.place(x=10, y=10)


        # Creating the Tkinter canvas containing the Matplotlib figure
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.window)

        self.canvas.draw()  # Initially draw canvas

        self.canvas.get_tk_widget().pack()  # Placing the canvas on the Tkinter window

        self.toolbar = NavigationToolbar2Tk(self.canvas, self.window)  # Placing the toolbar on the Tkinter window

        self.trigger_retry_scan = False

        # Adding the subplots
        self.radar_plot1 = self.fig.add_subplot(221)
        self.radar_plot1.set_title("Front Radar")
        self.radar_plot1.set_xlabel("Distance")
        self.radar_plot1.set_ylabel("Amplitude")


        self.radar_plot2 = self.fig.add_subplot(222)
        self.radar_plot2.set_title("Back Radar")
        self.radar_plot2.set_xlabel("Distance")

        self.polar_plot = self.fig.add_subplot(223, polar=True)

        self.grid_plot = self.fig.add_subplot(224)
        self.grid_plot.set_title("Occupancy Grid")

        # Initialize empty plots
        self.radar_ax1 = None
        self.radar_ax2 = None
        self.polar_ax = None
        self.grid_ax = None

        self.peaks_ax1 = None
        self.peaks_ax2 = None

        self.mode = "default"

        self.step_through = tkinter.IntVar()

        # Buttons
        self.button_run_connected = Button(self.window, text="Run Connected", height=5, width=30, command=self.run_connected)
        self.button_run_connected.place(x=17, y=100)

        self.button_run_sim = Button(self.window, text="Run Simulation", height=5, width=30, command=self.run_simulation)
        self.button_run_sim.place(x=17, y=200)

        self.step_through_checkbutton = Checkbutton(self.window, text="Step-Through Mode", variable=self.step_through)
        self.step_through_checkbutton.place(x=17, y=300)





        self.window.bind("<Key>", self.key_pressed)

        self.update_gui()  # Display gui

    def key_pressed(self, event):
        """Used to increment through the simulation data, when window.mainloop() is called elsewhere, the program
        will halt until this function is called, breaking the loop and allowing the program to continue"""
        if self.can_proceed:
            self.window.quit()

    def run_connected(self):
        """Sets mode to connected and destroys buttons"""
        self.mode = "connected"
        self.button_run_connected.destroy()
        self.button_run_sim.destroy()
        self.step_through_checkbutton.destroy()
        self.update_gui()
        self.window.quit()

    def run_simulation(self):
        """Sets mode to simulated and destroys buttons"""
        self.mode = "simulated"
        self.button_run_connected.destroy()
        self.button_run_sim.destroy()
        self.step_through_checkbutton.destroy()
        self.update_gui()
        self.window.quit()

    def change_status(self, status_label):
        self.status_initializing.place_forget()
        self.status_scanning.place_forget()
        self.status_pathing.place_forget()
        self.status_uploading.place_forget()
        self.status_executing.place_forget()
        status_label.place(x=10, y=10)
        self.update_gui()

    def retry_scan(self):
        self.trigger_retry_scan = True
        self.clear_plots()
        self.update_gui()
        self.window.quit()

    def clear_canvas(self):
        """May not be necessary anymore, completely clears and resets the tk window"""
        for item in self.canvas.get_tk_widget().find_all():

            self.canvas.get_tk_widget().delete(item)

            # Placing the canvas on the Tkinter window
            self.canvas.get_tk_widget().pack()

            # Creating the Matplotlib toolbar
            self.toolbar = NavigationToolbar2Tk(self.canvas, self.window)
            self.toolbar.update()

            # Placing the toolbar on the Tkinter window
            self.canvas.get_tk_widget().pack()

    def clear_plots(self):
        """Intended to be called with every GUI update, does not clear occupancy grid"""
        self.radar_plot1.clear()
        self.radar_plot2.clear()
        self.polar_plot.clear()
        self.grid_plot.clear()

        # Reset titles
        self.radar_plot1.set_title("Front Radar")
        self.radar_plot1.set_xlabel("Distance")
        self.radar_plot1.set_ylabel("Amplitude")

        self.radar_plot2.set_title("Back Radar")
        self.radar_plot2.set_xlabel("Distance")

        self.canvas.draw()

    def update_gui(self):

        self.window.update_idletasks()
        self.window.update()

    def unpack_polar_data(self, polar_data):
        # TODO Modify radar_processing.py to remove the need for this function
        print("POLAR DATA")
        print(polar_data)
        angles = []
        distances = []
        for angle in polar_data:
            for distance in polar_data[angle]:
                angles.append(angle)
                distances.append(distance)

        return angles, distances

    def plot_image_data(self, image, frame_distances, polar_data, hit_grid, angle):
        """Extracts and plots distances. Finds the polar coordinates, adds them to polar data, and plots"""
        self.clear_plots()

        self.frame_distances = frame_distances

        self.radar_ax1 = self.radar_plot1.plot(self.frame_distances, image["radar_data"][0])
        self.radar_ax2 = self.radar_plot2.plot(self.frame_distances, image["radar_data"][1])
        self.radar_plot1.set_ylim(0,500)
        self.radar_plot2.set_ylim(0, 500)

        self.peaks_ax1 = self.radar_plot1.plot(list(image["peaks"][0].keys()), list(image["peaks"][0].values()), "x")
        self.peaks_ax2 = self.radar_plot2.plot(list(image["peaks"][1].keys()), list(image["peaks"][1].values()), "x")

        self.radar_field_lines = [self.polar_plot.plot([np.radians(angle + config.radar_field_size/2), np.radians(angle + config.radar_field_size/2 + 180)], [config.radar_range_interval[1], config.radar_range_interval[1]], color="red", linestyle=":"), self.polar_plot.plot([np.radians(angle - config.radar_field_size/2), np.radians(angle - config.radar_field_size/2 + 180)], [config.radar_range_interval[1], config.radar_range_interval[1]], color="red", linestyle=":")]

        angles, distances = self.unpack_polar_data(polar_data)
        self.polar_ax = self.polar_plot.scatter(np.radians(angles), distances)

        hit_grid[config.grid_center] = config.hit_maximum

        self.grid_ax = self.grid_plot.pcolor(hit_grid[::-1]/config.hit_maximum, cmap=config.cmap, edgecolors='k', linewidths=3)

        self.cell_size_label = Label(self.window, text = f"Cell Size: {round(config.cell_size, 2)}m")
        self.cell_size_label.place(x = 17, y = 300)

        self.canvas.draw()
        self.update_gui()

    def plot_final_grid(self, hit_grid):
        """Currently just assumes all hits are correct detections"""
        # TODO Define settings in config.py to fine tune the conversion from the hit grid to the binary occupancy grid
        self.final_grid = np.zeros([config.grid_size, config.grid_size], dtype=float)

        for ix, x in enumerate(hit_grid):

            for iy, y, in enumerate(x):

                if y != 0:
                    self.final_grid[ix][iy] = 0.5
                else:
                    self.final_grid[ix][iy] = 0.0

        self.final_grid[int((config.grid_size + 1)/2 - 1)][int((config.grid_size + 1)/2 - 1)] = 1.0
        self.grid_plot.clear()
        self.grid_ax = self.grid_plot.pcolor(self.final_grid[::-1], cmap=config.cmap, edgecolors='k', linewidths=3)

        self.canvas.draw()
        self.update_gui()



    def await_execution(self):
        execute_button = Button(self.window,text="Execute Path", height=5, width=30, command=self.window.quit)
        execute_button.place(x=17, y=140)
        button_retry_scan = Button(self.window, text="Retry Scan", height=5, width=30, command=self.retry_scan)
        button_retry_scan.place(x=17, y=240)
        self.window.mainloop()
        button_retry_scan.destroy()
        execute_button.destroy()
        self.canvas.draw()
        self.update_gui()


    def get_input(self):
        input_label = Label(self.window, text = f"x:          y:")
        input_label.place(x = 17, y = 80)

        self.canvas.draw()
        self.update_gui()
        x_box = Text(self.window, height=1,width=2)
        x_box.place(x=17,y=100)
        y_box = Text(self.window, height=1, width=2)
        y_box.place(x=57, y=100)

        confirm_button = Button(self.window,text="Confirm", height=5, width=30, command=self.window.quit)
        confirm_button.place(x=17, y=140)
        button_retry_scan = Button(self.window, text="Retry Scan", height=5, width=30, command=self.retry_scan)
        button_retry_scan.place(x=17, y=240)
        self.canvas.draw()
        self.update_gui()

        self.can_proceed = False
        self.window.mainloop()
        if not self.trigger_retry_scan:
            input_coords = (int(y_box.get(1.0,"end-1c"))-1, int(x_box.get(1.0,"end-1c"))-1)
        else:
            input_coords = (1,1)
        x_box.destroy()
        y_box.destroy()
        input_label.destroy()
        confirm_button.destroy()
        button_retry_scan.destroy()
        self.can_proceed = True

        return input_coords


    def plot_destination(self, destination):
        self.final_grid[destination[0]][destination[1]] = 1
        self.grid_ax = self.grid_plot.pcolor(self.final_grid[::-1], cmap=config.cmap, edgecolors='k', linewidths=3)
        self.canvas.draw()
        self.update_gui()

    def plot_path(self, path):
        # Unzip the list of tuples into two lists of x coordinates and y coordinates
        y_coords, x_coords = zip(*path)
        x_coords = [x+0.5 for x in x_coords]
        y_coords = [config.grid_size-y-0.5 for y in y_coords]
        # Plot the path

        self.grid_plot.plot(x_coords, y_coords, 'yellow', linewidth=4, markersize=5)
        self.canvas.draw()
        self.update_gui()
