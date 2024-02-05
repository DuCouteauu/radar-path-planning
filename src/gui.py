from tkinter import *
import matplotlib.pyplot
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg,
                                               NavigationToolbar2Tk)
import numpy as np


class GUI():

    def __init__(self, frame_distances):

        self.frame_distances = frame_distances

        self.polar_data = {}

        self.window = Tk()  # The main Tkinter window
        self.window.title('Path Planning of Autonomous Vehicles')  # Setting the title
        self.window.geometry("1000x1000")  # Dimensions of the main window
        self.fig = Figure(figsize=(10, 10), dpi=100)

        # Creating the Tkinter canvas containing the Matplotlib figure
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.window)

        self.canvas.draw()  # Initially draw canvas

        self.canvas.get_tk_widget().pack()  # Placing the canvas on the Tkinter window

        self.toolbar = NavigationToolbar2Tk(self.canvas, self.window)  # Placing the toolbar on the Tkinter window

        # Adding the subplots
        self.radar_plot1 = self.fig.add_subplot(221)
        self.radar_plot2 = self.fig.add_subplot(222)
        self.polar_plot = self.fig.add_subplot(223, polar=True)
        self.grid_plot = self.fig.add_subplot(224)

        # Move to config at some point
        self.cmap1 = matplotlib.colors.ListedColormap(['Blue', 'pink', 'red'])
        self.cmap2 = matplotlib.colors.ListedColormap(['Blue', 'pink', 'red', 'green'])

        # Initialize empty plots
        self.radar_ax1 = self.radar_plot1.plot([], [])
        self.radar_ax2 = self.radar_plot2.plot([], [])
        self.polar_ax = self.polar_plot.plot([], [])
        self.grid_ax = self.grid_plot.plot([], [])

        self.update_gui()  # Display gui

    def clear_canvas(self):
        """May not be necessary"""
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
        """Does not clear occupancy grid"""
        self.radar_plot1.clear()
        self.radar_plot2.clear()
        self.polar_plot.clear()
        self.canvas.draw()

    def update_gui(self):
        self.window.update_idletasks()
        self.window.update()

    def plot_image_data(self, image):
        """Extracts and plots distances. Finds the polar coordinates, adds them to polar data, and plots"""
        self.clear_plots()
        self.radar_ax1 = self.radar_plot1.plot(self.frame_distances, image["radar_data"][0])
        self.radar_ax2 = self.radar_plot2.plot(self.frame_distances, image["radar_data"][1])

        self.polar_data[np.radians(image["radar_angle"])] = image["polar_distance_data"][0]
        self.polar_data[np.radians(image["radar_angle"] + 180)] = image["polar_distance_data"][1]

        self.polar_ax = self.polar_plot.scatter(self.polar_data.keys(), self.polar_data.values())
        self.canvas.draw()
        self.update_gui()


    def generate_and_plot_grid(self, polar_map):
        """Needs to be changed to use new grid_mapping functions. grid_mapping.main() is no longer used"""
        pass
        # grid = grid_mapping.main(polar_map)
        #
        # grid_ax = grid_plot.pcolor(grid[::-1], cmap=cmap1, edgecolors='k', linewidths=3)
        # canvas.draw()
        # update_gui()
        # return grid