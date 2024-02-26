import radar_processing
from pyfirmata import Arduino, SERVO, OUTPUT
import config
import GUI
import simulation_data

# Uncomment if running in connected mode
# import acconeer.exptool as et


def main():
    """Temporary main function"""

    gui = GUI.GUI()

    image = None
    image_buffer = None

    gui.window.mainloop()
    if gui.mode == "simulated":
        # Init radar stub
        radar_stub = radar_processing.RadarStub()

        # Scanning Loop
        for frame in range(config.rotations_per_scan):
            image_buffer = image  # Save previous image as a buffer
            # Simulated data, cycles through saved data sets until scan is complete
            radar_data = [simulation_data.four_sweep[frame%len(simulation_data.four_sweep)], simulation_data.continuous_data[frame % len(simulation_data.continuous_data)]]
            peaks = radar_stub.get_radar_data_peaks(radar_data)  # Get the peaks of the radar frame
            # Pack the data, peaks, and angle into an image
            image = {"radar_data": radar_data,
                     "peaks": peaks,
                     "radar_angle": radar_stub.servo_angle}

            if image_buffer is not None:
                # Process radar peaks using the buffered image
                radar_stub.process_radar_peaks(image["peaks"], image_buffer["peaks"])

            # Plot data and update GUI
            gui.plot_image_data(image, radar_stub.frame_distances, radar_stub.polar_data, radar_stub.hit_grid, radar_stub.servo_angle)
            if gui.step_through.get():
                gui.window.mainloop()  # Await user input (press any key to proceed)
            radar_stub.increment_servo(config.radar_increment)  # Rotate and move on to the next frame

        gui.plot_final_grid(radar_stub.hit_grid)  # Obtains the binary occupancy grid from the hit grid and plots it
        gui.window.mainloop()  # Wait for desired destination

    elif gui.mode == "connected":
        # Arduino Setup
        board = Arduino(config.arduino_port)
        board.digital[config.arduino_servo_pin].mode = SERVO
        board.digital[config.arduino_servo_pin].write(config.servo_offset)

        # Init radar system
        radar = radar_processing.RadarConnected(board)

        # Scanning Loop
        for frame in range(config.rotations_per_scan):
            image_buffer = image  # Save previous image as a buffer
            image = radar.take_and_average_images()  # Take new image

            if image_buffer is not None:
                # Process radar peaks using the buffered image
                radar.process_radar_peaks(image["peaks"], image_buffer["peaks"])

            # Plot data and update GUI
            gui.plot_image_data(image, radar.frame_distances, radar.polar_data, radar.hit_grid, radar.servo_angle)
            if gui.step_through.get():
                gui.window.mainloop()  # Await user input (press any key to proceed)
            radar.increment_servo(config.radar_increment)  # Rotate and move on to the next frame

    else:
        raise Exception("No valid mode of operation selected")


if __name__ == "__main__":
    main()


