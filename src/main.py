# This file will initialize the gui and radar system objects and call the processing and update_gui
import time
from argparse import Namespace
import radar_processing
import GUI
from pyfirmata import Arduino, SERVO, OUTPUT
import pathfinding
import config
# import acconeer.exptool as et
import numpy
import vehicle_control
import test_data
import serial
import serial_comms


def main():
    """Temporary main function"""

    # Initialize GUI
    gui = GUI.GUI()
    gui.change_status(gui.status_initializing)

    gui.window.mainloop()  # Await user input

    # Current system has a separate block for simulated/connected processes, TODO: Unify them
    if gui.mode == "simulated":

        # Initialize radar stub
        radar_stub = radar_processing.RadarStub()
        image = None
        image_buffer = None

        # Scanning loop
        gui.change_status(gui.status_scanning)
        for frame_number in range(config.rotations_per_scan):
            image_buffer = image  # Save previous image as a buffer
            # Simulated data, cycles through saved data sets until scan is complete
            radar_data = [test_data.front_radar_sim[frame_number % len(test_data.front_radar_sim)], test_data.back_radar_sim[frame_number % len(test_data.back_radar_sim)]]
            peaks = radar_stub.get_radar_data_peaks(radar_data)  # Get the peaks of the radar frame
            # Pack the data, peaks, and angle into a single image
            image = {"radar_data": radar_data,
                     "peaks": peaks,
                     "radar_angle": radar_stub.servo_angle}

            # Process peaks (only possible after more than 1 frame of radar data has been collected)
            if image_buffer is not None:
                radar_stub.process_radar_peaks(image["peaks"], image_buffer["peaks"])

            # Update GUI with all corresponding data for the given frame
            gui.plot_image_data(image, radar_stub.frame_distances, radar_stub.polar_data, radar_stub.hit_grid, radar_stub.servo_angle)
            if gui.step_through.get():
                gui.window.mainloop()  # Await user input (press any key to proceed)
            radar_stub.increment_servo(config.radar_increment)  # Rotate and move on to the next frame

        # Obtains the binary occupancy grid from the hit grid and plots it
        gui.plot_final_grid(radar_stub.hit_grid)
        gui.change_status(gui.status_pathing)

        # Generate binary grid from hits, TODO: Refactor section to grid_mapping.py
        # Current system assumes all hits are detected objects, TODO: Generalize this to be probabilistic
        binary_grid = (numpy.ceil(gui.final_grid)).astype(int)
        binary_grid[int((config.grid_size - 1) / 2)][int((config.grid_size - 1) / 2)] = 0

        # Get and plot the desired destination
        destination = gui.get_input()
        gui.plot_destination(destination)
        print(f"Resulting Binary Occupancy Grid:\n{binary_grid}")

        # Path Planning
        path = pathfinding.astar(binary_grid, config.grid_center,
                                 destination)  # x and y are flipped and y is inverse (on the plot)

        # Plot Path
        print(f"Planned Path: {path}")
        gui.plot_path(path)

        gui.window.mainloop()  # End of simulation

    elif gui.mode == "connected":
        # Arduino Setup
        board = Arduino(config.arduino_port, baudrate=57600)
        board.digital[config.arduino_servo_pin].mode = SERVO

        # Initialize radar system
        radar = radar_processing.RadarConnected(board)
        gui.change_status(gui.status_scanning)

        # This loop allows us to restart the scanning phase at different breakpoint in the process
        while True:
            image = None
            image_buffer = None

            # Rotate servo to its starting position
            board.digital[config.arduino_servo_pin].write(config.servo_offset)
            # Scanning Loop
            for frame_number in range(config.rotations_per_scan):
                image_buffer = image  # Save previous image as a buffer
                image = radar.take_and_average_images()  # Take new image

                if image_buffer is not None:
                    # Process radar peaks using the buffered image
                    radar.process_radar_peaks(image["peaks"], image_buffer["peaks"])

                # Plot data and update GUI
                gui.plot_image_data(image, radar.frame_distances, radar.polar_data, radar.hit_grid, radar.servo_angle)
                if gui.step_through.get():
                    gui.window.mainloop()  # Await user input (press any key to proceed)
                radar.increment_servo(config.radar_increment)  # Rotate and move on to the next frame_number

            # Obtains the binary occupancy grid from the hit grid and plots it
            gui.plot_final_grid(radar.hit_grid)
            gui.change_status(gui.status_pathing)
            binary_grid = (numpy.ceil(gui.final_grid)).astype(int)
            binary_grid[int((config.grid_size - 1) / 2)][int((config.grid_size - 1) / 2)] = 0

            # Get destination
            destination = gui.get_input()

            # Retry scan if user selects option
            if gui.trigger_retry_scan:
                gui.trigger_retry_scan = False
                radar.reset_system()
                continue

            # Close PyFirmata/radar communication
            board.sp.close()
            radar.client[0].disconnect()
            radar.client[1].disconnect()

            # Plan and plot path to destination
            gui.plot_destination(destination)
            print(f"Resulting Binary Occupancy Grid:\n{binary_grid}")
            path = pathfinding.astar(binary_grid, config.grid_center,
                                     destination)  # x and y are flipped and y is inverse (on the plot)

            print(f"Planned Path: {path}")
            gui.plot_path(path)

            gui.change_status(gui.status_uploading)

            # Initialize serial communication for instruction upload
            ser = serial.Serial("COM6", baudrate=57600)

            instructions = vehicle_control.path_to_instructions(path)  # Generate discrete instructions
            print(f"Vehicle Instructions: {instructions}")

            # Upload Instructions
            serial_comms.send_instructions(instructions, ser)
            print("UPLOAD COMPLETE")

            # Wait for user to trigger execution
            gui.change_status(gui.status_executing)
            gui.await_execution()
            if gui.trigger_retry_scan:
                gui.trigger_retry_scan = False
                radar.reset_system()
                ser.close()
                board = Arduino(config.arduino_port, baudrate=57600)
                board.digital[config.arduino_servo_pin].mode = SERVO
                radar.board = board
                time.sleep(3)
                continue

            # Send execution command (tell vehicle to follow instructions)
            serial_comms.confirm_commands(ser)

            # Vehicle will start moving after a delay (set in arduino code), this delay is used to unplug radars/arduino

            gui.window.mainloop()  # End of process

    else:
        raise Exception("No valid mode of operation selected")


if __name__ == "__main__":
    main()


