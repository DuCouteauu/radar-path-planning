# radar-path-planning

This goal of this project is to design and implement a system that can scan a surrounding environment using radar imaging, and plan/execute a path to the desired location.

# Running the Simulation

In order to test the system using simulated data, the user can simply run the main.py script and click "Run Simulated" in the GUI. The simulation will use data from test_data.py.

# Running the Connected System

Testing the system using real hardware is much more complicated:

- In order to use the connected radar system, this entire project repository must first be copied into the acconeer-python-exploration repo to process the radar data. The repository can be found at https://github.com/acconeer/acconeer-python-exploration (Copy the project into the "examples" directory).  Additionally, in the README of that repository are instructions to install and setup the acconeer python package, which is necessary to run the radar system. Any import statements that use this acconeer package have been commented out as it allows the simulated system to be run without being in the acconeer repository. Uncomment these import statements in main.py and radar_processing.py if you would like to run the connected system.
- The arduino must be running the CapstoneFirmata.ino file (a modified version of the Firmata protocol) so it can process instructions from a Python script through the PyFirmata library. Upload this file to the arduino through the arduino IDE before running the Python project.
- Make sure that the ports in config.py correspond to the correct devices.
- Once both the arduino and radar system have been configured, the system can be run in connected mode
- Keep all devices connected via USB until the path has been planned and uploaded (the status in the top left of the GUI will change from "Uploading" to "Executing"). Once this happens, there will be a 10 second delay before the vehicle follows the planned path, the arduino and radar system must be disconnected during this delay so the vehicle can move freely.
