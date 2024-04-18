from argparse import Namespace
# import acconeer.exptool as et
import numpy as np
from time import sleep
import scipy.signal
import config
import grid_mapping

# TODO: Move util functions out of the RadarSystem class, as many of them are static

class RadarSystem:
    """The radar system collects and processes the radar data such that it can be plotted and used for path planning"""
    polar_data = {}
    servoCCW = True  # Indicates the direction of the next servo rotation
    servo_angle = 0
    hit_grid = np.zeros([config.grid_size, config.grid_size], dtype=int)  # Remember x and y are backwards in this grid

    # frame_distances is a list containing the distances that the radar system sweeps through
    # TODO Generalize this
    frame_distances = np.linspace(config.radar_range_interval[0],
                                     config.radar_range_interval[1],
                                     num=3098)
    for i, d in enumerate(frame_distances):
        if d > config.min_distance:
            min_cutoff_index = i
            break
    frame_distances = frame_distances[min_cutoff_index:]

    def reset_system(self):
        self.polar_data = {}
        self.servoCCW = True  # Indicates the direction of the next servo rotation
        self.servo_angle = 0
        self.hit_grid = np.zeros([config.grid_size, config.grid_size],
                            dtype=int)  # Remember x and y are backwards in this grid


    def round_data(self, data, decimal):
        # TODO Make a utils file for static functions like this
        """Rounds to 1 decimal place"""
        return [round(x, decimal) for x in data]

    def get_radar_data_peaks(self, radar_frame):
        """Extracts the peaks from the radar frame using the preset configuration"""
        peaks_front = scipy.signal.find_peaks(radar_frame[0], distance=config.peak_distance, width=config.peak_width,
                                              height=config.height, )[0]
        peaks_back = scipy.signal.find_peaks(radar_frame[1], distance=config.peak_distance, width=config.peak_width,
                                             height=config.height, )[0]

        peak_locations = np.take(self.frame_distances, peaks_front), np.take(self.frame_distances, peaks_back)
        peak_heights = np.take(radar_frame[0], peaks_front), np.take(radar_frame[1], peaks_back)

        front_peaks = dict(zip(peak_locations[0], peak_heights[0]))
        back_peaks = dict(zip(peak_locations[1], peak_heights[1]))

        return front_peaks, back_peaks

    def extract_peak_differences(self, peaks, buffer_peaks):
        # TODO Make a utils file for static functions like this
        """Returns the head hits and tail hits for a corresponding frame/buffer frame pair. This function is
        inefficient as it loops through both peaks and buffer peaks twice, if radar processing speed becomes a
        problem then this could be the main factor"""
        buffer_keys = list(buffer_peaks.keys())
        present_keys = list(peaks.keys())

        tail_hits = {}
        head_hits = {}

        # Debug print statements
        print(f"PEAKS: {peaks}")
        print(f"BUFFER PEAKS: {buffer_peaks}")
        # If a peak exists in buffer peaks that is not in present peaks, a tail hit is registered
        for buffer_key in buffer_keys:
            if len(present_keys) == 0:
                tail_hits.update({buffer_key: buffer_peaks[buffer_key]})
            else:
                for key in present_keys:
                    if abs(key - buffer_key) > config.max_peak_change_per_frame:
                        # Tail Hit
                        tail_hits.update({buffer_key: buffer_peaks[buffer_key]})

        # If a peak exists in present peaks that is not in buffer peaks, a head hit is registered
        for key in present_keys:
            if len(buffer_keys) == 0:
                head_hits.update({key: peaks[key]})
            else:
                for buffer_key in buffer_keys:
                    if abs(key - buffer_key) > config.max_peak_change_per_frame:
                        # Head Hit
                        head_hits.update({key: peaks[key]})

        return head_hits, tail_hits  # Returns 2 dicts

    def process_radar_peaks(self, peaks, buffer_peaks):
        """Processes radar peaks and updates polar data"""
        front_hits = self.extract_peak_differences(peaks[0], buffer_peaks[0])
        back_hits = self.extract_peak_differences(peaks[1], buffer_peaks[1])

        self.update_polar_data(front_hits, back_hits)

    def update_polar_data(self, front_hits, back_hits):
        """
        This is a complex function because the exact location of a given hit depends on 3 factors:
        1. Which direction is the servo rotating
        2. Is it the front or back radar?
        3. Is it a tail or head hit?
        There may be a better method but this works okay for now
        """
        polar_hits = {}
        if self.servoCCW:
            polar_hits.update({self.servo_angle + config.radar_field_size / 2 - config.radar_increment / 2:
                             list(front_hits[0].keys())})  # Front head hits
            polar_hits.update({self.servo_angle - config.radar_field_size / 2 - config.radar_increment / 2:
                             list(front_hits[1].keys())})  # Front tail hits

            polar_hits.update({180 + self.servo_angle + config.radar_field_size / 2 - config.radar_increment / 2:
                             list(back_hits[0].keys())})  # Back head hits
            polar_hits.update({180 + self.servo_angle - config.radar_field_size / 2 - config.radar_increment / 2:
                             list(back_hits[1].keys())})  # Back tail hits

        else:

            polar_hits.update({self.servo_angle - config.radar_field_size / 2 + config.radar_increment / 2:
                             list(front_hits[0].keys())})  # Front head hits
            polar_hits.update({self.servo_angle + config.radar_field_size / 2 + config.radar_increment / 2:
                             list(front_hits[1].keys())})  # Front tail hits

            polar_hits.update({180 + self.servo_angle - config.radar_field_size / 2 + config.radar_increment / 2:
                             list(back_hits[0].keys())})  # Back head hits
            polar_hits.update({180 + self.servo_angle + config.radar_field_size / 2 + config.radar_increment / 2:
                             list(back_hits[1].keys())})  # Back tail hits

        # Reformat polar_hits to remove empty data and dupes before updating the hit grid
        for i in polar_hits:
            if type(polar_hits[i]) is list:
                if i in self.polar_data:
                    self.polar_data[i] = [*self.polar_data[i], *polar_hits[i]]
                else:
                    self.polar_data[i] = polar_hits[i]

                self.update_grid_hit_data(i, polar_hits[i])

    def update_grid_hit_data(self, angle, distances: list):
        """Updates the probabilistic hit grid"""
        for distance in distances:
            x_cell, y_cell = grid_mapping.cart_to_cell(*grid_mapping.polar_to_cart(distance, np.radians(angle)))
            print(f"Hit Detected At: {x_cell}, {y_cell}")
            self.hit_grid[config.grid_size-y_cell][x_cell-1] = self.hit_grid[config.grid_size-y_cell][x_cell-1] + 1


class RadarConnected(RadarSystem):
    """The connected radar system"""
    def __init__(self, board):

        # Define radar config
        self.args = [
            Namespace(serial_port=config.front_radar_port, socket_addr=None, spi=False, sensors=[1], verbose=False,
                      debug=False, quiet=False),
            Namespace(serial_port=config.back_radar_port, socket_addr=None, spi=False, sensors=[1], verbose=False,
                      debug=False, quiet=False)]
        et.utils.config_logging(self.args[0])
        et.utils.config_logging(self.args[1])
        self.client = [et.a111.Client(**et.a111.get_client_args(self.args[0])),
                       et.a111.Client(**et.a111.get_client_args(self.args[1]))]

        self.radar_config = et.a111.EnvelopeServiceConfig()
        self.radar_config.downsampling_factor = config.downsampling_factor  # Tweaks range step size (Default == 1)
        self.radar_config.running_average_factor = config.running_average_factor
        self.radar_config.sensor = self.args[0].sensors

        # Set the measurement range [meter]
        self.radar_config.range_interval = config.radar_range_interval

        # Set the target measurement rate [Hz]
        self.radar_config.update_rate = config.refresh_rate

        # Connect to radar system
        self.client[0].connect()
        self.client[1].connect()

        self.session_info = [self.client[0].setup_session(self.radar_config),
                             self.client[1].setup_session(self.radar_config)]

        print("STARTING RADAR SESSION")
        print("Session info:\n", self.session_info[0], "\n")
        print("Session info:\n", self.session_info[1], "\n")

        # TODO Generalize this
        frame_distances = np.linspace(self.session_info[0]["range_start_m"],
                                         self.session_info[0]["range_start_m"] + self.session_info[0]["range_length_m"],
                                         num=self.session_info[0]["data_length"])

        for i, d in enumerate(frame_distances):
            if d > config.min_distance:
                min_cutoff_index = i
                break

        self.frame_distances = frame_distances[min_cutoff_index:]

        self.session_info = [self.client[0].setup_session(self.radar_config),
                             self.client[1].setup_session(self.radar_config)]
        self.client[0].start_session()
        self.client[1].start_session()

        self.board = board
        self.min_cutoff_index = 0

        self.radar_field_size = config.radar_field_size

    def rotate_servo(self, angle):
        """Rotates the servo to a desired angle, adds an artificial delay afterwards to prevent noisy radar data"""
        self.board.digital[config.arduino_servo_pin].write(angle + config.servo_offset)
        sleep(config.connected_rotation_delay)

    def increment_servo(self, theta):
        """Determines the angle of the next radar frame and rotates the servo accordingly"""
        if self.servo_angle == 180:
            self.servoCCW = False
        if self.servo_angle == 0:
            self.servoCCW = True

        if self.servoCCW:
            self.servo_angle = self.servo_angle + theta
        else:
            self.servo_angle = self.servo_angle - theta

        self.rotate_servo(self.servo_angle)

    def collect_data(self, number_of_frames):
        """Collects a frame (or multiple) of data"""
        i = 0
        data = []
        while i < number_of_frames:
            data_info1, data1 = self.client[0].get_next()
            data_info2, data2 = self.client[1].get_next()
            data1 = data1[self.min_cutoff_index:]
            data2 = data2[self.min_cutoff_index:]
            data.append((data1, data2))
            i = i + 1

        print("Radar 1 Sweep {}:\n".format(1), data_info1, "\n", data1, "\n")
        print("Radar 2 Sweep {}:\n".format(1), data_info2, "\n", data2, "\n")

        return data1, data2

    def take_image(self):
        """Collects radar data and processes it into an image of radar data, peak information, and angle"""
        data = self.collect_data(1)  # Collect radar frame

        # Get polar distance from radar frame
        peak_locations, peak_heights = self.get_radar_data_peaks(data)

        return {"radar_data": data, "peaks": peak_locations, "peak_heights": peak_heights,
                "radar_angle": self.servo_angle}

    def take_and_average_images(self):
        """Takes multiple images at a given angle and averages their results in an attempt to reduce noise"""
        front_images = []
        back_images = []
        for i in range(config.images_per_rotation):
            front_images.append(self.take_image()["radar_data"][0])

            back_images.append(self.take_image()["radar_data"][1])
        averages = [np.mean(front_images, axis=0), np.mean(back_images, axis=0)]
        peak_locations, peak_heights = self.get_radar_data_peaks(averages)
        return {"radar_data": averages, "peaks": [peak_locations, peak_heights],
                "radar_angle": self.servo_angle}


class RadarStub(RadarSystem):
    """The simulated radar system"""

    def rotate_servo(self):
        """Simulates the delay of a rotation"""
        sleep(config.sim_rotation_delay)

    def increment_servo(self, theta):
        """Determines the angle of the next radar frame and rotates the servo accordingly"""
        if self.servo_angle == 180:
            self.servoCCW = False
        if self.servo_angle == 0:
            self.servoCCW = True

        if self.servoCCW:
            self.servo_angle = self.servo_angle + theta
        else:
            self.servo_angle = self.servo_angle - theta

        self.rotate_servo()

