from robot_systems.robot import HamBot
from robot_systems.camera import Camera

import time
import math 
import statistics
import random

class MyRobot(HamBot):
    # Physical dimensions
    WHEEL_RADIUS=0.045
    AXLE_WIDTH = 0.205
    DT = 0.1
    MAX_SPEED = 30 # was 60
    MIN_SPEED = 10
    ROTATE_MAX_SP = 30
    ROTATE_MIN_SP = 5
    LIDAR_TOLERANCE = 0.025
    ENCODER_TOLERANCE = 0.3
    ANGLE_TOLERANCE = 2.0
    IMAGE_TOLERANCE = 0.05 # Adjust the tolerance, since it is in pixels. 
    DEPTH_MATCH_TOLERANCE = 0.5 # Adjust the tolerance for depth matching.
    WALL_LOST = 0.4 # was 0.35 and 0.3
    FRONT_CLEARANCE = 0.3
    CELL_LIMIT = 0.45

    # (N, E, S, W) — 1 means "wall on that side"
    MAP = {
        1: (1, 0, 0, 1),
        2: (1, 1, 0, 0),
        3: (1, 0, 1, 1),
        4: (1, 1, 0, 0),
        5: (0, 1, 0, 1),
        6: (0, 0, 1, 1),
        7: (1, 1, 0, 0),
        8: (0, 1, 0, 1),
        9: (0, 0, 0, 1),
        10: (1, 1, 0, 0),
        11: (0, 0, 1, 1),
        12: (0, 1, 0, 0),
        13: (0, 1, 1, 1),
        14: (0, 0, 1, 1),
        15: (1, 1, 1, 0),
        16: (0, 1, 1, 1)
    }



    def __init__(self):
        super().__init__(drivetrain=HamBot.DRIVE_2WD, lidar_enabled=True, camera_enabled=True, camera_type=Camera.CAM_PICAM)

        if self.camera is None:
            self.disconnect_robot()
            raise RuntimeError("Camera not initialized.")
        
        self.camera.set_target_colors(
            (80, 220, 220),
            hue_tolerance=15,
            min_saturation=100,
            min_value=60,
        )

        self.forward_motion = True

    """
    ---- Helper functions ----
    """

    def run_motors(self, left_speed, right_speed):

        self.set_right_motor_speed(right_speed)
        self.set_left_motor_speed(left_speed)

    # Return the distance to the front object in metres, using the LiDAR. If no object is detected, return 100 metres.
    def get_front_distance(self, timeout=0.1):
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            front = self.get_range_image()[180]

            if front > 0:
                return front / 1000.0  # metres
            elif front < 0:
                return 100.0

            time.sleep(0.005)

        raise RuntimeError("LiDAR reading timed out.")
    
    # Saturate the speed to be within the allowed range
    def saturation(self, speed, cap=MAX_SPEED, rotate=False):
        if not rotate:
            if speed > cap:
                return cap
            elif speed < -cap:
                return -cap
            else:
                return speed
        else:
            if speed > self.ROTATE_MAX_SP:
                return self.ROTATE_MAX_SP
            elif speed < -self.ROTATE_MAX_SP:
                return -self.ROTATE_MAX_SP
            elif 0 < speed < self.ROTATE_MIN_SP:
                return self.ROTATE_MIN_SP
            elif -self.ROTATE_MIN_SP < speed < 0:
                return -self.ROTATE_MIN_SP
            else:
                return speed

    # Return the average of the left and right wheel encoder readings in radians
    def get_aver_encoders(self):
        """Return the mean wheel-encoder position in radians."""
        return sum(self.get_encoder_readings()) / len(self.get_encoder_readings())

    # Compute the PID output based on the current error and previous error
    def compute_pid(self, kp, ki, kd, integral, error, prev_error):
        proportional = kp * error

        integral += error * self.DT
        integral_term = ki * integral

        if prev_error is not None:
            derivative = (error - prev_error) / self.DT
        else:
            derivative = 0.0

        derivative_term = kd * derivative

        output = proportional + integral_term + derivative_term

        return output, integral, error

    # Return the width of the camera frame in pixels
    def get_width(self):
        frame = self.camera.get_frame()

        if frame is not None:
            return frame.shape[1]  # Return the width of the frame
        return 0
    """
    ---- Localization: Trilateration functions ----
    """

    # Each physical cell is 2 feet wide. Use meters throughout the calculations.
    CELL_SIZE_M = 0.6096
    # Use the four rows and four columns shown inside the physical maze in Figure 5b.
    GRID_SIZE = 4
    # The origin is at the grid center. Divide the full width by two to find each edge.
    HALF_WIDTH_M = GRID_SIZE * CELL_SIZE_M / 2

    # Each color is a dictionary key for its known cone center (x, y) in meters.
    # Positive x points east and positive y points north. These values assume the
    # cone centers sit exactly on the corners. Use measured coordinates if they differ.
    LANDMARKS = {
        "yellow": (-HALF_WIDTH_M, HALF_WIDTH_M),
        "red": (HALF_WIDTH_M, HALF_WIDTH_M),
        "green": (-HALF_WIDTH_M, -HALF_WIDTH_M),
        "blue": (HALF_WIDTH_M, -HALF_WIDTH_M),
    }

    def circle_difference(self, first, second):
        """Subtract two circle equations to get a line: a*x + b*y = c."""
        # Each input contains a cone center and the measured distance to that center.
        # Subtraction cancels the squared robot coordinates and leaves a straight line.
        (x1, y1), r1 = first
        (x2, y2), r2 = second
        return (
            2 * (x2 - x1),
            2 * (y2 - y1),
            r1**2 - r2**2 - x1**2 + x2**2 - y1**2 + y2**2,
        )

    def trilaterate(self, distances):
        """Return the robot's estimated (x, y) in meters from three distances by color."""
        # The guide uses three landmarks to build two equations for the robot's position.
        if len(distances) != 3:
            raise ValueError("Provide distances to exactly three landmarks.")

        # Each circle marks possible robot positions on the map. Its center is the
        # cone's known (x, y), and its radius is the measured distance to that center.
        # Start with an empty list and add one circle for each usable measurement.
        circles = []
        # Read one (color, distance) pair at a time from the input dictionary.
        for color, distance in distances.items():
            # Check the color before looking up its coordinates. Reject distances
            # such as NaN, infinity, zero, or negative values because they cannot
            # describe a usable radius for this calculation.
            if color not in self.LANDMARKS or not math.isfinite(distance) or distance <= 0:
                raise ValueError(f"Invalid landmark or distance: {color} = {distance}")
            # Use the color to look up the cone's coordinates in self.LANDMARKS.
            # append() adds one entry to the list, for example
            # ((-1.2192, 1.2192), 1.0) for a yellow cone measured at 1 meter.
            circles.append((self.LANDMARKS[color], distance))

        # Python list indexes start at zero. Entries 0 and 1 are the first two cones.
        # self.circle_difference() subtracts their circle equations and returns the
        # three coefficients of the first line, a*x + b*y = c.
        a, b, c = self.circle_difference(circles[0], circles[1])
        # Repeat with the second and third cones. The new coefficients describe
        # d*x + e*y = f. We need both lines to solve for the robot's x and y below.
        d, e, f = self.circle_difference(circles[1], circles[2])
        # Solve for the intersection. A near-zero determinant means the two lines
        # cannot give a reliable unique position, so check before dividing.
        determinant = a * e - b * d
        if math.isclose(determinant, 0.0, abs_tol=1e-9):
            raise ValueError("Landmark geometry does not give a unique position.")
        # Solve the two line equations for the robot position (x, y).
        # Both coordinates use the same determinant after flipping both signs for y.
        return (c * e - b * f) / determinant, (a * f - c * d) / determinant

    def grid_cell(self, x, y):
        """Number the cells west to east, starting with the northernmost row."""
        if not (-self.HALF_WIDTH_M <= x <= self.HALF_WIDTH_M
                and -self.HALF_WIDTH_M <= y <= self.HALF_WIDTH_M):
            return None
        # Measure x from the west edge and y from the north edge, then divide by
        # the cell width. Keep points exactly on the outer edge in the last cell.
        column = min(self.GRID_SIZE - 1, int((x + self.HALF_WIDTH_M) / self.CELL_SIZE_M))
        row = min(self.GRID_SIZE - 1, int((self.HALF_WIDTH_M - y) / self.CELL_SIZE_M))
        # Row and column start at zero. The assignment numbers its first cell as 1.
        return row * self.GRID_SIZE + column + 1

    def diagnose(self, hue, seconds):
        """Display one color detection and raw forward LiDAR readings at rest."""
        if self.camera is None:
            raise RuntimeError("Pi Camera did not initialize.")
        # Start with the demo's thresholds. Real colors need an on-site check.
        self.camera.set_target_colors(
            (hue, 200, 150), hue_tolerance=15, min_saturation=80, min_value=60
        )
        time.sleep(2)
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            # The camera object provides the image and color detections. Report
            # the blob center and width in pixels so we can check alignment.
            frame = self.camera.get_frame()
            landmarks = self.camera.find_landmarks(min_area=800) if frame is not None else []
            if landmarks:
                landmark = landmarks[0]
                print(f"Target H={hue}: x={landmark.x}, "
                      f"image width={frame.shape[1]}, box width={landmark.width}")
            else:
                print(f"Target H={hue}: no detection")

            # HamBot.get_range_image() returns the LiDAR scan in millimeters.
            # Index 180 points forward. Show nearby readings individually so we
            # can compare the cone and wall returns against a tape measurement.
            scan = self.get_range_image()
            if scan is not None and not isinstance(scan, (int, float)) and len(scan) == 360:
                readings = " ".join(
                    f"{index}={scan[index] if scan[index] > 0 else 'invalid'}"
                    for index in range(170, 191)
                )
                print(f"Front LiDAR (mm): {readings}")
            else:
                print("Front LiDAR: no valid scan")
            time.sleep(0.5)

    def surface_at_ray_mm(self, scan, ray):
        """Read the projected ray and check that a neighbor supports its range."""
        if scan is None or isinstance(scan, (int, float)) or len(scan) != 360:
            return None

        # Keep the projected ray as the anchor. A nearer return several rays away
        # can belong to a wall or a different cylinder.
        center = scan[ray]
        if not math.isfinite(center) or center <= 0:
            return None
        # Python excludes the slice endpoint. For ray 179, this reads 178, 179, 180.
        # Use the selected ray as the reference and keep valid ranges within 35 mm.
        nearby = [mm for mm in scan[ray - 1:ray + 2]
                  if math.isfinite(mm) and mm > 0 and abs(mm - center) <= 35]
        # The selected ray is included, so a second reading confirms it from one side.
        return statistics.median(nearby) if len(nearby) >= 2 else None

    def save_trace_photo(self, frame, box, color, ray, distance):
        """Save the accepted camera detection and its paired LiDAR measurement."""
        import cv2

        photo = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        x, y, width, height = box
        cv2.rectangle(photo, (int(x - width / 2), int(y - height / 2)),
                      (int(x + width / 2), int(y + height / 2)), (0, 255, 0), 2)
        label = f"{color} | LiDAR {ray} | {distance:.3f} m"
        cv2.putText(photo, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, (0, 255, 0), 2)
        # A new name on each run avoids mistaking an old capture for this run.
        path = f"trace_{color}_{time.time_ns()}.jpg"
        if cv2.imwrite(path, photo):
            print(f"Saved trace photo: {path}", flush=True)
        else:
            print(f"Could not save trace photo: {path}", flush=True)

    def collect_distances(self, hues, aim_columns, radius_m, seconds, trace=False,
                          pixels_per_degree=30.0):
        """Collect up to three distinct cylinder distances during one turn."""
        # Store the center distance in meters and the IMU-relative bearing by color.
        # Bearings help avoid assigning nearly the same direction to two landmarks.
        distances = {}
        bearings = {}
        deadline = time.monotonic() + seconds
        previous_heading = None
        turned = 0.0
        try:
            while time.monotonic() < deadline:
                # HamBot's IMU uses degrees counterclockwise from east. Keep track
                # of the turn even when the reported heading wraps around zero.
                heading = self.get_heading(blocking=True)
                if heading is None or not math.isfinite(heading):
                    raise RuntimeError("No valid IMU heading. Search stopped.")
                if previous_heading is not None:
                    turned += (heading - previous_heading + 180) % 360 - 180
                previous_heading = heading
                fine_step = False
                candidates = []

                # The wheels stay stopped while the camera checks each remaining
                # color. Select the candidate closest to the calibrated direction.
                for color, hue in hues.items():
                    if color in distances:
                        continue
                    # The corner photos showed that a higher saturation floor
                    # keeps pale wood out of the red and yellow detection boxes.
                    self.camera.set_target_colors(
                        (hue, 200, 150), hue_tolerance=15,
                        min_saturation=130 if color in ("yellow", "red") else 80,
                        min_value=60,
                    )
                    landmarks = self.camera.find_landmarks(min_area=800)
                    # Keep a frame from this stopped pose, before the next pivot.
                    # The camera API may use an adjacent frame for detection.
                    frame = self.camera.get_frame() if trace and landmarks else None
                    for landmark in landmarks:
                        # A narrow strip of background can pass the shape test.
                        # At 640x480, the false chair detections were under 90 px
                        # tall, while the confirmed distant cylinders exceeded 200 px.
                        # Require 120 px before pairing the color with a LiDAR range.
                        if landmark.height < 120 or landmark.height < 1.4 * landmark.width:
                            if trace:
                                print(f"{color}: rejected camera box "
                                      f"({landmark.width}x{landmark.height} px)", flush=True)
                            continue
                        # Compare the detected center with the calibrated image column.
                        # Divide the pixel difference by the scale to estimate degrees.
                        angle_error = (landmark.x - aim_columns[color]) / pixels_per_degree
                        if 0 < angle_error <= 15:
                            fine_step = True
                        if abs(angle_error) <= 3:
                            box = (landmark.x, landmark.y, landmark.width, landmark.height)
                            candidates.append((abs(angle_error), color, angle_error, box, frame))

                if candidates:
                    _, color, angle_error, box, frame = min(candidates, key=lambda item: item[:2])
                    pixel = box[0]
                    # Index 180 faces forward. On this camera, increasing x points
                    # toward smaller LiDAR indices. The conversion is approximate.
                    ray = round(180 - angle_error)
                    # Lower LiDAR indices point left, which increases IMU bearing.
                    bearing = (heading + 180 - ray) % 360
                    duplicate = any(
                        abs((bearing - previous + 180) % 360 - 180) <= 8
                        for previous in bearings.values()
                    )
                    readings = []
                    # The green cylinder measured about 70 px wide at 1.9 m.
                    # Width times range is roughly constant for the same object.
                    # Use a deliberately broad band to catch gross mismatches.
                    # Other colors need their own calibration before using it.
                    width = box[2]
                    range_matches_image = lambda mm: (
                        color != "green" or 85 <= width * (mm / 1000 + radius_m) <= 200
                    )
                    # Check nearby rays during the same stop. Store each ray
                    # separately so different directions cannot confirm each other.
                    nearby_readings = {index: [] for index in (ray - 1, ray + 1, ray - 2, ray + 2)}
                    if not duplicate:
                        # Sample the same direction while stopped. A corner may
                        # produce a peak in range, so check repeatability instead
                        # of requiring the cylinder to be closer than both walls.
                        # Allow two extra reads when the LiDAR briefly misses the
                        # cylinder. Stop sampling once three usable reads are ready.
                        for sample in range(5):
                            if sample:
                                time.sleep(0.25)
                            scan = self.get_range_image()
                            surface = self.surface_at_ray_mm(scan, ray)
                            if surface is not None and range_matches_image(surface):
                                readings.append(surface)
                                if len(readings) == 3:
                                    break
                            elif trace and surface is not None:
                                print(f"{color}: ray {ray} rejected (range disagrees with "
                                      f"camera width {width} px): {surface:.1f} mm", flush=True)
                            if color == "green" or (
                                scan is not None and not isinstance(scan, (int, float))
                                and len(scan) == 360
                                and (not math.isfinite(scan[ray]) or scan[ray] <= 0)
                            ):
                                for index, samples in nearby_readings.items():
                                    neighbor = self.surface_at_ray_mm(scan, index)
                                    if neighbor is not None and range_matches_image(neighbor):
                                        samples.append(neighbor)

                    # Prefer the projected ray. For green, a neighbor can also
                    # rescue an incomplete or unstable set of primary readings.
                    if not readings or (color == "green" and (
                        len(readings) < 3 or max(readings) - min(readings) > 35
                    )):
                        for index, samples in nearby_readings.items():
                            if len(samples) < 3 or max(samples) - min(samples) > 35:
                                continue
                            neighbor_bearing = (heading + 180 - index) % 360
                            if any(abs((neighbor_bearing - previous + 180) % 360 - 180) <= 8
                                   for previous in bearings.values()):
                                continue
                            if trace:
                                print(f"{color}: ray {ray} unusable, using neighbor {index}",
                                      flush=True)
                            ray, bearing, readings = index, neighbor_bearing, samples[:3]
                            break

                    if trace:
                        print(f"{color}: x={pixel}, LiDAR ray={ray}, bearing={bearing:.1f}, "
                              f"surface samples (mm)={readings}, duplicate={duplicate}", flush=True)
                    if len(readings) == 3 and max(readings) - min(readings) <= 35:
                        # Use the median of three agreeing surface ranges at this stop.
                        # Convert millimeters to meters and add the estimated radius.
                        distances[color] = statistics.median(readings) / 1000 + radius_m
                        bearings[color] = bearing
                        print(f"Recorded {color}: {distances[color]:.3f} m", flush=True)
                        if frame is not None:
                            self.save_trace_photo(frame, box, color, ray, distances[color])
                        # Three distinct colors supply the three ranges used by the
                        # professor's formula. The finally block stops the motors.
                        if len(distances) == 3:
                            return distances
                    elif trace:
                        print(f"{color}: rejected ({'direction already used' if duplicate else 'unstable or missing range'})",
                              flush=True)

                if abs(turned) >= 360 or time.monotonic() >= deadline:
                    break

                # Make shorter turns near a landmark to reduce overshoot. Stop
                # after each turn and let both sensors refresh before measuring.
                self.set_left_motor_speed(-15)
                self.set_right_motor_speed(15)
                time.sleep(0.04 if fine_step else 0.10)
                self.stop_motors()
                time.sleep(0.5)
        finally:
            self.stop_motors()
        return distances

    def localize(self, hues, aim_columns, radius_m, seconds, trace=False, pixels_per_degree=30.0):
        """Run the search on the physical HamBot and print the position and cell."""
        if self.camera is None:
            raise RuntimeError("Pi Camera did not initialize.")
        time.sleep(2)
        distances = self.collect_distances(
            hues, aim_columns, radius_m, seconds, trace, pixels_per_degree
        )
        if len(distances) < 3:
            print(f"Search ended with {len(distances)} of 3 usable distances.")
            return
        x, y = self.trilaterate(distances)
        print(f"Estimated position: ({x:.3f}, {y:.3f}) m")
        print(f"Estimated cell: {self.grid_cell(x, y)}")

    """
    ---- Localization: Particle Filter functions ----
    """

    def check_walls(self):

        # Get the LiDAR scan and check the distances in the four cardinal directions
        scan = self.get_range_image()
        
        front_values = [d for d in scan[175:186] if math.isfinite(d) and d > 0]
        front = statistics.median(front_values) / 1000.0 if front_values else None

        left_values = [d for d in scan[85:96] if math.isfinite(d) and d > 0]
        left = statistics.median(left_values) / 1000.0 if left_values else None

        right_values = [d for d in scan[265:276] if math.isfinite(d) and d > 0]
        right = statistics.median(right_values) / 1000.0 if right_values else None

        back_values = [d for d in scan[0:6] + scan[355:360] if math.isfinite(d) and d > 0]
        back = statistics.median(back_values) / 1000.0 if back_values else None

        walls = [0, 0, 0, 0] # [forward, right, backward, left]

        # Check if there is a wall in each direction
        if front is not None and front < self.CELL_LIMIT:
            walls[0] = 1
        if right is not None and right < self.CELL_LIMIT:
            walls[1] = 1
        if back is not None and back < self.CELL_LIMIT:
            walls[2] = 1
        if left is not None and left < self.CELL_LIMIT:
            walls[3] = 1

        return walls


    def choose_action(self):
        walls = self.check_walls()
        actions = ["forward", "right", "backward", "left"]

        # Prefer forward movement if there are no walls
        if walls[0] == 0:
            return "forward"

        # Pick a random action
        open_actions = [action for action, wall in zip(actions, walls) if wall == 0]
        if open_actions:
            return random.choice(open_actions)
        
        return None

    def execute_action(self, action):
        
        # Rotate the robot based on the action
        if action == "right":
            self.compass_pid(-80, 0.7, 0.02, 0.05)
        if action == "backward":
            self.compass_pid(170, 0.7, 0.02, 0.05)
        if action == "left":
            self.compass_pid(80, 0.7, 0.02, 0.05)

        # Move forward a cell
        self.encoder_pid(0.6, 6, 0.2, 0.4)

    def update_particle(self, particle, action):
        cell, orientation = particle

        directions = ["N", "E", "S", "W"]
        turns = {"forward": 0, "right": 1, "backward": 2, "left": -1}

        # Rotate a particle based on the action 
        heading_index = (directions.index(orientation) + turns[action]) % 4
        orientation = directions[heading_index]

        walls = self.MAP[cell]

        # Determine the new cell based on the action and orientation
        if action == "forward":
            if orientation == "N" and walls[0] == 0:
                cell -= 4
            elif orientation == "E" and walls[1] == 0:
                cell += 1
            elif orientation == "S" and walls[2] == 0:
                cell += 4
            elif orientation == "W" and walls[3] == 0:
                cell -= 1

        elif action == "right":
            if orientation == "N" and walls[1] == 0:
                cell += 1
            elif orientation == "E" and walls[2] == 0:
                cell += 4
            elif orientation == "S" and walls[3] == 0:
                cell -= 1
            elif orientation == "W" and walls[0] == 0:
                cell -= 4

        elif action == "backward":
            if orientation == "N" and walls[2] == 0:
                cell += 4
            elif orientation == "E" and walls[3] == 0:
                cell -= 1
            elif orientation == "S" and walls[0] == 0:
                cell -= 4
            elif orientation == "W" and walls[1] == 0:
                cell += 1

        elif action == "left":
            if orientation == "N" and walls[3] == 0:
                cell -= 1
            elif orientation == "E" and walls[0] == 0:
                cell -= 4
            elif orientation == "S" and walls[1] == 0:
                cell += 1
            elif orientation == "W" and walls[2] == 0:
                cell += 4

        return (cell, orientation)

    def observe_walls(self):
        heading = self.get_heading()
        if heading is None or not math.isfinite(heading):
            raise RuntimeError("Invalid compass reading")

        heading = (int((heading % 360 + 45) // 90) * 90) % 360
        front, right, back, left = self.check_walls()

        # Convert local heading to global wall measurements 
        if heading == 0:
            mesured_walls = [left,  front, right, back]
        elif heading == 90:
            mesured_walls = [front, right, back,  left]
        elif heading == 180:
            mesured_walls = [right, back,  left,  front]
        elif heading == 270:
            mesured_walls = [back,  left,  front, right]

        return mesured_walls

    def sensor_model(self, s, z):
        # Provided sensor model probabilities
        if s == 0 and z == 0:
            return 0.6
        elif s == 0 and z == 1:
            return 0.4
        elif s == 1 and z == 0:
            return 0.2
        elif s == 1 and z == 1:
            return 0.8

    def normalize(self, weights):
        # Normalize the weights to sum to 1
        total_weight = sum(weights)
        if total_weight == 0:
            raise ValueError("Total weight is zero, cannot normalize.")
        return [w / total_weight for w in weights]

    def get_histogram(self, cells):
        # Get the histogram of cell visits
        counts = dict.fromkeys(self.MAP, 0)
        for cell in cells:
            counts[cell] += 1
        return counts

    def print_grid(self, counts):
        # Print the grid of cell counts in a 4x4 format
        for start in range(1, 17, 4):
            print(" ".join(f"{counts[cell]:4d}" for cell in range(start, start + 4)))

    def particle_filter(self):
        # Declare variables 
        num_particles = 160
        particles = [ (cell, random.choice(["N", "E", "S", "W"])) 
                     for cell in range(1, 17) for _ in range(1, 11) ]

        weights = [1.0 / num_particles] * num_particles
        iteration = 0

        while True: 
            print(f"Iteration: {iteration}")
            iteration += 1

            # Choose an action for the robot
            action = self.choose_action()

            if action is None:
                print("No available actions: stopping robot.")
                self.stop_motors()
                break

            # Execute action and move particles accordingly
            self.execute_action(action)
            for i in range(num_particles):
                particles[i] = self.update_particle(particles[i], action)

            # Get the wall measurements 
            mes_z = self.observe_walls()
            for i in range(num_particles):
                particle = particles[i]
                cell = particle[0]
                expected_walls = self.MAP[cell]

                # Compute the likelihood 
                likelihood = 1.0
                for j in range(4):
                    likelihood *= self.sensor_model(expected_walls[j], mes_z[j])
                
                weights[i] = likelihood

            # Normalize the weights
            weights = self.normalize(weights)

            # Resample particles based on their weights
            drawn = random.choices(particles, weights=weights, k=num_particles)
            particles = [ (p[0], random.choice(['N','E','S','W'])) for p in drawn ]
            weights = [1.0 / num_particles] * num_particles

            # Get the histogram of particle states
            counts = self.get_histogram(p[0] for p in particles)
            self.print_grid(counts)

            # Determine the most likely cell and its concentration
            mode_cell = max(counts, key=counts.get)
            concentration = counts[mode_cell] / num_particles
            localized = concentration >= 0.80

            print(f"Mode cell: {mode_cell}")
            print(f"Concentration: {concentration:.1%}")
            print(f"80% localized: {localized}")

            # Finish if the robot is localized 
            if localized:
                print(f"Robot is localized in cell {mode_cell}")
                break

    """
    ---- Bug Zero Algorithm functions ----
    """
    def check_landmarks(self):
        landmarks = self.camera.find_landmarks()
        if not landmarks:
            return False
        return True

    # Move towards the goal using PID control 
    def move_to_goal(self, kp_fw, ki_fw, kd_fw, kp_st, ki_st, kd_st):
        front_integral = 0
        steering_integral = 0
        prev_front_error = None 
        prev_steering_error = None
        target_distance = 0.25
        image_width = self.get_width()
        
        while True:

            landmarks = self.camera.find_landmarks()

            scan = self.get_range_image()

            front_values = [d for d in scan[170:190] if math.isfinite(d) and d > 0]

            cur_dist = min(front_values) / 1000.0 if front_values else None

            if len(landmarks) > 0:

                # Compute the front movement PID output 
                front_error = cur_dist - target_distance

                forward_speed, front_integral, prev_front_error = self.compute_pid(kp_fw, ki_fw, kd_fw, front_integral, front_error, prev_front_error)

                # Compute the steering PID output
                goal = landmarks[0]
                goal_x = goal.x

                midpoint = image_width / 2

                steering_error = goal_x - midpoint

                steering_output, steering_integral, prev_steering_error = self.compute_pid(kp_st, ki_st, kd_st, steering_integral, steering_error, prev_steering_error)

                turn = -steering_output
                left_speed = forward_speed + turn
                right_speed = forward_speed - turn

                peak_speed = max(abs(left_speed), abs(right_speed))
                if peak_speed > self.MAX_SPEED:
                    scale = self.MAX_SPEED / peak_speed
                    left_speed *= scale
                    right_speed *= scale


                print(f"steering_error: {steering_error}, front_error: {front_error}, left_speed: {left_speed}, right_speed: {right_speed}")
               
                # Check if the robot has reached the goal
                if abs(steering_error) <= self.IMAGE_TOLERANCE * image_width and abs(front_error) <= self.LIDAR_TOLERANCE:
                    self.stop_motors()
                    return True
                

            else:
                # If no landmarks are detected, stop the robot
                self.stop_motors()
                print("No landmarks detected: stopping motors.")
                return False

            self.run_motors(left_speed, right_speed)

            time.sleep(self.DT)

    def follow_wall(self, safe_distance, wall_type,
                    kp_fw, ki_fw, kd_fw, kp_st, ki_st, kd_st):

        if wall_type not in ("left", "right"):
            raise ValueError("wall_type must be 'left' or 'right'")

        target_distance = 0.15

        front_integral = steering_integral = 0.0
        prev_front_error = prev_steering_error = None

        angle_gain = 15.0  # Starting value; motor-command units per radian.

        def sector_median(center):
            values = sorted(
                d / 1000.0
                for d in scan[center - 3:center + 4]
                if math.isfinite(d) and d > 0
            )
            return values[len(values) // 2] if len(values) >= 3 else None

        try:
            while True:
                scan = self.get_range_image()

                front_values = [d for d in scan[170:190] if math.isfinite(d) and d > 0]

                if not front_values:
                    self.stop_motors()
                    front_integral = steering_integral = 0.0
                    prev_front_error = prev_steering_error = None
                    time.sleep(self.DT)
                    continue

                front_distance = min(front_values) / 1000.0

                if self.forward_motion:
                    self.forward_motion = False
                    self.lidar_pid(target_distance * 2, 40, 4, 6)


                if front_distance < self.FRONT_CLEARANCE:
                    angle = -60 if wall_type == "left" else 60
                    self.compass_pid(angle, 0.7, 0.02, 0.05)
                    

                    front_integral = steering_integral = 0.0
                    prev_front_error = prev_steering_error = None
                    continue

                start, stop = (
                    (80, 140) if wall_type == "left" else (220, 280)
                )

                wall_values = [d for d in scan[start:stop] if math.isfinite(d) and d > 0]
                if not wall_values:
                    self.stop_motors()
                    front_integral = steering_integral = 0.0
                    prev_front_error = prev_steering_error = None
                    time.sleep(self.DT)
                    continue
                
                wall_distance = min(wall_values) / 1000.0


                # Forward PID, capped independently of steering.
                front_error = front_distance - target_distance
                forward, front_integral, prev_front_error = self.compute_pid(
                    kp_fw, ki_fw, kd_fw,
                    front_integral, front_error, prev_front_error
                )

                forward = self.saturation(forward, 20)

                # Side-distance steering PID.
                steering_error = wall_distance - safe_distance
                steering, steering_integral, prev_steering_error = (
                    self.compute_pid(
                        kp_st, ki_st, kd_st,
                        steering_integral, steering_error, prev_steering_error
                    )
                )


                # Estimate alignment using side and front-side measurements.
                side_center = 90 if wall_type == "left" else 270
                diagonal_center = 120 if wall_type == "left" else 240

                side = sector_median(side_center)
                diagonal = sector_median(diagonal_center)

                wall_angle = None

                if side is not None and diagonal is not None:
                    # Mirror the geometry for right-wall following:
                    # x = forward distance; y = distance toward the selected side.
                    x = diagonal * math.sin(math.radians(30))
                    y = diagonal * math.cos(math.radians(30))

                    candidate_angle = math.atan2(y - side, x)

                    # Basic plausibility gates. These do NOT prove both readings
                    # belong to the same wall.
                    if (
                        side < safe_distance + 0.20
                        and abs(candidate_angle) <= math.radians(35)
                    ):
                        wall_angle = candidate_angle
                        steering += angle_gain * wall_angle

                steering = self.saturation(steering)
                # Diagnostic constraint for normal wall following.
                forward = max(0.0, forward)
                steering_limit = 0.7 * forward
                steering = max(-steering_limit, min(steering, steering_limit))

                turn = steering if wall_type == "left" else -steering

                left_speed = forward - turn
                right_speed = forward + turn

                scale = max(1.0, abs(left_speed) / self.MAX_SPEED,
                                abs(right_speed) / self.MAX_SPEED)
                
                left_speed /= scale
                right_speed /= scale

                # True assumes the cylinder's full width is visible, without occlusion.
                if self.check_landmarks():
                    self.stop_motors()
                    return True

                print(
                    f"front={front_distance:.3f}, wall={wall_distance:.3f}, "
                    f"forward={forward:.2f}, steering={steering:.2f}, "
                    f"left={left_speed:.2f}, right={right_speed:.2f}"
                )

                self.run_motors(left_speed, right_speed)
                time.sleep(self.DT)

        finally:
            self.stop_motors()


    # Bug Zero algorithm that combines moving to the goal and following the wall
    def bug_zero(self, safe_distance, wall_type, pid_mtg_params, pid_wf_params):
        mtg_kp_fw, mtg_ki_fw, mtg_kd_fw, mtg_kp_st, mtg_ki_st, mtg_kd_st = pid_mtg_params
        wf_kp_fw, wf_ki_fw, wf_kd_fw, wf_kp_st, wf_ki_st, wf_kd_st = pid_wf_params

        while True:
            # Check if the goal is detected
            if self.move_to_goal(mtg_kp_fw, mtg_ki_fw, mtg_kd_fw, mtg_kp_st, mtg_ki_st, mtg_kd_st):
                print("Goal reached")
                return True

            # If the goal is not detected, follow the wall
            if self.follow_wall(safe_distance, wall_type, wf_kp_fw, wf_ki_fw, wf_kd_fw, wf_kp_st, wf_ki_st, wf_kd_st):
                continue

    
    """
    ---- PID control functions ----
    """


    def lidar_pid(self, target_distance, kp, ki, kd):
        integral = 0
        prev_error = None

        while True:
            cur_dist = self.get_front_distance()

            if cur_dist is None:
                self.stop_motors()
                print("Invalid LiDAR reading: stopping motors.")
                break   

            error = target_distance - cur_dist

            proportional = kp * error

            integral += error * self.DT
            integral_term = ki * integral

            if prev_error is not None:
                derivative = (error - prev_error) / self.DT
            else:
                derivative = 0.0

            derivative_term = kd * derivative

            output = proportional + integral_term + derivative_term

            motor_speed = self.saturation(-output)

            print(
                f"d={cur_dist:.3f}, e={error:.3f}, "
                f"P={proportional:.2f}, I={integral_term:.2f}, "
                f"D={derivative_term:.2f}, speed={motor_speed:.2f}"
            )

            if abs(error) <= self.LIDAR_TOLERANCE:

                self.stop_motors()
                break

            self.run_motors(motor_speed, motor_speed)
            prev_error = error
            time.sleep(self.DT)


    def encoder_pid(self, target_dist, kp, ki, kd):

        start_pos = self.get_aver_encoders()
        integral = 0.0
        prev_error = None
        
        # Convert the distance to radians
        dist_rad = target_dist / self.WHEEL_RADIUS

        while True:
            cur_theta = self.get_aver_encoders()

            if cur_theta is None:
                self.stop_motors()
                print("Invalid Encoder reading: stopping motors.")
                break 

            traveled = cur_theta - start_pos

            error = dist_rad - traveled

            proportional = kp * error

            integral += error * self.DT
            integral_term = ki * integral

            if prev_error is not None:
                derivative = (error - prev_error) / self.DT
            else:
                derivative = 0.0

            derivative_term = kd * derivative

            output = proportional + integral_term + derivative_term

            motor_speed = self.saturation(output)

            print(
                f"trav={traveled:.3f}, e={error:.3f}, "
                f"P={proportional:.2f}, I={integral_term:.2f}, "
                f"D={derivative_term:.2f}, speed={motor_speed:.2f}"
            )

            if abs(error) <= self.ENCODER_TOLERANCE:
                self.stop_motors()
                break

            self.run_motors(motor_speed, motor_speed)

            prev_error = error

            time.sleep(self.DT)

    def compass_pid(self, target_angle, kp, ki, kd):

        prev_angle = self.get_heading()

        integral = 0.0
        prev_error = None
        total_rotation = 0

        if target_angle >= 0:
            #Rotate counter-clockwise
            left = -1
            right = 1
            direction = 1
        else:
            #Rotate clockwise
            left = 1
            right = -1
            direction = -1

        while True:
            cur_angle = self.get_heading(blocking=True)

            if cur_angle is None:
                self.stop_motors()
                print("Invalid heading: stopping motors.")
                break

            delta_angle = (cur_angle - prev_angle + 180) % 360 - 180
            total_rotation += direction * delta_angle
            prev_angle = cur_angle

            error = abs(target_angle) - total_rotation

            proportional = kp * error

            integral += error * self.DT
            integral_term = ki * integral

            if prev_error is not None:
                derivative = (error - prev_error) / self.DT
            else:
                derivative = 0.0

            derivative_term = kd * derivative

            output = proportional + integral_term + derivative_term

            left_speed = self.saturation(left * output, rotate=True)
            right_speed = self.saturation(right * output, rotate=True)

            print(
                f"rotated={total_rotation:.3f}, e={error:.3f}, "
                f"P={proportional:.2f}, I={integral_term:.2f}, "
                f"D={derivative_term:.2f}, l_speed={left_speed:.2f}, r_speed={right_speed:.2f}"
            )

            """
            # True assumes the cylinder's full width is visible, without occlusion.
            if self.check_landmarks():
                self.stop_motors()
                return True
            """
            
            if abs(error) <= self.ANGLE_TOLERANCE:
                self.stop_motors()
                break

            self.run_motors(left_speed, right_speed)

            prev_error = error

            time.sleep(self.DT)

    """
    ---- Basic movement functions ----
    """

    def move_forward(self, speed, dist):
        """Drive forward for ``dist`` metres."""

        start_pos = self.get_aver_encoders()
        
        # Convert the distance in metres
        dist_rad = round(dist / self.WHEEL_RADIUS, 4)

        self.run_motors(speed, speed)

        while True:
            cur_tetha = self.get_aver_encoders()
            traveled = cur_tetha - start_pos

            # Stop when the robot has traveled the requested distance
            if traveled >= dist_rad - 1:
                
                self.stop_motors()

                break

            time.sleep(0.1)

    def rotate(self, speed, angle):
        """Turn in place by ``angle`` degrees; positive angles are CCW."""

        prev_angle = self.get_heading() 
        total_rotation = 0

        if angle > 0:
            # Empirical offset compensates for the robot's turning overshoot.
            angle -= 23
            left_speed = -speed
            right_speed = speed

            #self.last_wheel = True
        else:
            angle += 23
            left_speed = speed
            right_speed = -speed

            #self.last_wheel = False
        
        self.run_motors(left_speed, right_speed)

        while True:

            cur_angle = self.get_heading()

            # Normalize the heading change to the shortest signed angle across 0°/360°
            delta_angle = (cur_angle - prev_angle + 180) % 360 - 180

            total_rotation += delta_angle
            prev_angle = cur_angle

            if abs(total_rotation) >= abs(angle) - 1:
                self.stop_motors()
                break

            time.sleep(0.01) 

    def drive_circle(self, radius, angular_speed):
        """Drive one full circle with the requested radius and angular speed."""

        half_axle = self.AXLE_WIDTH / 2

        omega = abs(angular_speed)

        inner_speed = omega * (radius - half_axle) / self.WHEEL_RADIUS
        outer_speed = omega * (radius + half_axle) / self.WHEEL_RADIUS
        
        # Positive angular speed means counter-clockwise
        if angular_speed > 0:
            left_speed = inner_speed
            right_speed = outer_speed

            #self.last_wheel = True

        # Negative angular speed means clockwise
        else:
            left_speed = outer_speed
            right_speed = inner_speed

            #self.last_wheel = False

        prev_heading = self.get_heading()
        total_rotation = 0

        self.run_motors(left_speed, right_speed)

        while True:
            current_heading = self.get_heading()

            # Normalize the heading change to the shortest signed angle across 0°/360°
            delta_heading = ((current_heading - prev_heading + 180) % 360) - 180

            total_rotation += delta_heading
            prev_heading = current_heading

            # Stop when the robot has completed a full circle
            if abs(total_rotation) >= 345:
                self.stop_motors()
                break

            time.sleep(0.1)


