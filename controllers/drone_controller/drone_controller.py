from controller import Robot, GPS, Gyro, InertialUnit, Motor, LED, Camera
from controller import Emitter
from controller import Receiver
import math
import os
import json
from  genrate_path import generate_drone_path,plot_drone_path,generate_direct_path

def save_remaining_path(path, current_index, filename="remaining_path.json"):
    """
    Saves the waypoints in path[current_index:] to a JSON file
    alongside this script.
    """
    # take only the not-yet-visited waypoints
    remaining = path[current_index:]
    # build a path pointing to the same directory as this script
    dir_path = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(dir_path, filename)

    with open(file_path, "w") as f:
        json.dump(remaining, f, indent=2)
    print(f"Saved {len(remaining)} remaining waypoints to {file_path}")

def load_and_delete_remaining_path(filename="remaining_path.json"):
    """
    Reads the JSON file of remaining waypoints, deletes it, and returns the list.
    If the file doesn't exist, returns an empty list.
    """
    # locate the file alongside this script
    dir_path = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(dir_path, filename)

    try:
        with open(file_path, "r") as f:
            remaining = json.load(f)
    except FileNotFoundError:
        print(f"No file {file_path} found, nothing to load.")
        return []

    # delete it
    try:
        os.remove(file_path)
        print(f"Loaded and deleted {file_path}")
    except OSError as e:
        print(f"Warning: could not delete {file_path}: {e}")

    return remaining

def clamp(value, low, high):
    return max(low, min(value, high))

def distance(a, b):
    return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

def angle_diff(target_angle, current_angle):
    """Compute smallest angle difference [-π, π]."""
    diff = target_angle - current_angle
    while diff > math.pi:
        diff -= 2 * math.pi
    while diff < -math.pi:
        diff += 2 * math.pi
    return diff


robot = Robot()
timestep = int(robot.getBasicTimeStep())
max_battery = 10000

# Device initialization
imu = robot.getDevice("inertial unit")
imu.enable(timestep)
gps = robot.getDevice("gps")
gps.enable(timestep)
gyro = robot.getDevice("gyro")
gyro.enable(timestep)
led_left = robot.getDevice("front left led")
led_right = robot.getDevice("front right led")
cam_roll = robot.getDevice("camera roll")
cam_pitch = robot.getDevice("camera pitch")
camera = robot.getDevice("camera")
camera.enable(timestep)
robot.batterySensorEnable(timestep)
receiver = robot.getDevice('receiver')
receiver.enable(timestep)
emitter = robot.getDevice('emitter')

motors = [
    robot.getDevice("front left propeller"),
    robot.getDevice("front right propeller"),
    robot.getDevice("rear left propeller"),
    robot.getDevice("rear right propeller")
]
for motor in motors:
    motor.setPosition(float('inf'))
    motor.setVelocity(1.0)

# PID Constants
k_vertical_thrust = 68.5
k_vertical_offset = 0.6
k_vertical_p = 3.0
k_roll_p = 50.0
k_pitch_p = 30.0

k_horizontal_p = 5.0
deadzone = 0.05

yaw_threshold = 0.03  
forward_pitch = -1.5  # constant forward pitch once aligned

# Wait until GPS is ready
for _ in range(10):
    robot.step(timestep)

# Get starting location
start_pos = gps.getValues()

path, grid = generate_drone_path(start_pos[:2],10)

# Print the path
max_battery = robot.batterySensorGetValue()
plot_drone_path(path,grid)

current_target_index = 0
target = path[current_target_index]
ascending = True # Indicates whether the drone is still ascending/desending to its cruising altitude (e.g., 10 meters)
tolerance = 0.4 # Distance threshold (in meters) to determine if the drone has reached a waypoint
alt_tolerance = 0.2 # Altitude threshold (in meters) to consider the drone has reached the desired vertical level
print(f"New target {target}")
print("Starting path following...")
final_goal = False # Flag to indicate if the drone has entered the final return-to-home and landing phase
stay_still = False # Used to make the drone pause (used for Drone 2 to wait for a signal from Drone 1)
going_home = False # Setting if the drone's current path is going home or not (landing at charger)

#Keep drone 2 still till drone 1 is done
if robot.getName() == "Drone 2":
    stay_still = True

while robot.step(timestep) != -1:

    if stay_still:
        # If drone is in "stay" mode (e.g., Drone 2 waiting at charging station)
        if robot.batterySensorGetValue()/max_battery < 0.95:
            emitter.send("go".encode("utf-8"))
            while receiver.getQueueLength() > 0:
                # Clear any received messages
                incoming_msg = receiver.getString()
                receiver.nextPacket()

        else:
            # If battery is full, listen for a "go" signal to resume flying
            while receiver.getQueueLength() > 0:
                incoming_msg = receiver.getString()
                receiver.nextPacket()
                print(f"{robot.getName()}  recived message {incoming_msg}")
                if incoming_msg == "go":
                    # If "go" received, resume from last saved path
                    remaining = load_and_delete_remaining_path()
                    robot.step(1000) # pause briefly
                    if len(remaining)<5:
                        # If no usable path, generate a fresh path
                        path, grid = generate_drone_path(start_pos[:2],10)
                    else:
                        # Else resume from where it left off before returning home
                        path, grid = generate_direct_path(start_pos[:2],remaining[0][:2],10)
                        path += remaining

                        
                    # Print the path
                    plot_drone_path(path,grid)
                    stay_still = False
                    final_goal = False
                    going_home = False
                    ascending = True
                    current_target_index = 0
                    target = path[current_target_index]
                    receiver.disable() # Stop listening
                    break
        
    else:
        time = robot.getTime()
        if robot.batterySensorGetValue()/max_battery < 0.25 and not going_home and  current_target_index < len(path)-10:
                # If battery is low (<25%) and not already returning, and far from goal, go home
                save_remaining_path(path, current_target_index+1)

                #current_pos = gps.getValues()
                path, grid = generate_direct_path(path[current_target_index +1 ][:2],start_pos[:2],10)

                # Print the path
                current_target_index = 0
                going_home =  True # Mark as returning
                
                plot_drone_path(path,grid)
        
        # Read orientation and position from sensors
        roll, pitch, current_yaw = imu.getRollPitchYaw()
        altitude = gps.getValues()[2]
        x, y, _ = gps.getValues()

        roll_vel = gyro.getValues()[0]
        pitch_vel = gyro.getValues()[1]

        # Blink LEDs
        led_state = int(time) % 2
        led_left.set(led_state)
        led_right.set(1 - led_state)

        # Stabilize camera pitch based on pitch rate
        cam_pitch.setPosition(-0.1 * pitch_vel +1)

        # Get current target waypoint
        target_x, target_y, target_z = target

        # Compute direction vector to target
        dx = target_x - x
        dy = target_y - y
        dz = target_z - altitude

        distance_to_target = math.sqrt(dx**2 + dy**2 + dz**2)

        # If drone is close enough to the current target
        if distance_to_target < tolerance:
            current_target_index += 1
            if current_target_index == len(path)-1 and not final_goal:
                # If near final point and not already in final goal mode
                if robot.batterySensorGetValue()/max_battery > 0.749:
                    # If battery is sufficient, create new path to explore
                    path, grid = generate_drone_path(start_pos[:2],10)
                    current_target_index = 0
                else:
                    # Else prepare to land at starting position
                    path.append([start_pos[0], start_pos[1], 10])
                    path.append([start_pos[0], start_pos[1], 0.6])
                    final_goal = True
                    ascending = True
                
                
            if current_target_index == len(path):
                # If final target reached
                for motor in motors:
                    motor.setVelocity(0.0)
                stay_still = True # Enter charging/listening mode
                receiver.enable(timestep)
                current_target_index = 0
                pass
            
                
            target = path[current_target_index]
            print(f"New target {target}")
            print(f"Perecent {current_target_index*100/len(path)}% done")
            continue

        # Default disturbances
        pitch_disturbance = 0.0
        roll_disturbance = 0.0
        yaw_disturbance = 0.0

        if ascending:
            # Only control vertical motion
            if abs(dz) < alt_tolerance: 
                # Close enough to target altitude
                ascending = False
                print("Reached 10m altitude. Starting horizontal movement.")
        else:
            # Begin aligning yaw to face direction of next waypoint
            desired_yaw = math.atan2(dy, dx)
            yaw_error = angle_diff(desired_yaw, current_yaw)

            if abs(yaw_error) > yaw_threshold:
                # Apply yaw rotation until aligned with target
                yaw_disturbance = clamp(3.0 * yaw_error, -1.0, 1.0)
            else:
                # Once aligned, start moving forward
                pitch_disturbance = forward_pitch

        # PID stabilization
        roll_input = k_roll_p * clamp(roll, -1.0, 1.0) + roll_vel + roll_disturbance
        pitch_input = k_pitch_p * clamp(pitch, -1.0, 1.0) + pitch_vel + pitch_disturbance
        vertical_input = k_vertical_p * (clamp(dz + k_vertical_offset, -1.0, 1.0) ** 3)

        # Calculate motor speeds based on stabilization and movement commands
        motor_inputs = [
            k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_disturbance,
            k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_disturbance,
            k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_disturbance,
            k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_disturbance
        ]

        # Apply velocities to motors (motors 1 & 2 reversed for opposite direction)
        motors[0].setVelocity(motor_inputs[0])
        motors[1].setVelocity(-motor_inputs[1])
        motors[2].setVelocity(-motor_inputs[2])
        motors[3].setVelocity(motor_inputs[3])