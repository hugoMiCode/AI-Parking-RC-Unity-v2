import sys
import os
import time
import numpy as np
import onnx
import onnxruntime
from onnx import numpy_helper
import matplotlib.pyplot as plt
import serial
import serial.tools.list_ports
import time
import numpy as np
from picarx import Picarx



HEADER = 0x54
POINT_PER_PACK = 12
LIDAR_PORT = '/dev/ttyUSB0'
BAUDRATE = 230400

threshold_lidar = 1000 # mm
maxAngle = 40 # degrees
maxThrottle = 10 # %



model_dir = "./model"
model_path = model_dir + "/Park90RC.onnx"
session = onnxruntime.InferenceSession(model_path, None)
ser = serial.Serial(
    port=LIDAR_PORT,
    baudrate=BAUDRATE,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)
px = Picarx()

def read_frame(frame):
    # the frame is 47 bytes

    if len(frame) < 47:
        return [], []

    header = frame[0]
    if header != HEADER:
        return[], []
    
    # verlen => frame[1]
    # speed => frame[2:4] # speed in degrees per second
    start_angle = int.from_bytes(frame[4:6], 'little') / 100.0 # angle in degree
    end_angle = int.from_bytes(frame[42:44], 'little') / 100.0 # angle in degree
    if start_angle > end_angle:
        end_angle += 360
    step = (end_angle - start_angle) / (POINT_PER_PACK - 1)

    angles = []
    distances = []
    for i in range(POINT_PER_PACK):
        index = 6 + i*3
        distances.append(int.from_bytes(frame[index:index+2], 'little'))  # Distance in mm
        angles.append((start_angle + i*step)%360)

    # timestamp => frame[44:47]
    # CRC_Check => frame[46]

    return (angles, distances)

def get_distance90(angles, distances):
    distances90 = [0]*90
    counts = np.zeros(90)
    for angle, distance in zip(angles, distances):
        if distance == 0:
            continue
        # Goes from an angle in [0, 360] to an index in [0, 90]
        # Needs to be fixed. Exemple: 
        #   - index of 0 corresponds to a span of -2 to 2 degrees
        #   - index of 1 corresponds to a span of 2 to 6 degrees etc..
        #  - index of 89 corresponds to a span of 354 to 358 degrees
        indice = int(((angle+0.5)/4)%90) 
        distances90[indice] += distance
        counts[indice] += 1
    for i in range(len(distances90)):
        if counts[i] == 0:
            continue
        distances90[i] = distances90[i] / counts[i] 

    # Normalize the distances
    Ndistance90 = [1.0 if distance > threshold_lidar else distance / threshold_lidar for distance in distances90]

    # Fill the gaps with 1.0
    Ndistance90 = [1.0 if distance == 0 else distance for distance in Ndistance90]

    return Ndistance90

def get_action(Ndistance90):
    # For more information check "https://netron.app/" select the model and check the inputs and outputs
    input_feed = {}
    input_feed["obs_0"] = [Ndistance90]

    results = session.run(["deterministic_continuous_actions"], input_feed)

    return results[0][0]

def step(action):
    # This function manage the physics of the car:
    # maxAngles and maxThrottle needs to be defined for the car to run properly

    if abs(action[1]) > 0.15: # Check if the throttle action is beyond the neutral point (0.15)
        throttle = action[1] * maxThrottle
        px.forward(throttle)
    else:
        px.stop()
        
    steering = action[0] * maxAngle
    px.set_dir_servo_angle(steering)

def reset():
    px.stop()
    px.set_dir_servo_angle(0)



reset()

if ser.isOpen():
    try:
        ser.flushInput()
        buffer = bytearray()
        while True:
            bytes_to_read = ser.inWaiting()

            if bytes_to_read > 0:
                buffer += ser.read(bytes_to_read)

                # We make sure that the buffer starts with the HEADER
                while buffer[0] != HEADER:
                    buffer = buffer[1:]

                angles_buffer = []
                distances_buffer = []

                # Read each frame of 47 bytes in the buffer that contains many frames
                while len(buffer) >= 47:
                    new_angles, new_distances = read_frame(buffer[:47])

                    angles_buffer += new_angles
                    distances_buffer += new_distances
                    buffer = buffer[47:]

                # The variables angles_buffer and distances_buffer contain the angles and distances of the lidar
                # We now need to reduce these vectors to a single vector of 90 elements

                distance90 = get_distance90(angles_buffer, distances_buffer)
 
                action = get_action(distance90)

                step(action)




                # Debug
                # print()
                # print(F"NEW BUFFER({len(angles_buffer)}): ")
                # for i in range(len(distances360)):
                #     print(f"Clear: {i} {distances360[i]}")
                
                # for i in range(0, len(distance90), 2):
                #     if i + 1 < len(distance90):
                #         print(f"Reduced: {i} {distance90[i]:<10} {i+1} {distance90[i+1]}")
                #     else:
                #         print(f"Reduced: {i} {distance90[i]}")

                print(f"Steering: {action[0]}, Throttle: {action[1]}")
                
                
            time.sleep(0.1)
    except KeyboardInterrupt:
        px.stop()
        px.set_dir_servo_angle(0)
        print("Stopped by User")
    finally:
        ser.close()
else:
    print("Could not open serial port")




