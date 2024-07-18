import serial
import serial.tools.list_ports
import time
import numpy as np
import matplotlib.pyplot as plt

# This script reads the lidar data and plots it in a polar graph
# The way the data is procesesed NEEDS to be the same as in the run_model.py script
# If there are not the same the data shown in the graph will be different from the data used to make the predictions in the model

# The script will save the graph periodically

HEADER = 0x54
POINT_PER_PACK = 12


lidar_port = '/dev/ttyUSB0'
# This value NEEDS to be the same as the threshold used in the model
threshold_lidar = 1000 # mm


def extract_data(frame):
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

    distances = []
    angles = []
    for i in range(POINT_PER_PACK):
        index = 6 + i*3
        distances.append(int.from_bytes(frame[index:index+2], 'little'))  # Distance in mm
        angles.append((start_angle + i*step)%360)

    # timestamp => frame[44:47]
    # CRC_Check => frame[46]

    # Debug
    # for angle, distance in zip(angles, distances):
    #     # if int(angle) == 60:
    #     print(f"{angle:.2f}\t{distance} cm")

    return (angles, distances)

anglesGraph = np.linspace(0, 2 * np.pi, 90, endpoint=False)
def plot_and_save_map(data_points):
    distances = np.array(data_points)
    
    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    
    # Tracer des lignes droites du centre vers chaque point
    for angle, distance in zip(anglesGraph, distances):
        ax.plot([angle, angle], [0, distance], color='blue')
    
    # Tracer des points pour chaque distance mesurée
    ax.scatter(anglesGraph, distances, c=distances, cmap='viridis', marker='o', s=10)
    
    ax.grid(True)
    
    print("image saved")
    plt.savefig("graph.png")
    plt.close(fig)


ser = serial.Serial(
    port=lidar_port,
    baudrate=230400,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)


if ser.isOpen():
    try:
        ser.flushInput()
        start_time = time.time()
        buffer = bytearray()
        while time.time() - start_time < 200:
            bytes_to_read = ser.inWaiting()

            if bytes_to_read > 0:
                buffer += ser.read(bytes_to_read)

                while buffer[0] != HEADER:
                    buffer = buffer[1:]

                angles_buffer = []
                distances_buffer = []
                while len(buffer) >= 47:
                    new_angles, new_distances = extract_data(buffer[:47])

                    angles_buffer += new_angles
                    distances_buffer += new_distances
                    buffer = buffer[47:]

                counts = np.zeros(90)
                distances90 = [0]*90
                for angle, distance in zip(angles_buffer, distances_buffer):
                    if distance == 0:
                        continue
                    indice = int(((angle+0.5)/4)%90) # NEEDS to be the same as in the run_model.py script (at least the same way of reducing the data)
                    distances90[indice] += distance
                    counts[indice] += 1
                for i in range(len(distances90)):
                    if counts[i] == 0:
                        continue
                    distances90[i] = distances90[i] / counts[i]

                Ndistance90 = [1.0 if distance > threshold_lidar else distance / threshold_lidar for distance in distances90]

                Ndistance90 = [1.0 if distance == 0 else distance for distance in Ndistance90]

                # Debug
                # print()
                # print(F"BUFFER({len(angles_buffer)}): ")                
                # for i in range(0, len(Ndistance90), 2):
                #     if i + 1 < len(Ndistance90):
                #         print(f"{i} {Ndistance90[i]:<10} {i+1} {Ndistance90[i+1]}")
                #     else:
                #         print(f"{i} {Ndistance90[i]}")


                if len(angles_buffer) > 500:
                    plot_and_save_map(Ndistance90)
                
                
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("Stopped by User")
    finally:
        ser.close()
else:
    print("Could not open serial port")

