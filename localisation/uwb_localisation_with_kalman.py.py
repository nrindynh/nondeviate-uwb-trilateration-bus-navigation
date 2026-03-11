"""
This update removes A3 as the messenger, uses separate ESP as messenger. 
Receives input bus request from ESP & sends back to the ESP for processing.
"""


import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from functools import partial
import select

import serial
from time import sleep, time
import math
from collections import deque
import numpy as np

import socket 
import sys 

# Variables to be modified:
#################################################
# Range for plot
plot_x_lim = [-2000, 500]
plot_y_lim = [-1000, 500]

# List of available buses
bus_stop_buses = [2, 5 ,12, 24] 
# Continuously updated list of requested buses, contains string
bus_request = [] 
# The bus position list inserts buses to their position based on their order
bus_position_list = ["0", "0", "0", "0", "0", "0", "0", "0", "0", "0"]
cm_to_pixel = 1

# Define Zones (in cm)
zone_defs = {
    "Front":   {'x_min': -350, 'x_max': 350,  'y_min': -500, 'y_max': 200},
    "Back": {'x_min': -1050, 'x_max': -350, 'y_min': -500, 'y_max': 200},
    #"Back":  {'x_min': 350, 'x_max': 450, 'y_min': -500, 'y_max': 1000}
}

zone_num_dict = {"Front": 1,
                 #"Middle":2
                 "Back": 2
                }
# Anchor positions in cm
anchors_cm = {'A1': (-350, 200), 'A2': (-600, 200), 'A3': (-600, 150)}

# Setting connection to ESP as False to prevent error
esp_connection = False
#################################################

# Serial setup for single ESP
ser = serial.Serial(port='COM5', baudrate=115200, timeout=0)

# Reading messages from UWB from Port sent through Wifi
try:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
except socket.error as msg:
    print("Failed to create socket. Error Code : " + str(msg[0]) + " Message " + msg[1])
    sys.exit()

try:
    s.bind(("", 3333))
except socket.error as msg:
    print("Bind failed. Error: " + str(msg[0]) + ": " + msg[1])
    sys.exit()
print("Server listening")


# Kalman Filter Class used in the Tag class to improve localisation of each tag
class KalmanFilter2D:
    def __init__(self, process_variance=1e-4, measurement_variance=150):
        """
        process_variance: How much we expect the system to change between measurements
        measurement_variance: How noisy we expect our measurements to be
        """
        self.x = np.zeros((4, 1))  # State: [x, y, vx, vy] (position and velocity)
        self.P = np.eye(4) * 500   # Initial uncertainty
        
        # State transition matrix (assumes constant velocity model)
        self.F = np.array([[1, 0, 1, 0],  # x = x + vx*dt (dt=1)
                           [0, 1, 0, 1],  # y = y + vy*dt
                           [0, 0, 1, 0],  # vx = vx (constant velocity)
                           [0, 0, 0, 1]]) # vy = vy
        
        # Observation matrix (we only observe position, not velocity)
        self.H = np.array([[1, 0, 0, 0],  # observe x
                           [0, 1, 0, 0]]) # observe y
        
        # Measurement noise covariance
        self.R = np.eye(2) * measurement_variance
        
        # Process noise covariance
        self.Q = np.eye(4) * process_variance
        
        self.initialized = False

    def predict(self):
        """Predict the next state"""
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, z):
        """Update with new measurement z = [x, y]"""
        if not self.initialized:
            # Initialize filter with first measurement
            self.x[0, 0] = z[0]  # x position
            self.x[1, 0] = z[1]  # y position
            self.x[2, 0] = 0     # x velocity
            self.x[3, 0] = 0     # y velocity
            self.initialized = True
            return
            
        z = np.array(z).reshape(2, 1)
        
        # Innovation (measurement residual)
        y = z - self.H @ self.x
        
        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R
        
        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # Update state estimate
        self.x = self.x + K @ y
        
        # Update error covariance
        self.P = (np.eye(self.F.shape[0]) - K @ self.H) @ self.P

    def get_position(self):
        """Get current filtered position estimate"""
        return self.x[0, 0], self.x[1, 0]
    
    def get_velocity(self):
        """Get current velocity estimate"""
        return self.x[2, 0], self.x[3, 0]


# Tag manager class creates Tag if it does not exist
class Tag:
    def __init__(self, tag_id, anchors, history_len=10, timeout=10.0):
        """
        self.id is the bus number
        self.anchor_data is a dictionary of each anchor's latest recording of the tag
        self.history is the history list of the distance of the tag with relation to the anchor
        """
        self.id = tag_id
        self.anchor_data = {anchor_id: {'distance': 0.0, 'timestamp': 0.0, 'valid': False} for anchor_id in anchors}
        self.history = {anchor_id: deque(maxlen=history_len) for anchor_id in anchors}

        self.last_seen = time()
        self.timeout = timeout
        self.visuals = {'dot': None, 'filtered_dot': None, 'label': tag_id, 'lines': []}
        
        # Add Kalman filter for this tag
        self.kalman_filter = KalmanFilter2D(process_variance=1e-4, measurement_variance=100) 
        
        # Store raw and filtered positions for comparison
        self.raw_position = None
        self.filtered_position = None

    def update(self, anchor_id, distance):
        now = time()
        self.last_seen = now
        self.history[anchor_id].append(distance)
        # Making a list from deque of distances
        dlist = list(self.history[anchor_id])
        # Implement some sort of filtering later on
        self.anchor_data[anchor_id] = {'distance': distance, 'timestamp': now, 'valid': True}

    def is_active(self):
        # Return boolean, True if active
        return (time() - self.last_seen) <= self.timeout

    def get_distances(self):
        """Return distances in consistent order: A1, A2, A3"""
        distances = {}
        
        # Collect valid distances
        for anchor_id, data in self.anchor_data.items():
            if data['valid'] and self.is_active():
                distances[anchor_id] = data['distance']
        
        # Return in consistent order A1, A2, A3
        if all(anchor in distances for anchor in ['A1', 'A2', 'A3']):
            return (distances['A1'], distances['A2'], distances['A3'])
        
        return None
    
    def update_position(self, raw_x, raw_y):
        """Update both raw and filtered positions"""
        self.raw_position = (raw_x, raw_y)
        
        # Predict next state
        self.kalman_filter.predict()
        
        # Update with new measurement
        self.kalman_filter.update([raw_x, raw_y])
        
        # Get filtered position
        self.filtered_position = self.kalman_filter.get_position()


# Creates Tags and handles messages from Wifi
class TagManager:
    # Updated handle message to accept bus request from users, will save them in a list
    def __init__(self, anchors):
        # anchors is a list of anchors
        self.anchors = anchors
        self.tags = {}

    def handle_message(self, msg):
        # Separates msg, up
        # print("[Received UDP]", msg)  # Expected format: "A1:TagID:222A:50"

        parts = msg.strip().split(':')  # Separate message by Anchor ID, Tag ID, dist
        print(parts)
        if len(parts) == 3:
            anchor_id, tag_id, dist_with_unit = parts
            
            try:
                distance = float(dist_with_unit.replace('m', ''))  # remove m
                print(f"Anchor {anchor_id}, Tag {tag_id}, Distance {distance}m")
            except ValueError:
                print("Could not parse distance:", dist_with_unit)
                return    
            # Creates/updates tag object
            if tag_id not in self.tags:
                self.tags[tag_id] = Tag(tag_id, self.anchors)
            self.tags[tag_id].update(anchor_id, distance)
            # Check if bus requested is in bus_request list, sends back through Wifi / USB-UART 
        '''
        if len(parts) == 5:
            # PLEASE CHECK THIS AGAIN, HOW IS IT SENT OVER
            # If message sent from ESP32 is user request, size will be longer
            # Save wanted bus to list
            username, bus_stop_id, bus_collection, arr_bus, selected_bus_numbers = parts 
            for bus in selected_bus_numbers:
                if bus not in bus_request:
                    bus_request.append(bus_stop)
            return
        '''

    def cleanup(self):
        for tag in list(self.tags):
            if not self.tags[tag].is_active():
                print(f"Removing inactive tag: {tag}")
                del self.tags[tag]

    def get_active_tags(self):
        return list(self.tags.values())
 

# Trilateration function takes in coordinate and distance input
def trilaterate_3anchors(p1, p2, p3, r1, r2, r3):
    """
    p1, p2, p3 are the position values of anchor 1, 2, 3 (A1, A2, A3)
    r1, r2, r3 is the distance values of the tag from anchor 1, 2, 3 (A1, A2, A3)
    """
    try:
        x1, y1 = p1
        x2, y2 = p2
        x3, y3 = p3
        
        # Convert distances from meters to cm to match anchor coordinates
        r1_cm = r1 * 100
        r2_cm = r2 * 100
        r3_cm = r3 * 100
        
        A = 2 * (x2 - x1)
        B = 2 * (y2 - y1)
        C = r1_cm**2 - r2_cm**2 - x1**2 + x2**2 - y1**2 + y2**2
        D = 2 * (x3 - x2)
        E = 2 * (y3 - y2)
        F = r2_cm**2 - r3_cm**2 - x2**2 + x3**2 - y2**2 + y3**2
        
        denominator = A * E - B * D
        if abs(denominator) < 1e-6:
            print("Trilateration: Anchors are collinear, cannot solve")
            return None, None
            
        x = (C * E - B * F) / denominator
        y = (A * F - C * D) / denominator
        
        # print(f"Trilateration result: x={x:.1f}cm, y={y:.1f}cm (distances: A1={r1:.2f}m, A2={r2:.2f}m, A3={r3:.2f}m)")
        return x, y
    except Exception as e:
        print("Trilateration error:", e)
        return None, None


# Bus Stop Zone Classification Function
def classify_zone(x, y, zone_defs):
    for zone_name, zone in zone_defs.items():
        if (zone['x_min'] <= x <= zone['x_max'] and 
            zone['y_min'] <= y <= zone['y_max']):
            return zone_name
    return "Out of Bounds"


# Clear Bus List
def clear_bus_list():
    bus_request.clear()


# Update bus request list from the msg list
def update_bus_request(msg_list):
    for bus in msg_list:
        if bus != "0" and bus != "entry 0x400805b4":
            bus_request.append(bus)

# Setup Matplotlib
fig, ax = plt.subplots(figsize=(12, 8))
plt.title("Real-time UWB Localization with Kalman Filter")
ax.set_xlim(plot_x_lim[0], plot_x_lim[1])
ax.set_ylim(plot_y_lim[0], plot_y_lim[1])
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)
ax.set_xlabel('X Position (cm)')
ax.set_ylabel('Y Position (cm)')

# Bus Stop Dimensions (in cm, converted to pixels for display)
bus_stop_x = 0 * cm_to_pixel
bus_stop_y = 50 * cm_to_pixel
bus_stop_length = -700 * cm_to_pixel  # 5 meters
bus_stop_width = 150 * cm_to_pixel   # 1 meter

# Draw Bus Stop
bus_stop_rect = patches.Rectangle(
    (bus_stop_x, bus_stop_y), 
    bus_stop_length, 
    bus_stop_width, 
    facecolor='lightgray',
    alpha=0.8,
    zorder=1
)
ax.add_patch(bus_stop_rect)


# Create Zone Rectangles
zone_rects = {}
for zone_name, zone in zone_defs.items():
    rect = patches.Rectangle(
        (zone['x_min'] * cm_to_pixel, zone['y_min'] * cm_to_pixel),
        (zone['x_max'] - zone['x_min']) * cm_to_pixel,
        (zone['y_max'] - zone['y_min']) * cm_to_pixel,
        linewidth=1, edgecolor='yellow', facecolor='turquoise', alpha=0.5,
        zorder=2 
    )
    ax.add_patch(rect)
    zone_rects[zone_name] = rect

# Add legend
# raw_line = plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='orange', markersize=8, alpha=0.6, label='Raw Position')
filtered_line = plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='red', markersize=10, label='Bus Position')
anchor_line = plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='lightblue', markersize=8, label='Anchors')
ax.legend(handles=[filtered_line, anchor_line], loc='upper right')

# Defining anchor position and diagram marker
anchor_patches = {}
anchor_texts = {}

# Creating anchors
anchors = {anchor_id: (x * cm_to_pixel, y * cm_to_pixel) for anchor_id, (x, y) in anchors_cm.items()}

for anchor_id, (x, y) in anchors.items():
    patch = patches.Circle((x, y), radius=5, edgecolor='blue', facecolor='lightblue', linewidth=1, zorder=5)
    ax.add_patch(patch)
    txt = ax.text(x, y + 20, f"{anchor_id}\n({x},{y})", color='blue', fontweight='bold',zorder=6)
    anchor_patches[anchor_id] = patch
    anchor_texts[anchor_id] = txt

manager = TagManager(['A1', 'A2', 'A3'])

def update_plot(frame, manager, esp_connection):
    # This boolean decides if the message should be sent, always set as False first
    should_send = False
    bus_coordinates_list = {}
    bus_position_list = ["0", "0", "0", "0", "0", "0", "0", "0", "0", "0"]

    # Read ESP data from Wifi, split into msg and addr, should not use addr
    ready = select.select([s], [], [], 0.01)  # 10ms timeout
    if ready[0]:
        data, addr = s.recvfrom(1024)
        msg = data.strip().decode()
        manager.handle_message(msg)
    else:
        # No new data, just update existing tags
        pass
        
    # Read ESP data from UDP socket, will just be bus request, send message of user:bus_number 
    # NEED CHANGE THIS TO FIT THE FORMAT
    if ser.in_waiting > 0:
        msg = ser.readline().decode().strip()
        print("msg is", msg)
        user, bus = msg.split(":")
        #selected_bus_list = msg.split(":")

        # If there is message from serial output, first clear bus list
        clear_bus_list()

        # Add in the selected buses again
        update_bus_request(bus)

        if esp_connection == False:
            esp_connection = True

        print(bus_request)
    else:
        # No new data, just update existing tags
        pass

    # Removes inactive tags
    manager.cleanup()
    artists = list(anchor_patches.values()) + list(anchor_texts.values())


    print("bus request is", bus_request)

    # Obtain distances for each tag
    for tag in manager.get_active_tags():
        distances = tag.get_distances()
        if not distances:
            continue
        
        r1, r2, r3 = distances  # A1, A2, A3 distances
        
        # Get raw trilateration result
        raw_x, raw_y = trilaterate_3anchors(
            anchors_cm['A1'],  # 
            anchors_cm['A2'],  # 
            anchors_cm['A3'],  # 
            r1, r2, r3         # distances to A1, A2, A3
        )
        
        if raw_x is None or raw_y is None:
            continue

        # Update tag position with Kalman filtering
        tag.update_position(raw_x, raw_y)
        
        # Get filtered position
        filtered_x, filtered_y = tag.filtered_position
        bus_coordinates_list[tag] = (filtered_x, filtered_y)

        # Classify zone based on filtered position
        current_zone = classify_zone(filtered_x, filtered_y, zone_defs)
        print(f"{tag.id} is in {current_zone} Zone.")

        # Check request list, updates if bus number requested
        if tag.id in bus_request:
            should_send = True
            print(f"{tag.id} in bus_request")
            # Send to ESP using serial write, reply is a string 
    
        else:
            print(f"{tag.id} not in bus_request")

        # Update zone highlighting
        for zone_name, rect in zone_rects.items():
            if zone_name == current_zone and current_zone != "Out of Bounds":
                rect.set_facecolor('lightyellow')
                rect.set_alpha(0.4)
            else:
                rect.set_facecolor('none')
                rect.set_edgecolor('yellow')
                rect.set_alpha(0.7)

        # Create visuals if they don't exist
        if tag.visuals['dot'] is None:
            # # Raw position (orange, semi-transparent)
            # tag.visuals['dot'] = ax.plot([raw_x], [raw_y], 'o', color='orange', markersize=8, alpha=0.6)[0]
            
            # Filtered position (red, solid)
            tag.visuals['filtered_dot'] = ax.plot([filtered_x], [filtered_y], 'ro', markersize=10, zorder=4)[0]
            
            # Label shows filtered position
            tag.visuals['label'] = ax.text(filtered_x + 15, filtered_y + 15, f"{tag.id}", color='red', fontweight='bold', zorder=4)
            
            # Lines connect to filtered position
            tag.visuals['lines'] = [ax.plot([], [], 'r--', alpha=0.6, zorder=3)[0] for _ in range(3)]
        else:
            # Update raw position
            tag.visuals['dot'].set_data([raw_x], [raw_y])
            
            # Update filtered position
            tag.visuals['filtered_dot'].set_data([filtered_x], [filtered_y])
            
            # Update label with filtered position
            tag.visuals['label'].set_position((filtered_x + 15, filtered_y + 15))
            tag.visuals['label'].set_text(f"{tag.id}\nRaw: ({int(raw_x)},{int(raw_y)})\nFiltered: ({int(filtered_x)},{int(filtered_y)})")

        # Update lines to anchors (use filtered position)
        for i, anchor_id in enumerate(['A1', 'A2', 'A3']):
            ax_pos, ay_pos = anchors[anchor_id]
            tag.visuals['lines'][i].set_data([ax_pos, filtered_x], [ay_pos, filtered_y])

        # Add all visual elements to artists list
        # artists.append(tag.visuals['dot'])        # Raw position
        artists.append(tag.visuals['filtered_dot']) # Filtered position
        artists.append(tag.visuals['label'])      # Label
        artists.extend(tag.visuals['lines'])      # Lines to anchors
        artists.extend(zone_rects.values())
        artists.append(bus_stop_rect)

        if should_send:
            reply_message: string = ""
            # bus is string
            for bus in bus_position_list:
                reply_message += (bus + ",")
            reply_message +="\n"
            ser.write(reply_message.encode())
    return artists

ani = FuncAnimation(fig, partial(update_plot, manager=manager, esp_connection=esp_connection), interval=50, blit=True, cache_frame_data=False)
plt.show()