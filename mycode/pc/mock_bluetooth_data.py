#!/usr/bin/env python3
import random
import time
import requests
import math
from datetime import datetime

# InfluxDB configuration
INFLUX = "http://localhost:8086/api/v2/write?org=uq&bucket=prac3&precision=ns"
TOKEN = "supertoken"
HEADERS = {"Authorization": f"Token {TOKEN}"}

# Function to create InfluxDB line protocol format
def lp(meas, tags:dict, fields:dict, ts:int):
    tag_str = ",".join(f"{k}={v}" for k,v in tags.items())
    field_str = ",".join(f"{k}={v}" for k,v in fields.items())
    return f"{meas},{tag_str} {field_str} {ts}"

# Grid dimensions from the prac specifications
GRID_W, GRID_H = 4.0, 3.0

# Simulated iBeacons with fixed positions as mentioned in the prac spec
IBEACONS = [
    {"name": "ibeacon_A", "x": 0.0, "y": 0.0},
    {"name": "ibeacon_B", "x": GRID_W, "y": 0.0},
    {"name": "ibeacon_C", "x": 0.0, "y": GRID_H},
    {"name": "ibeacon_D", "x": GRID_W, "y": GRID_H},
    {"name": "ibeacon_E", "x": GRID_W/2, "y": 0.0},
    {"name": "ibeacon_F", "x": 0.0, "y": GRID_H/2},
    {"name": "ibeacon_G", "x": GRID_W, "y": GRID_H/2},
    {"name": "ibeacon_H", "x": GRID_W/2, "y": GRID_H}
]

# Initial position and velocity
x, y = 0.5, 0.5
vx, vy = 0.6, 0.1
dt = 0.05  # time step in seconds

# For ultrasonic ranging simulation
ultrasonic_nodes = [
    {"name": "us_node_1", "x": GRID_W/2, "y": GRID_H/2},
    {"name": "us_node_2", "x": GRID_W, "y": GRID_H/2}
]

def calculate_distance(x1, y1, x2, y2):
    """Calculate Euclidean distance between two points"""
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def simulate_rssi(distance):
    """Simulate RSSI based on distance with realistic noise"""
    # Simple path loss model: RSSI = -10*n*log10(d) + A
    # where n is path loss exponent, d is distance, A is reference RSSI at 1m
    n = 2.0  # Path loss exponent
    A = -60  # RSSI at 1m
    ideal_rssi = -10 * n * math.log10(max(distance, 0.1)) + A
    noise = random.gauss(0, 3)  # Add noise with 3 dBm standard deviation
    return round(ideal_rssi + noise)

def simulate_ultrasonic(distance):
    """Simulate ultrasonic range measurement with noise"""
    if distance > 3.0:  # Typical max ultrasonic range
        return None
    noise = random.gauss(0, 0.02)  # 2cm standard deviation
    return max(0.0, distance + noise)

def send_to_influxdb(payload):
    """Send data to InfluxDB with error handling and debugging"""
    try:
        response = requests.post(INFLUX, headers=HEADERS, data=payload.encode())
        if response.status_code != 204:
            print(f"Error sending data: {response.status_code} {response.text}")
            return False
        return True
    except Exception as e:
        print(f"Failed to send data: {e}")
        return False

def main():
    global x, y, vx, vy
    
    print("Starting mock Bluetooth data generator...")
    print(f"Sending data to InfluxDB at {INFLUX}")
    print("Press Ctrl+C to stop")
    
    # Setup iBeacon positions once at the beginning
    print("Setting up iBeacon positions...")
    ts = time.time_ns()
    ibeacon_packets = []
    for ibeacon in IBEACONS:
        ib_tags = {"name": ibeacon["name"]}  # Simplified tags
        ib_fields = {"x": ibeacon["x"], "y": ibeacon["y"]}
        ibeacon_packets.append(lp("ibeacon_positions", ib_tags, ib_fields, ts))
    
    # Send all iBeacon position packets
    ibeacon_payload = "\n".join(ibeacon_packets)
    if send_to_influxdb(ibeacon_payload):
        print("iBeacon positions set successfully!")
    else:
        print("Failed to set iBeacon positions!")
    
    # Test connection with a simple data point
    test_payload = lp("test_measurement", {"test": "connection"}, {"value": 1}, ts)
    if send_to_influxdb(test_payload):
        print("InfluxDB connection test successful!")
    else:
        print("InfluxDB connection test failed!")
    
    # For calculating distance traveled
    last_x, last_y = x, y
    total_distance = 0.0
    
    iteration = 0
    try:
        while True:
            iteration += 1
            ts = time.time_ns()
            
            # Calculate distance traveled in this step
            step_distance = calculate_distance(last_x, last_y, x, y)
            total_distance += step_distance
            last_x, last_y = x, y
            
            # Generate multilat packets (one for each visible iBeacon)
            packets = []
            for ibeacon in IBEACONS:
                distance = calculate_distance(x, y, ibeacon["x"], ibeacon["y"])
                if distance < 5.0:  # Assume beacons have 5m range
                    rssi = simulate_rssi(distance)
                    ml_tags = {"method": "multilat", "ibeacon": ibeacon["name"]}
                    # Add Gaussian noise to position estimate from multilateration
                    ml_fields = {
                        "x": x + random.gauss(0, 0.8),
                        "y": y + random.gauss(0, 0.8),
                        "rssi": rssi
                    }
                    packets.append(lp("position", ml_tags, ml_fields, ts))
            
            # Generate ultrasonic packets
            for us_node in ultrasonic_nodes:
                distance = calculate_distance(x, y, us_node["x"], us_node["y"])
                us_range = simulate_ultrasonic(distance)
                if us_range is not None:  # Only if in range
                    us_tags = {"method": "ultrasonic", "node": us_node["name"]}
                    us_fields = {"us_range": us_range}
                    packets.append(lp("position", us_tags, us_fields, ts))
            
            # Generate Kalman filter output packet (simulated fusion result)
            # In a real implementation, this would be the output of your Kalman filter
            k_tags = {"method": "kalman"}
            k_fields = {
                "x": x,
                "y": y,
                "vx": vx,
                "vy": vy
            }
            packets.append(lp("position", k_tags, k_fields, ts))
            
            # Send all packets
            payload = "\n".join(packets)
            if send_to_influxdb(payload):
                if iteration % 20 == 0:  # Only print every 20 iterations to avoid console spam
                    print(f"Sent data point #{iteration}: pos=({x:.2f},{y:.2f}), v=({vx:.2f},{vy:.2f})")
            
            # Update position for next iteration
            x += vx * dt
            y += vy * dt
            
            # Bounce off walls
            if x <= 0.1 or x >= GRID_W - 0.1:
                vx = -vx
                x = max(0.1, min(x, GRID_W - 0.1))
            if y <= 0.1 or y >= GRID_H - 0.1:
                vy = -vy
                y = max(0.1, min(y, GRID_H - 0.1))
                
            # Occasionally change direction slightly to make movement more natural
            if random.random() < 0.05:  # 5% chance each iteration
                vx += random.gauss(0, 0.05)
                vy += random.gauss(0, 0.05)
                
                # Keep velocity within reasonable bounds
                speed = math.sqrt(vx*vx + vy*vy)
                if speed > 1.0:  # Cap max speed
                    vx = vx / speed
                    vy = vy / speed
                elif speed < 0.2:  # Ensure minimum speed
                    if speed < 0.01:  # Prevent division by zero
                        vx, vy = 0.2, 0.0
                    else:
                        vx = 0.2 * vx / speed
                        vy = 0.2 * vy / speed
            
            time.sleep(dt)
            
    except KeyboardInterrupt:
        print("\nStopping data generator")

if __name__ == "__main__":
    main()