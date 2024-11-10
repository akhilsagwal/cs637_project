import traci
import math
import subprocess
import sys
import time
import csv

# Class to represent an RSU
class RSU:
    def __init__(self, id, x, y):
        self.id = id
        self.x = x
        self.y = y
        self.connected_vehicles = []

# Load RSUs from text file
def load_rsus(file_path):
    rsus = []
    with open(file_path, 'r') as file:
        next(file)  # Skip header
        for line in file:
            id, x, y = line.strip().split(',')
            rsus.append(RSU(int(id), float(x), float(y)))
    return rsus

# Function to predict future vehicle position based on speed and heading angle
def predict_future_position(vehicle_id, time_ahead=2):
    pos = traci.vehicle.getPosition(vehicle_id)
    speed = traci.vehicle.getSpeed(vehicle_id)
    angle = math.radians(traci.vehicle.getAngle(vehicle_id))
    future_x = pos[0] + speed * time_ahead * math.cos(angle)
    future_y = pos[1] + speed * time_ahead * math.sin(angle)
    return future_x, future_y

# Function to check if an obstacle is blocking line-of-sight
def is_blocking_los(vehicle_pos, rsu, obstacle_pos):
    return math.dist(vehicle_pos, obstacle_pos) < math.dist(vehicle_pos, (rsu.x, rsu.y))

# Function to calculate signal strength with blockage awareness
def calculate_signal_strength(vehicle_pos, rsu, obstacles):
    distance = math.sqrt((vehicle_pos[0] - rsu.x) ** 2 + (vehicle_pos[1] - rsu.y) ** 2)
    for obstacle in obstacles:
        if is_blocking_los(vehicle_pos, rsu, obstacle):
            distance *= 1.5  # Increase effective distance if blocked
    return max(0, 1 / (1 + distance)) if distance > 0 else 1.0

# Estimate connection duration based on predicted positions
def estimate_connection_duration(vehicle_id, rsu, max_duration=10, step_size=1):
    duration = 0
    for t in range(1, max_duration + 1, step_size):
        future_pos = predict_future_position(vehicle_id, time_ahead=t)
        distance = math.sqrt((future_pos[0] - rsu.x) ** 2 + (future_pos[1] - rsu.y) ** 2)
        if distance > 400:  # Disconnect if future position is out of range
            break
        duration += step_size
    return duration

# Run B-AWARE simulation with logging
def run_b_aware_simulation(config_file, rsu_file):
    rsus = load_rsus(rsu_file)
    sumo_process = subprocess.Popen(["sumo-gui", "-c", config_file, "--remote-port", "8813"])
    time.sleep(2)

    with open('b_aware_simulation_log.csv', mode='w', newline='') as log_file:
        log_writer = csv.writer(log_file)
        log_writer.writerow(["Step", "Vehicle_ID", "RSU_ID", "Signal_Strength", "Expected_Connection_Time", "Score"])

        try:
            traci.init(8813)
            print(f"Running B-AWARE simulation for configuration: {config_file}")

            for step in range(500):  # Adjusted to 500 steps
                traci.simulationStep()
                vehicle_ids = traci.vehicle.getIDList()

                if not vehicle_ids:
                    print("All vehicles have reached their destinations.")
                    break

                for vehicle_id in vehicle_ids:
                    vehicle_pos = traci.vehicle.getPosition(vehicle_id)
                    nearby_obstacles = [traci.vehicle.getPosition(v) for v in traci.vehicle.getIDList() if v != vehicle_id]

                    best_rsu, best_score = None, 0

                    for rsu in rsus:
                        signal_strength = calculate_signal_strength(vehicle_pos, rsu, nearby_obstacles)
                        connection_duration = estimate_connection_duration(vehicle_id, rsu)
                        score = signal_strength * connection_duration  # Combined metric for selection

                        if score > best_score:
                            best_score, best_rsu = score, rsu

                    if best_rsu:
                        log_writer.writerow([step, vehicle_id, best_rsu.id, signal_strength, connection_duration, best_score])
                        print(f"Vehicle {vehicle_id} connected to RSU {best_rsu.id} with score {best_score}")

        except Exception as e:
            print(f"An error occurred: {e}")

        finally:
            traci.close()
            if sumo_process.poll() is None:
                sumo_process.terminate()
                try:
                    sumo_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    sumo_process.kill()
            print("SUMO process terminated.")
            sys.exit()

# Running B-AWARE simulation for I-10 West and Chandler AZ
print("Running B-AWARE simulation for I-10 West...")
run_b_aware_simulation("I10West.sumocfg", "I10West_junctions.txt")

# Uncomment for Chandler AZ
# print("Running B-AWARE simulation for Chandler, AZ...")
# run_b_aware_simulation("ChandlerAZ.sumocfg", "ChandlerAZ_junctions.txt")
