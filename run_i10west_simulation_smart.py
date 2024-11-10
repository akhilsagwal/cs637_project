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

# Load RSU locations from file
def load_rsus(filename):
    rsus = []
    with open(filename, 'r') as file:
        next(file)  # Skip header line
        for line in file:
            id, x, y = line.strip().split(',')
            rsus.append(RSU(int(id), float(x), float(y)))
    return rsus

# Predict future vehicle position based on speed and heading angle
def predict_future_position(vehicle_id, time_ahead=2):
    pos = traci.vehicle.getPosition(vehicle_id)
    speed = traci.vehicle.getSpeed(vehicle_id)
    angle = math.radians(traci.vehicle.getAngle(vehicle_id))
    future_x = pos[0] + speed * time_ahead * math.cos(angle)
    future_y = pos[1] + speed * time_ahead * math.sin(angle)
    return future_x, future_y

# Check for obstacles in the LOS between vehicle and RSU
def is_obstacle_in_los(vehicle_pos, rsu, vehicle_id):
    obstacles = traci.vehicle.getIDList()
    for obs_id in obstacles:
        if obs_id == vehicle_id:
            continue
        obs_pos = traci.vehicle.getPosition(obs_id)
        if (min(vehicle_pos[0], rsu.x) <= obs_pos[0] <= max(vehicle_pos[0], rsu.x) and
            min(vehicle_pos[1], rsu.y) <= obs_pos[1] <= max(vehicle_pos[1], rsu.y)):
            return True
    return False

# Calculate signal strength considering obstacles
def calculate_signal_strength(vehicle_pos, rsu, vehicle_id):
    distance = math.sqrt((vehicle_pos[0] - rsu.x) ** 2 + (vehicle_pos[1] - rsu.y) ** 2)
    if distance == 0:
        return 1.0
    if is_obstacle_in_los(vehicle_pos, rsu, vehicle_id):
        return max(0, 1 / (1 + distance * 5))
    return max(0, 1 / (1 + distance))

# Estimate connection duration based on current speed and RSU distance
def estimate_connection_duration(vehicle_id, rsu):
    vehicle_pos = traci.vehicle.getPosition(vehicle_id)
    vehicle_speed = traci.vehicle.getSpeed(vehicle_id)
    distance = math.sqrt((vehicle_pos[0] - rsu.x) ** 2 + (vehicle_pos[1] - rsu.y) ** 2)
    return distance / vehicle_speed if vehicle_speed != 0 else float('inf')

# Run Smart RSU selection simulation with logging
def run_simulation(config_file, rsus):
    sumo_process = subprocess.Popen(["sumo", "-c", config_file, "--remote-port", "8813"])
    time.sleep(2)

    with open('i10west_smart_simulation_log.csv', mode='w', newline='') as log_file:
        log_writer = csv.writer(log_file)
        log_writer.writerow(["Step", "Vehicle_ID", "RSU_ID", "Signal_Strength", "Connection_Duration", "Score"])

        try:
            traci.init(8813)
            for step in range(100):
                traci.simulationStep()
                vehicle_ids = traci.vehicle.getIDList()

                for vehicle_id in vehicle_ids:
                    predicted_pos = predict_future_position(vehicle_id)
                    best_rsu = None
                    best_score = 0

                    for rsu in rsus:
                        signal_strength = calculate_signal_strength(predicted_pos, rsu, vehicle_id)
                        connection_duration = estimate_connection_duration(vehicle_id, rsu)
                        score = signal_strength * connection_duration
                        if score > best_score:
                            best_score = score
                            best_rsu = rsu

                    if best_rsu:
                        log_writer.writerow([step, vehicle_id, best_rsu.id, signal_strength, connection_duration, best_score])
                        print(f"Vehicle {vehicle_id} connected to RSU {best_rsu.id} with score {best_score}")

        except Exception as e:
            print(f"Error: {e}")

        finally:
            traci.close()
            if sumo_process.poll() is None:
                sumo_process.terminate()
                try:
                    sumo_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    sumo_process.kill()
            sys.exit()

rsus_i10west = load_rsus("I10West_junctions.txt")
run_simulation("I10West.sumocfg", rsus_i10west)