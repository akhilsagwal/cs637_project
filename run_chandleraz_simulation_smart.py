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

# Check if an obstacle will block LOS for a given future position
def is_blocking_los(future_vehicle_pos, rsu, time_ahead):
    obstacles = traci.vehicle.getIDList()
    for obstacle_id in obstacles:
        obstacle_pos = predict_future_position(obstacle_id, time_ahead)
        if (min(future_vehicle_pos[0], rsu.x) <= obstacle_pos[0] <= max(future_vehicle_pos[0], rsu.x) and
            min(future_vehicle_pos[1], rsu.y) <= obstacle_pos[1] <= max(future_vehicle_pos[1], rsu.y)):
            return True  # Obstacle found in LOS
    return False  # No obstacles in LOS

# Calculate signal strength considering obstacles
def calculate_signal_strength(vehicle_pos, rsu):
    distance = math.sqrt((vehicle_pos[0] - rsu.x) ** 2 + (vehicle_pos[1] - rsu.y) ** 2)
    if distance == 0:
        return 1.0
    return max(0, 1 / (1 + distance))

# Estimate connection duration based on future positions and obstacles
def estimate_connection_duration(vehicle_id, rsu, max_duration=10, step_size=1):
    duration = 0
    for t in range(1, max_duration + 1, step_size):
        future_pos = predict_future_position(vehicle_id, time_ahead=t)
        distance = math.sqrt((future_pos[0] - rsu.x) ** 2 + (future_pos[1] - rsu.y) ** 2)
        
        # Check if out of range or if LOS is blocked by obstacles at this future position
        if distance > 400 or is_blocking_los(future_pos, rsu, t):
            break  # Stop counting duration if future position is blocked or out of range
        
        duration += step_size
    return duration

# Run Smart RSU selection simulation with logging
def run_simulation(config_file, rsus):
    sumo_process = subprocess.Popen(["sumo-gui", "-c", config_file, "--remote-port", "8813"])
    time.sleep(2)

    with open('chandleraz_smart_simulation_log.csv', mode='w', newline='') as log_file:
        log_writer = csv.writer(log_file)
        log_writer.writerow(["Step", "Vehicle_ID", "RSU_ID", "Signal_Strength", "Expected_Connection_Time", "Score"])

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
                        signal_strength = calculate_signal_strength(predicted_pos, rsu)
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

rsus_chandler = load_rsus("ChandlerAZ_junctions.txt")
run_simulation("ChandlerAZ.sumocfg", rsus_chandler)
