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

# Load RSU locations from file
def load_rsus(file_path):
    rsus = []
    with open(file_path, 'r') as file:
        next(file)  # Skip header
        for line in file:
            id, x, y = line.strip().split(',')
            rsus.append(RSU(int(id), float(x), float(y)))
    return rsus

# Predict future position based on speed and angle
def predict_future_position(vehicle_pos, vehicle_speed, vehicle_angle, time_ahead=2):
    delta_x = vehicle_speed * time_ahead * math.cos(math.radians(vehicle_angle))
    delta_y = vehicle_speed * time_ahead * math.sin(math.radians(vehicle_angle))
    return (vehicle_pos[0] + delta_x, vehicle_pos[1] + delta_y)

# Function to estimate connection duration based on predicted positions
def estimate_connection_duration(vehicle_id, rsu, max_duration=10, step_size=1):
    duration = 0
    for t in range(1, max_duration + 1, step_size):
        future_pos = predict_future_position(traci.vehicle.getPosition(vehicle_id), traci.vehicle.getSpeed(vehicle_id), traci.vehicle.getAngle(vehicle_id), time_ahead=t)
        distance = math.sqrt((future_pos[0] - rsu.x) ** 2 + (future_pos[1] - rsu.y) ** 2)
        if distance > 400:  # Disconnect if future position is out of range
            break
        duration += step_size
    return duration

# Check for obstacles in the LOS between the vehicle and RSU
def check_for_obstacles(vehicle_pos, rsu):
    obstacles = traci.vehicle.getIDList()  # Other vehicles as obstacles
    for obstacle_id in obstacles:
        obstacle_pos = traci.vehicle.getPosition(obstacle_id)
        if (min(vehicle_pos[0], rsu.x) <= obstacle_pos[0] <= max(vehicle_pos[0], rsu.x) and
            min(vehicle_pos[1], rsu.y) <= obstacle_pos[1] <= max(vehicle_pos[1], rsu.y)):
            return True  # Obstacle found in LOS
    return False  # No obstacles in LOS

# Calculate signal strength considering obstacles
def calculate_signal_strength(predicted_pos, rsu):
    distance = math.sqrt((predicted_pos[0] - rsu.x) ** 2 + (predicted_pos[1] - rsu.y) ** 2)
    if distance == 0:
        return 1.0

    # Check for obstacles in LOS and adjust signal strength
    is_blocked = check_for_obstacles(predicted_pos, rsu)
    if is_blocked:
        return max(0, 1 / (1 + distance * 5))  # Significant reduction in signal if blocked
    else:
        return max(0, 1 / (1 + distance))

# Run the Smart RSU Selection Simulation with trajectory consideration and obstacle handling
def run_smart_simulation_with_trajectory(config_file, rsus):
    sumo_process = subprocess.Popen(["sumo-gui", "-c", config_file, "--remote-port", "8813"])
    time.sleep(2)

    # Open a CSV file for logging
    with open('smart_simulation_log.csv', mode='w', newline='') as log_file:
        log_writer = csv.writer(log_file)
        log_writer.writerow(["Step", "Vehicle_ID", "RSU_ID", "Signal_Strength", "Expected_Connection_Time", "Score"])

        try:
            traci.init(8813)
            print(f"Running Smart selection simulation with trajectory and obstacle handling for configuration: {config_file}")

            for step in range(500):  # Adjusted steps to 500
                traci.simulationStep()
                vehicle_ids = traci.vehicle.getIDList()

                if not vehicle_ids:
                    print("All vehicles have reached their destinations.")
                    break

                for vehicle_id in vehicle_ids:
                    vehicle_pos = traci.vehicle.getPosition(vehicle_id)
                    vehicle_speed = traci.vehicle.getSpeed(vehicle_id)
                    vehicle_angle = traci.vehicle.getAngle(vehicle_id)

                    # Predict future position
                    predicted_pos = predict_future_position(vehicle_pos, vehicle_speed, vehicle_angle)

                    best_rsu = None
                    best_score = 0

                    # Evaluate each RSU for optimal connection based on predicted position and connection duration
                    for rsu in rsus:
                        signal_strength = calculate_signal_strength(predicted_pos, rsu)
                        connection_duration = estimate_connection_duration(vehicle_id, rsu)
                        score = signal_strength * connection_duration  # Maximize both signal strength and duration

                        if score > best_score:
                            best_score = score
                            best_rsu = rsu

                    # Log the connection and signal strength data
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

# Paths for RSU files
rsu_file_chandler = "ChandlerAZ_junctions.txt"
rsu_file_i10west = "I10West_junctions.txt"

# Run the Smart RSU selection simulation with trajectory and obstacle handling for both configurations
rsus_chandler = load_rsus(rsu_file_chandler)
print("Running Smart selection simulation with trajectory and obstacle handling for Chandler, AZ...")
run_smart_simulation_with_trajectory("ChandlerAZ.sumocfg", rsus_chandler)

rsus_i10 = load_rsus(rsu_file_i10west)
print("Running Smart selection simulation with trajectory and obstacle handling for I-10 West...")
run_smart_simulation_with_trajectory("I10West.sumocfg", rsus_i10)
