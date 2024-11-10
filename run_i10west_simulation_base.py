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

# Load RSU locations from text file for I-10 West
def load_rsus(file_path):
    rsus = []
    with open(file_path, 'r') as file:
        next(file)
        for line in file:
            id, x, y = line.strip().split(',')
            rsus.append(RSU(int(id), float(x), float(y)))
    return rsus

# Check for obstacles in the LOS between vehicle and RSU
def check_for_obstacles(vehicle_pos, rsu):
    obstacles = traci.vehicle.getIDList()  # Get list of other vehicles as potential obstacles
    for obstacle_id in obstacles:
        obstacle_pos = traci.vehicle.getPosition(obstacle_id)
        if (min(vehicle_pos[0], rsu.x) <= obstacle_pos[0] <= max(vehicle_pos[0], rsu.x) and
            min(vehicle_pos[1], rsu.y) <= obstacle_pos[1] <= max(vehicle_pos[1], rsu.y)):
            return True  # Obstacle found in LOS
    return False  # No obstacles in LOS

# Function to calculate signal strength, considering obstacles
def calculate_signal_strength(vehicle_pos, rsu):
    distance = math.sqrt((vehicle_pos[0] - rsu.x) ** 2 + (vehicle_pos[1] - rsu.y) ** 2)
    if distance == 0:
        return 1.0

    # Check for obstacles in LOS and adjust signal strength
    is_blocked = check_for_obstacles(vehicle_pos, rsu)
    if is_blocked:
        return max(0, 1 / (1 + distance * 5))  # Significant reduction in signal if blocked
    else:
        return max(0, 1 / (1 + distance))

# Function to run the baseline simulation with logging
def run_simulation(config_file, rsus):
    sumo_process = subprocess.Popen(
        ["sumo", "-c", config_file, "--remote-port", "8813"]
    )
    time.sleep(2)

    # Open a CSV file for logging
    with open('i10west_baseline_simulation_log.csv', mode='w', newline='') as log_file:
        log_writer = csv.writer(log_file)
        log_writer.writerow(["Step", "Vehicle_ID", "RSU_ID", "Signal_Strength"])

        try:
            traci.init(8813)
            print(f"Running baseline simulation for {config_file}")

            for step in range(100):  # Adjusted steps to 500
                traci.simulationStep()
                vehicle_ids = traci.vehicle.getIDList()
                
                if not vehicle_ids:
                    print("All vehicles have reached their destinations.")
                    break

                for vehicle_id in vehicle_ids:
                    vehicle_pos = traci.vehicle.getPosition(vehicle_id)
                    best_rsu = None
                    best_signal = 0

                    for rsu in rsus:
                        signal_strength = calculate_signal_strength(vehicle_pos, rsu)
                        if signal_strength > best_signal:
                            best_signal = signal_strength
                            best_rsu = rsu

                    # Log the connection and signal strength data
                    if best_rsu:
                        log_writer.writerow([step, vehicle_id, best_rsu.id, best_signal])
                        if vehicle_id not in best_rsu.connected_vehicles:
                            best_rsu.connected_vehicles.append(vehicle_id)
                        print(f"Vehicle {vehicle_id} connected to RSU {best_rsu.id} with signal {best_signal}")

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
            sys.exit()

# Load and run the simulation for I-10 West
rsus_i10 = load_rsus("I10West_junctions.txt")
print("Running baseline simulation for I-10 West...")
run_simulation("I10West.sumocfg", rsus_i10)
