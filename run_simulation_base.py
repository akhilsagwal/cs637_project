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

# Function to load RSUs from a text file
def load_rsus_from_file(file_path):
    rsus = []
    with open(file_path, 'r') as file:
        next(file)  # Skip header line
        for line in file:
            id, x, y = line.strip().split(', ')
            rsus.append(RSU(id=int(id), x=float(x), y=float(y)))
    return rsus

# Function to check for obstacles in the line of sight (LOS) between vehicle and RSU
def check_for_obstacles(vehicle_pos, rsu, vehicle_id):
    obstacles = traci.vehicle.getIDList()  # Get list of all vehicles
    for obstacle_id in obstacles:
        if obstacle_id == vehicle_id:
            continue
        obstacle_pos = traci.vehicle.getPosition(obstacle_id)
        if (min(vehicle_pos[0], rsu.x) <= obstacle_pos[0] <= max(vehicle_pos[0], rsu.x) and
            min(vehicle_pos[1], rsu.y) <= obstacle_pos[1] <= max(vehicle_pos[1], rsu.y)):
            return True  # Obstacle found in LOS
    return False

# Function to calculate signal strength with obstacle consideration
def calculate_signal_strength(vehicle_pos, rsu, vehicle_id):
    distance = math.sqrt((vehicle_pos[0] - rsu.x) ** 2 + (vehicle_pos[1] - rsu.y) ** 2)
    if distance == 0:
        return 1.0
    
    # Reduce signal strength if an obstacle is in the LOS
    is_blocked = check_for_obstacles(vehicle_pos, rsu, vehicle_id)
    if is_blocked:
        return max(0, 1 / (1 + distance * 5))  # Reduced signal strength due to obstruction
    else:
        return max(0, 1 / (1 + distance))

# Function to run the RSU selection simulation
def run_simulation(config_file, rsu_file, log_file_name):
    rsus = load_rsus_from_file(rsu_file)  # Load RSUs for the specific map

    # Start SUMO as a subprocess with TraCI server enabled
    sumo_process = subprocess.Popen(
        ["sumo-gui", "-c", config_file, "--remote-port", "8813"]
    )
    time.sleep(2)  # Wait for SUMO to initialize

    # Open a CSV file for logging
    with open(log_file_name, mode='w', newline='') as log_file:
        log_writer = csv.writer(log_file)
        log_writer.writerow(["Step", "Vehicle_ID", "RSU_ID", "Signal_Strength"])

        try:
            traci.init(8813)  # Connect to the TraCI server on port 8813
            print(f"Running baseline simulation for configuration: {config_file}")

            for step in range(500):  # Adjusted steps to 500
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
                        signal_strength = calculate_signal_strength(vehicle_pos, rsu, vehicle_id)
                        if signal_strength > best_signal:
                            best_signal = signal_strength
                            best_rsu = rsu

                    if best_rsu:
                        # Log the connection data
                        log_writer.writerow([step, vehicle_id, best_rsu.id, best_signal])
                        if vehicle_id not in best_rsu.connected_vehicles:
                            best_rsu.connected_vehicles.append(vehicle_id)
                        print(f"Vehicle {vehicle_id} connected to RSU {best_rsu.id} with signal {best_signal}")

        except Exception as e:
            print(f"An error occurred: {e}")

        finally:
            traci.close()
            print("Simulation completed and traci closed.")
            if sumo_process.poll() is None:
                sumo_process.terminate()
                try:
                    sumo_process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    sumo_process.kill()
            print("SUMO process terminated.")
            sys.exit()

# Run the simulation for both maps with logging

# I-10 West Simulation
print("Running baseline simulation for I-10 West...")
run_simulation("I10West.sumocfg", "I10West_junctions.txt", "i10west_baseline_simulation_log.csv")

# Chandler AZ Simulation
print("Running baseline simulation for Chandler, AZ...")
run_simulation("ChandlerAZ.sumocfg", "ChandlerAZ_junctions.txt", "chandleraz_baseline_simulation_log.csv")
