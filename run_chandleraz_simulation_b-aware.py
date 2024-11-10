import traci
import math
import networkx as nx
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

# Load RSUs from file
def load_rsus(file_path):
    rsus = []
    with open(file_path, 'r') as file:
        next(file)
        for line in file:
            id, x, y = line.strip().split(',')
            rsus.append(RSU(int(id), float(x), float(y)))
    return rsus

# Predict future vehicle position based on speed and angle
def predict_future_position(vehicle_id, time_ahead=2):
    pos = traci.vehicle.getPosition(vehicle_id)
    speed = traci.vehicle.getSpeed(vehicle_id)
    angle = math.radians(traci.vehicle.getAngle(vehicle_id))
    future_x = pos[0] + speed * time_ahead * math.cos(angle)
    future_y = pos[1] + speed * time_ahead * math.sin(angle)
    return (future_x, future_y)

# Calculate if an obstacle will block LOS
def is_blocking_los(vehicle_pos, rsu, obstacle_pos, obstacle_size):
    dist_to_rsu = math.dist(vehicle_pos, (rsu.x, rsu.y))
    obstacle_dist = math.dist(vehicle_pos, obstacle_pos)
    return obstacle_dist < dist_to_rsu and obstacle_dist < obstacle_size / 2

# Calculate signal strength based on distance and blockage
def calculate_signal_strength(vehicle_pos, rsu, obstacles):
    distance = math.sqrt((vehicle_pos[0] - rsu.x) ** 2 + (vehicle_pos[1] - rsu.y) ** 2)
    for obstacle in obstacles:
        obstacle_pos = traci.vehicle.getPosition(obstacle)
        obstacle_size = max(traci.vehicle.getLength(obstacle), traci.vehicle.getWidth(obstacle))
        if is_blocking_los(vehicle_pos, rsu, obstacle_pos, obstacle_size):
            distance *= 1.5  # Increase effective distance if blocked
    return max(0, 1 / (1 + distance)) if distance > 0 else 1.0

# Estimate connection duration based on predicted positions
def estimate_connection_duration(vehicle_id, rsu, max_duration=10, step_size=1):
    duration = 0
    for t in range(1, max_duration + 1, step_size):
        future_pos = predict_future_position(vehicle_id, time_ahead=t)
        distance = math.sqrt((future_pos[0] - rsu.x) ** 2 + (future_pos[1] - rsu.y) ** 2)
        if distance > 400:
            break
        duration += step_size
    return duration

# Construct and find optimal RSU schedule using DAG
def construct_dag(vehicle_id, rsus, horizon=10, timestep=1):
    G = nx.DiGraph()
    start_node = ("start", 0)
    G.add_node(start_node)
    last_nodes = {start_node}

    for t in range(1, horizon + 1):
        future_pos = predict_future_position(vehicle_id, t * timestep)
        current_nodes = set()
        
        for rsu in rsus:
            distance = math.sqrt((future_pos[0] - rsu.x) ** 2 + (future_pos[1] - rsu.y) ** 2)
            if distance <= 400:
                node = (rsu.id, t)
                G.add_node(node)
                potential_datarate = 1 / (1 + distance)
                edge_weight = -potential_datarate

                for last_node in last_nodes:
                    G.add_edge(last_node, node, weight=edge_weight)
                current_nodes.add(node)

        last_nodes = current_nodes or last_nodes

    end_node = ("end", horizon + 1)
    G.add_node(end_node)
    for node in last_nodes:
        G.add_edge(node, end_node, weight=0)
    
    try:
        path = nx.shortest_path(G, source=start_node, target=end_node, weight="weight")
    except nx.NetworkXNoPath:
        path = [start_node]
    
    return path

# Function to run B-AWARE simulation with RSU scheduling
def run_b_aware_simulation(config_file, rsus):
    sumo_process = subprocess.Popen(["sumo-gui", "-c", config_file, "--remote-port", "8813"])
    time.sleep(2)

    with open('chandleraz_baware_simulation_log.csv', mode='w', newline='') as log_file:
        log_writer = csv.writer(log_file)
        log_writer.writerow(["Step", "Vehicle_ID", "RSU_ID", "Signal_Strength", "Expected_Connection_Time"])

        try:
            traci.init(8813)
            print("Starting B-AWARE simulation with DAG optimization")

            for step in range(100):
                traci.simulationStep()
                vehicle_ids = traci.vehicle.getIDList()
                print(f"Step {step}: Processing {len(vehicle_ids)} vehicles")

                for vehicle_id in vehicle_ids:
                    optimal_schedule = construct_dag(vehicle_id, rsus)

                    for rsu_id, t in optimal_schedule:
                        if rsu_id == "end":
                            continue
                        rsu = next((r for r in rsus if r.id == rsu_id), None)
                        if rsu:
                            vehicle_pos = traci.vehicle.getPosition(vehicle_id)
                            nearby_obstacles = [v for v in traci.vehicle.getIDList() if v != vehicle_id]
                            signal_strength = calculate_signal_strength(vehicle_pos, rsu, nearby_obstacles)
                            connection_duration = estimate_connection_duration(vehicle_id, rsu)
                            print(f"Vehicle {vehicle_id} connecting to RSU {rsu.id} at step {step} with signal {signal_strength} and connection time {connection_duration}s")
                            log_writer.writerow([step, vehicle_id, rsu.id, signal_strength, connection_duration])

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

# Load RSUs and run the B-AWARE simulation
rsus_chandler = load_rsus("ChandlerAZ_junctions.txt")
run_b_aware_simulation("ChandlerAZ.sumocfg", rsus_chandler)
