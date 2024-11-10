import sumolib

# Function to extract and save all junction coordinates in a text file
def extract_junctions(net_file, output_file):
    # Load the network
    net = sumolib.net.readNet(net_file)
    
    # Open the output file in write mode
    with open(output_file, 'w') as file:
        file.write("Junction ID, X, Y\n")  # Write header

        # Extract and print all junctions with their coordinates
        junctions = []
        for node in net.getNodes():
            node_id = node.getID()
            x, y = node.getCoord()
            junctions.append((node_id, x, y))
            
            # Print to console
            print(f"Junction ID: {node_id}, Coordinates: ({x}, {y})")
            
            # Write to file
            file.write(f"{node_id}, {x}, {y}\n")
    
    print(f"Junction coordinates have been saved to {output_file}")
    return junctions

# Run the function for I-10 West map and save to text file
print("Junctions for ChandlerAZ:")
i10_junctions = extract_junctions("ChandlerAZ.net.xml", "ChandlerAZ_junctions.txt")  # Replace with the actual path if needed
