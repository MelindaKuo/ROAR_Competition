import numpy as np
import re

def extract_coordinates(line):
    start_str = "location=array(["
    end_str = "])"

    start_idx = line.find(start_str)
    if start_idx == -1:
        return None, None

    start_idx += len(start_str)
    end_idx = line.find(end_str, start_idx)
    if end_idx == -1:
        return None, None

    # Extract the substring containing the coordinates
    coordinates_str = line[start_idx:end_idx]

    # Split the coordinates string and extract x and y values
    coordinates = coordinates_str.split(',')
    if len(coordinates) < 3:
        return None, None, None

    x = coordinates[0].strip()
    y = coordinates[1].strip()
    z = coordinates[2].strip()
    
    return x, y,z

def extract_lane_width(line):
    start_str = 'lane_width='
    end_str = ')'

    start_idx = line.find(start_str)
    if start_idx == -1:
        return None

    start_idx += len(start_str)
    end_idx = line.find(end_str, start_idx)

    lane_widths = line[start_idx:end_idx].strip()
    return lane_widths

def extract_rotation(waypoint_str):
    # Find the starting position of 'roll_pitch_yaw=array(['
    start = waypoint_str.find('roll_pitch_yaw=array([')
    if start == -1:
        return None

    # Find the position of the closing bracket
    start += len('roll_pitch_yaw=array([')
    end = waypoint_str.find(']', start)

    if end == -1:
        return None

    # Extract the content inside the brackets
    roll_pitch_yaw_content = waypoint_str[start:end].strip()

    # Check if the content is empty and format accordingly
    if roll_pitch_yaw_content == '':
        return '[0,0,0]'
    else:
        return  roll_pitch_yaw_content 

# Process and write coordinates
coordinates = []
with open('points.txt', 'r') as infile, open('formatted_points.txt', 'w') as outfile:
    for line in infile:
        x, y ,z= extract_coordinates(line)
        if x is not None and y is not None:
            outfile.write(f'[{x}, {y}, {z}]\n')

# Read and format coordinates
coordinates = []
with open('formatted_points.txt', 'r') as file:
    lines = file.readlines()
    for line in lines:
        line = line.strip().strip('[]')  # Remove leading/trailing whitespace and brackets
        if line:
            try:
                # Split the coordinates string and convert to float
                x, y, z = map(float, line.split(','))
                
                # Check if z is 0. and set it to 0 if necessary
                if z == 0.:
                    z = 0
                
                coordinates.append([x, y, z])
            except ValueError:
                # Handle cases where conversion fails
                print(f"Warning: Could not convert line to floats: '{line}'")

coordinates_array = np.array(coordinates)


# Process and append lane widths
lane_widths = []
with open('points.txt', 'r') as infile, open('formatted_points.txt', 'a') as outfile:
    for line in infile:
        lane_widths_str = extract_lane_width(line)
        if lane_widths_str:
            lane_widths.extend(map(float, lane_widths_str.split(',')))



#Process and append Rotations
rotations = []
with open('points.txt', 'r') as infile:
    for line in infile:
        line = line.strip()
        rot = extract_rotation(line)
        if rot:
            # Convert the extracted rotation string to a list of floats
            rot = rot.strip('[]')  # Remove the brackets
            rotation_values = list(map(float, rot.split(',')))
            rotations.append(rotation_values)


rotations_array = np.array(rotations).reshape(-1, 3)
# Save the NumPy array to an .npz file
np.savez('coordinates.npz', locations=coordinates_array, rotations = rotations_array, lane_widths=np.array(lane_widths))

print("Coordinates and lane widths have been saved to 'coordinates.npz'")
