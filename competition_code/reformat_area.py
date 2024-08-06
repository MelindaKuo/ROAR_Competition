import numpy as np

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
    if len(coordinates) < 2:
        return None, None

    x = coordinates[0].strip()
    y = coordinates[1].strip()
    
    return x, y

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

def extract_rotation(line):
    start_str = 'roll_pitch_yaw=array(['
    end_str = '])'

    start_idx = line.find(start_str)
    if start_idx == -1:
        return None

    start_idx += len(start_str)
    end_idx = line.find(end_str, start_idx)

    rotations_str = line[start_idx+1:end_idx].strip()
    return rotations_str

# Process and write coordinates
coordinates = []
with open('points.txt', 'r') as infile, open('formatted_points.txt', 'w') as outfile:
    for line in infile:
        x, y = extract_coordinates(line)
        if x is not None and y is not None:
            outfile.write(f'[{x}, {y}],\n')

# Read and format coordinates
coordinates = []
with open('formatted_points.txt', 'r') as file:
    lines = file.readlines()
    # Remove the trailing comma from the last line
    if lines:
        lines[-1] = lines[-1].rstrip(',\n')
    
    for line in lines:
        line = line.strip().strip('[],')
        if line:
            x, y = map(float, line.split(','))
            coordinates.append([x, y])

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
with open('points.txt', 'r') as infile, open('formatted_points.txt', 'a') as outfile:
    for line in infile:
        line = line.strip('[],')
        rot = extract_rotation(line)
        if rot:
            rotations.append('[' + rot)
            
rotations_array = np.array(rotations)
# Save the NumPy array to an .npz file
np.savez('coordinates.npz', locations=coordinates_array, rotations = rotations_array, lane_widths=np.array(lane_widths))

print("Coordinates and lane widths have been saved to 'coordinates.npz'")
