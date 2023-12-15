import numpy as np
import sys
import shutil
import os
import csv

from enum import Enum
from converter import read_csv_initset
from utils import match_dimensions

### Enum variable for user input ###
class UserInput(Enum):
    NEW = 0
    BEST = 1
    RE = 2
    QUIT = 3

def get_user_input_init():
    print("\n[ Select the initial racing line ]")
    while True:
        print("new  : use previous optimization result")
        print("best : use best optimization result so far")
        print("re   : re-optimize the initial racing line")
        print("quit : quit the program")
        user_input = input("\nEnter your choice: ").strip().lower()
        if user_input == "" or user_input == "new":
            return UserInput.NEW
        elif user_input == "best":
            return UserInput.BEST
        elif user_input == "re":
            return UserInput.RE
        elif user_input == "quit":
            return UserInput.QUIT
        else:
            print("\n\nWarning: Invalid input. Note the following input format.")

def check_yes():
    print("If confirmed, type yes(y), or type quit(q) to exit.")
    user_input = input().strip().lower()
    while user_input not in ['yes', 'y']:
        if user_input in ['quit', 'q']:
            sys.exit("Quit by user")
        else:
            print("Invalid input")
            user_input = input().strip().lower()
    print()

def reset_directories():
    """ Reset directories for initial setting. """
    print("Resetting directories and files...")
    dirs_to_reset = ['./track', './result', './data', './param', './model']  # Added './model'
    for directory in dirs_to_reset:
        if os.path.exists(directory):
            shutil.rmtree(directory)
        os.makedirs(directory)
        if directory == './model':  
            os.makedirs(os.path.join(directory, 'engine'))  
            os.makedirs(os.path.join(directory, 'vehicle'))  
    shutil.copy('./initset/example/track.csv', './track/track.csv')
    shutil.copy('./initset/example/corner.json', './param/corner.json')
    shutil.copy('./initset/example/default.json', './param/default.json')
    shutil.copy('./initset/example/ioniq5.json', './model/vehicle/ioniq5.json')
    shutil.copy('./initset/example/ioniq5_engine.json', './model/engine/ioniq5_engine.json')

    print("...Done!\n")

def init_setting(corners, straights, track, path_initset):
    is_closed = track.closed
    number_of_samples = track.size

    # Cautionary Notes
    print("[ Initial Setting ]")
    print("Resetting will clear existing work. Recommended only once.")
    check_yes()

    print("Recommended to work with a single track in one folder.")
    print("High chance of malfunction with multiple tracks.")
    check_yes()

    print("Minimum straight segment length is 2m, minimum corner segment length is 1m")
    print("Control point interval assumed to be 1m as well")
    print("Functionality cannot be guaranteed for values less than this.")
    check_yes()

    print("Assuming all corners are part of segments: straight-corner-straight")
    print("the segments are repeated to form the track.")
    print("corner only, straight only, corner-straight, straight-corner and")
    print("corner-straight-corner are not yet implemented.")
    print("Whether the track is a closed track or open track is irrelevant.")
    check_yes()

    print("Files within the track folder need modifications")
    print("track/Track.csv: track mid points, left width, right width, road slope (optional)")
    check_yes()

    print("Files within the model folder need modifications")
    print("model/vehicle/ioniq5.json   : vehicle model")
    print("model/engine/ioniq5_engine.json    : engine model")
    check_yes()

    print("Files within the following folders need modifications: param")
    print("param/corner.json    : corner parameter values")
    print("param/default.json   : tunning parameter default values")
    check_yes()

    # Reset directories
    reset_directories()

    # Save track segments data
    corners = match_dimensions(corners)
    straights = match_dimensions(straights)
    track_size = number_of_samples + 1 if is_closed else number_of_samples
    num_of_corner = corners.shape[0]
    num_of_straight = straights.shape[0]
    num_of_segment = num_of_corner
    # print("corners") #[DEBUG]
    # print(corners) #[DEBUG]
    # print("starights") #[DEBUG]
    # print(straights) #[DEBUG]
    # # print("# of corner : ", num_of_corner) #[DEBUG]
    # print("# of straight : ", num_of_straight) #[DEBUG]

    # Check if the track has an incorrect shape
    if is_closed :
        if(num_of_corner != num_of_straight):
            raise ValueError("The closed track should have the same number of corners and straights")
    else :
        if(num_of_corner != num_of_straight-1):
            raise ValueError("In an open track, there should be one more straight than the corner")

    # arr's to store indicies
    start_arr = np.array([])
    mid1_arr = np.array([])
    mid2_arr = np.array([])
    end_arr = np.array([])

    for i in range(num_of_segment):
        j = i+1 # TO adjust index
        # Create subdirectories and copy files
        subdirectory = f'segment{j}'
        subdirectories = ['./track', './result', './param']
        for directory in subdirectories:
            os.makedirs(os.path.join(directory, subdirectory))
            if directory == './param':
                shutil.copy('./initset/example/default.json', os.path.join(directory, subdirectory, f'default_segment{j}.json'))

        start = straights[i-1][0]
        mid1  = corners[i][0]
        mid2  = corners[i][1]
        end   = straights[i][1]

        # print("start    :", start) # [DEBUG]
        # print("mid1     :", mid1) # [DEBUG]
        # print("mid2     :", mid2) # [DEBUG]
        # print("end      :", end) # [DEBUG]

        start_arr = np.append(start_arr, start)
        mid1_arr = np.append(mid1_arr, mid1)
        mid2_arr = np.append(mid2_arr, mid2)
        end_arr = np.append(end_arr, end)

        # Extract the segment data from the track data
        if start < end:
            x_arr = track.mid_xy_coordinates[0][start:end + 1]
            y_arr = track.mid_xy_coordinates[1][start:end + 1]
            left_widths_arr = track.left_widths[start:end + 1]
            right_widths_arr = track.right_widths[start:end + 1]
            road_slope_arr = track.road_slope[start:end + 1]
            friction_coefficient_arr = track.friction_coefficient[start:end + 1]
        elif start > end:
            # TODO : Not completely debugged yet (error-prone)
            x_arr = np.concatenate((track.mid_xy_coordinates[0][start:], track.mid_xy_coordinates[0][:end + 1]))
            y_arr = np.concatenate((track.mid_xy_coordinates[1][start:], track.mid_xy_coordinates[1][:end + 1]))
            left_widths_arr = np.concatenate((track.left_widths[start:], track.left_widths[:end + 1]))
            right_widths_arr = np.concatenate((track.right_widths[start:], track.right_widths[:end + 1]))
            road_slope_arr = np.concatenate((track.road_slope[start:], track.road_slope[:end + 1]))
            friction_coefficient_arr = np.concatenate((track.friction_coefficient[start:], track.friction_coefficient[:end + 1]))
        else:  # start == end
            raise ValueError("The minimum length of the segment is 4m.")

        segment_data = np.column_stack((x_arr, y_arr, left_widths_arr, right_widths_arr, road_slope_arr, friction_coefficient_arr))

        # Check if columns have the same size
        if any(len(arr) != len(segment_data) for arr in segment_data.T):
            raise ValueError(f"Warning: Columns in segment {j} do not have the same size.")
        
        # Save segment data as CSV
        segment_csv_path = os.path.join('./track', subdirectory, f'track_segment{j}.csv')
        np.savetxt(
            segment_csv_path,
            segment_data,
            delimiter=',',
            header="x,y,wl,wr,slope,friction_coefficient",
            comments="",
            fmt='%.8f'  # Specify the desired formatting
        )

    # print(start_arr)    #[DEBUG]
    # print(mid1_arr)     #[DEBUG]
    # print(mid2_arr)     #[DEBUG]
    # print(end_arr)      #[DEBUG]

    # Save initset results
    print("Saving results of initialization...")
    initset_data = read_csv_initset(path_initset)
    # print(initset_data) #[DEBUG]
    initset_data.clear()  # Clear the existing elements from initset_data list
    for i in range(4):
        new_dict = {'initset': 'true' if i == 0 else '', **{f"segment{j+1}": '' for j in range(num_of_segment)}}
        initset_data.append(new_dict)
    # Update the segment values
    for i in range(num_of_segment):
        segment_header = f"segment{i + 1}"
        initset_data[0][segment_header] = start_arr[i]
        initset_data[1][segment_header] = mid1_arr[i]
        initset_data[2][segment_header] = mid2_arr[i]
        initset_data[3][segment_header] = end_arr[i]
    # Save the updated initset_data back to the initset.csv file
    with open(path_initset, 'w', newline='') as csv_file:
        fieldnames = initset_data[0].keys()
        csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        csv_writer.writeheader()
        csv_writer.writerows(initset_data)
    print("...Done!\n")


def check_initset(path_initset):
    try:
        with open(path_initset, 'r') as csv_file:
            csv_reader = csv.DictReader(csv_file)
            initset_data = list(csv_reader)
        #print(initset_data) # [DEBUG]
        initset_value = initset_data[0]['initset'].lower()
        if initset_value == 'false':
            return False
        elif initset_value == 'true':
            return True
        else :
            sys.exit("initset in ./initset/initset.csv file should be boolean")
    except Exception as e :
        sys.exit(e)