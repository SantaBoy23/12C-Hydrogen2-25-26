import csv
import math

#Formats files
INPUT_FILE = 'skills_recording.csv'
OUTPUT_FILE = 'generated_auton.cpp'

#setup x, y
setup_x = float(input("Enter X position: "))
setup_y = float(input("Enter Y position: "))
setup_rot = float(input("Enter Rotation angle: "))

#path creation when things change (movement or turns)
MIN_DIST_CHANGE = 11.0 #was12 #was 15
MIN_ANGLE_CHANGE = 11.0 #was 12 #was 15
MIN_X_CHANGE = 2.75 #was2 #was 2.5
MIN_Y_CHANGE = 2.75 #was 2 #was2.5
SCALE_FACTOR = 1.0 #use this to change values if distances are off (for odom x,y)

#formats file for conmversion
def get_value(row, keys, default=0.0):
    for key in keys:
        if key in row:
            return float(row[key])
    return default

#main part of function that creates odom movements
def main():
    #pulls from CSV file to create new generated code file
    try:
        with open(INPUT_FILE, 'r') as f:
            reader = csv.DictReader(f, skipinitialspace=True)
            reader.fieldnames = [h.strip() for h in reader.fieldnames]
            data = list(reader)
    #aborts if unable to find correct file
    except FileNotFoundError:
        print(f"Error: Could not find {INPUT_FILE}")
        return

    #sets up header/auton name in .cpp file
    output_lines = []
    output_lines.append("void generated_skills_auto() {")
    
    #gets initial values for odom and imu
    first = data[0]
    start_x = get_value(first, ['odom_x']) * SCALE_FACTOR
    start_y = get_value(first, ['odom_y']) * SCALE_FACTOR
    start_rot = get_value(first, ['rotation', 'imu_heading'])

    #adds starting position to the code
    output_lines.append(f"  //Set Starting Position")
    output_lines.append(f"  chassis.odom_xyt_set({setup_x:.2f}_in, {setup_y:.2f}_in, {setup_rot:.2f}_deg);")
    output_lines.append("")

    #tracks the state of pistons, intake, etc.
    last_x = setup_x
    last_y = setup_y
    last_rot = setup_rot
    prev_x = setup_x
    prev_y = setup_y
    last_intake_top = int(get_value(first, ['intake_top']))
    last_intake_bot = int(get_value(first, ['intake_bottom']))
    last_center = int(get_value(first, ['center_piston']))
    last_match = int(get_value(first, ['match_load_piston']))
    last_odom = int(get_value(first, ['odom_pod_piston']))
    last_reset = int(get_value(first, ['odom_reset']))

    count = 0
    step = 1

    #when within the data set provided by CSV file, do...
    for i in range(1, len(data)):
        row = data[i]
        
        #gets odom and imu values
        curr_x = get_value(row, ['odom_x']) - start_x
        curr_y = get_value(row, ['odom_y']) - start_y
        curr_rot = get_value(row, ['rotation', 'imu_heading']) - start_rot
        
        #gets motor and piston values
        curr_intake_top = int(get_value(row, ['intake_top']))
        curr_intake_bot = int(get_value(row, ['intake_bottom']))
        curr_center = int(get_value(row, ['center_piston']))
        curr_match = int(get_value(row, ['match_load_piston']))
        curr_odom = int(get_value(row, ['odom_pod_piston']))
        curr_reset = int(get_value(row, ['odom_reset']))

        #calculate whether enough has changed to set new point
        dist = math.sqrt((curr_x - last_x)**2 + (curr_y - last_y)**2)
        angle_diff = abs(curr_rot - last_rot)
        
        mech_changed = (curr_intake_top != last_intake_top or 
                        curr_intake_bot != last_intake_bot or 
                        curr_center != last_center or 
                        curr_match != last_match or 
                        curr_odom != last_odom or
                        curr_reset != last_reset)

        #creates new points when the max distance/angle has been passed, or a piston/motor has changed state
        if dist > MIN_DIST_CHANGE or angle_diff > MIN_ANGLE_CHANGE or mech_changed:
            
            #add comment based on what created the new point (very simple currently and will be manually edited to reflect specific actions later on)
            if mech_changed:
                output_lines.append(f"  //Mechanism State Changed")
            elif angle_diff > 20:
                output_lines.append(f"  //Turn")
            else:
                output_lines.append(f"  //Move")

            dx = curr_x - last_x
            dy = curr_y - last_y

            # convert heading to radians
            heading_rad = math.radians(curr_rot)

            # robot forward direction vector
            fx = math.cos(heading_rad)
            fy = math.sin(heading_rad)

            # dot product
            dot = dx * fx + dy * fy

            direction = "rev" if dot >= 0 else "fwd"

            
            #makes it so that if the robot is moving a a straight line, it doen't have to use boomerang, and can siimply move straight
            if abs(curr_x) - abs(prev_x) < MIN_X_CHANGE:
                #formats above data into actual odom movements (disables slew for faster movements)
                output_lines.append(f"  chassis.pid_odom_set({{{{{prev_x:.2f}_in, {curr_y:.2f}_in}}, {direction}, 110}}, false);")
                output_lines.append("  chassis.pid_wait_quick_chain();")
            elif abs(curr_y) - abs(prev_y) < MIN_Y_CHANGE:
                output_lines.append(f"  chassis.pid_odom_set({{{{{curr_x:.2f}_in, {prev_y:.2f}_in}}, {direction}, 110}}, false);")
                output_lines.append("  chassis.pid_wait_quick_chain();")
            else:
                output_lines.append(f"  chassis.pid_odom_set({{{{{curr_x:.2f}_in, {curr_y:.2f}_in, {curr_rot:.2f}_deg}}, {direction}, 110}}, false);")
                output_lines.append("  chassis.pid_wait_quick_chain();")
            
            #adds whenever a motor/piston changes state
            if mech_changed:
                if curr_intake_top != last_intake_top:
                    output_lines.append(f"  intakeTop.move({curr_intake_top}); // Top Intake")
                if curr_intake_bot != last_intake_bot:
                    output_lines.append(f"  intakeBottom.move({curr_intake_bot}); // Bottom Intake")
                if curr_center != last_center:
                    output_lines.append(f"  center.set({'true' if curr_center else 'false'}); // Center Piston")
                if curr_match != last_match:
                    output_lines.append(f"  matchLoad.set({'true' if curr_match else 'false'}); // Match Load")
                if curr_odom != last_odom:
                    output_lines.append(f"  odomPod.set({'true' if curr_odom else 'false'}); // Odom Pod")
                if curr_reset != last_reset:
                    output_lines.append(f"  chassis.odom_xyt_set({curr_x:.2f}_in, {curr_y:.2f}_in, {curr_rot:.2f}_deg);")
            
            output_lines.append("")

            #updates the states of previous variables
            if abs(curr_x) - abs(prev_x) < MIN_X_CHANGE:
                last_x, last_y, last_rot = curr_x, curr_y, curr_rot
                prev_y = curr_y
                last_intake_top, last_intake_bot = curr_intake_top, curr_intake_bot
                last_center, last_match, last_odom = curr_center, curr_match, curr_odom
                last_reset = curr_reset
            elif abs(curr_y) - abs(prev_y) < MIN_Y_CHANGE:
                last_x, last_y, last_rot = curr_x, curr_y, curr_rot
                prev_x = curr_x
                last_intake_top, last_intake_bot = curr_intake_top, curr_intake_bot
                last_center, last_match, last_odom = curr_center, curr_match, curr_odom
                last_reset = curr_reset
            else:
                last_x, last_y, last_rot = curr_x, curr_y, curr_rot
                prev_x, prev_y = curr_x, curr_y
                last_intake_top, last_intake_bot = curr_intake_top, curr_intake_bot
                last_center, last_match, last_odom = curr_center, curr_match, curr_odom
                last_reset = curr_reset
            
            count += 1
            step += 1

    #finishes route and stops all motors
    output_lines.append("  chassis.drive_set(0, 0);")
    output_lines.append("  intakeTop.move(0);")
    output_lines.append("  intakeBottom.move(0);")
    output_lines.append("}")

    with open(OUTPUT_FILE, 'w') as f:
        f.write('\n'.join(output_lines))

    # tells how many odom movements were created from how many raw data points from the CSV file
    #(this really isn't necessary for the final verison, I just find it kinda cool, and like to use it for testing)
    print(f"Success! Turned {len(data)} raw points into {count} odom movements.")
    print(f"File saved to: {OUTPUT_FILE}")

if __name__ == "__main__":
    main()
