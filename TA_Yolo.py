import cv2
import argparse
import os
from math import sin, cos, tan
from datetime import datetime
from ultralytics import YOLO

from inference.utils import SimpleFPS, draw_fps, draw_annotation
from TA_Control import *
import global_variable


def parse_arguments():
    # Set up argument parsing
    parser = argparse.ArgumentParser(description="YOLOv8 Object Detection Script")

    # Add arguments
    parser.add_argument('--filename', type=str, default='/home/jty/program/jtyvtol-program-last/models/v6 torus full integer.tflite',
                        help='Model filename (default: v1.pt, v1.tflite, atau model.onnx)')
    parser.add_argument('--imgsz', type=int, default=256, help='Image size for inference (default: 320px)')
    parser.add_argument('--conf', type=float, default=0.4, help='Minimum confidence for inference (default: 0.7)')

    # Parse and return arguments
    return parser.parse_args()


def object_position(center_object, grid):
    if center_object[1] < grid["L3"]:
        if center_object[0] < grid["L1"]:
            global_variable.center_object_last_grid = 1
        elif center_object[0] < grid["L2"]:
            global_variable.center_object_last_grid = 2
        else:
            global_variable.center_object_last_grid = 3
    elif center_object[1] < grid["L4"]:
        if center_object[0] < grid["L1"]:
            global_variable.center_object_last_grid = 4
        elif center_object[0] < grid["L2"]:
            global_variable.center_object_last_grid = -1
        else:
            global_variable.center_object_last_grid = 5
    else:
        if center_object[0] < grid["L1"]:
            global_variable.center_object_last_grid = 6
        elif center_object[0] < grid["L2"]:
            global_variable.center_object_last_grid = 7
        else:
            global_variable.center_object_last_grid = 8

    

def centering(vehicle, center_target, center_object, action, height_target, grid, confirmation, frame, current_height, torus_size, bullseye_size, land_size):
    # Centering logic adapted from the first script
    x_ok = False
    y_ok = False
    z_ok = False
    height_ok = False
    velocity_x = 0
    velocity_y = 0
    velocity_z = 0

    # Get center object position in grid
    object_position(center_object, grid)
    
    if global_variable.fireball_carried == False: # Offset for fireball
        offset_dekat = 40
        offset_jauh = 100
    

    # Draw Offset Square
    if global_variable.fireball_carried == False:
        cv2.rectangle(frame, (center_target[0] - offset_dekat, center_target[1] - offset_dekat), (center_target[0] + offset_dekat, center_target[1] + offset_dekat), (0, 0, 0), 2)
    else:
        cv2.rectangle(frame, (center_target[0] - offset_dekat, center_target[1] - offset_dekat), (center_target[0] + offset_dekat, center_target[1] + offset_dekat), (0, 0, 0), 2)



    # Centering Basket
    if global_variable.fireball_carried == False:
        ## X axis (forward and backward)
        if center_object[1] < center_target[1] - offset_jauh:
            velocity_x = 0.1
        elif center_object[1] < center_target[1] - offset_dekat:
            velocity_x = 0.035
        elif center_object[1] > center_target[1] + offset_jauh:
            velocity_x = -0.1
        elif center_object[1] > center_target[1] + offset_dekat:
            velocity_x = -0.035
        else:
            velocity_x = 0
            x_ok = True

        ## Y axis (left and right)
        if center_object[0] < center_target[0] - offset_jauh:
            velocity_y = -0.1
        elif center_object[0] < center_target[0] - offset_dekat:
            velocity_y = -0.035
        elif center_object[0] > center_target[0] + offset_jauh:
            velocity_y = 0.1
        elif center_object[0] > center_target[0] + offset_dekat:
            velocity_y = 0.035
        else:
            velocity_y = 0
            y_ok = True

        ## Z Axis (up and down) -> Height
        if current_height >= height_target + 0.1:
            velocity_z = 0.05
        elif current_height <= height_target:
            velocity_z = -0.02
        else:
            velocity_z = 0
            height_ok = True
        print(f"HEIGHT_OK: {height_ok}")

        # Cek XY
        if x_ok and y_ok and height_ok:
            global_variable.xy_ok += 1
        print(f"XY: {global_variable.xy_ok}")

        # Send command
        send_ned_velocity(vehicle, velocity_x, velocity_y, velocity_z)

        # Drop torus
        if global_variable.xy_ok >= confirmation:
            global_variable.do_centering = False
            global_variable.xy_ok = 0
            x_ok = False
            y_ok = False 
            height_ok = False
            global_variable.center_object_last_grid = -1
            global_variable.object_found = True
            if action == drop_fireball: #drop fireball
                global_variable.do_drop = True



def tracking(vehicle, video_resolution):
    # Set YOLOv8 to quiet mode
    os.environ['YOLO_VERBOSE'] = 'False'
    # Parse the command line arguments
    args = parse_arguments()
    # Load the YOLO model specified in the arguments
    model = YOLO(args.filename, verbose=False)
    # Initialize FPS calculator
    fps_calculator = SimpleFPS()

    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Cannot open video source")
        return

    # Create capture directory if it doesn't exist
    capture_dir = 'capture'
    os.makedirs(capture_dir, exist_ok=True)

    # Get the width and height of the frames
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # Generate a timestamp for the video filename
    timestamp = datetime.now().strftime('%Y%m%d-%H%M%S')
    video_filename = os.path.join(capture_dir, f'{timestamp}.avi')

    # Initialize VideoWriter to save the original video
    out = cv2.VideoWriter(video_filename, cv2.VideoWriter_fourcc(*'XVID'), 20.0, (frame_width, frame_height))

    centerCap = (int(video_resolution[0] / 2), int(video_resolution[1] / 2))

    grid = {"L1": int(0.25 * video_resolution[0]),
            "L2": int(0.75 * video_resolution[0]),
            "L3": int(0.25 * video_resolution[1]),
            "L4": int(0.75 * video_resolution[1])}


    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        
        # Save the original frame
        out.write(frame)

        model = YOLO("/home/jty/program/jtyvtol-program-last/models/ext_basket.tflite", verbose=False) # Model basket and exit gate

        
        results = model.predict(source=frame, imgsz=args.imgsz, conf=0.85, verbose=False)

        # Draw the inference annotations
        frame = draw_annotation(frame, model.names, results)

        # Draw the fps value
        fps, is_fps_updated = fps_calculator.get_fps()
        draw_fps(frame, fps)

        # # prev_tick = cv2.getTickCount()  # For FPS monitor
        frame = cv2.resize(frame, (video_resolution[0], video_resolution[1]))
        # frame = cv2.flip(frame, 1)  # Remove or comment out this line to stop flipping the frame

        cv2.line(frame, (grid["L1"], 0), (grid["L1"], video_resolution[1]), (255,255,255), 1)
        cv2.line(frame, (grid["L2"], 0), (grid["L2"], video_resolution[1]), (255,255,255), 1)
        cv2.line(frame, (0, grid["L3"]), (video_resolution[0], grid["L3"]), (255,255,255), 1)
        cv2.line(frame, (0, grid["L4"]), (video_resolution[0], grid["L4"]), (255,255,255), 1)

        # Get height, roll angle, pitch angle
        current_height = global_variable.rngfnd["25"]["current_distance"] / 100
        roll, pitch = vehicle.attitude.roll, vehicle.attitude.pitch

        # Defining constant value
        torus_height, camera_to_magnet, camera_to_payload_x, camera_to_payload_y = 0.1, 0.12, 0.09, 0.05
        focus_length = 0.04
        height_above_torus = current_height - torus_height

        # X and Y torus offset
        torus_offset_y = focus_length*camera_to_magnet/height_above_torus * 100 # cm
        xoff_torus = - int((current_height*cos(roll)-torus_height) * tan(roll) * 100 * 37.8) # px
        yoff_torus = int((torus_offset_y - ((current_height*cos(pitch)-torus_height) * tan(pitch)*100)) * 37.8) # px
        
        center_target = None
        if global_variable.fireball_carried == False:
            cv2.putText(frame, "Detecting Api", (500, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            action = drop_fireball
            class_target_name = "Api"
            height_target = 5 #sesuain mau lepas di ketinggian berapa
            center_target = (centerCap[0] - 55, centerCap[1] + 125)
            if current_height > 7.5 :
                # Use static center target for higher altitude
                center_target = (centerCap[0] - 68, centerCap[1] + 150)
            else:
                # Use adaptive center target for lower altitude
                center_target = (centerCap[0] - 55, centerCap[1] + 125)
            confirmation = 2     
        
        if results:
            global_variable.object_detected = False

            # Iterate through the detected objects and find the one with the highest confidence
            best_confidence = 0
            best_obj = None

            # Filtering for best confidence
            
            for obj in results[0].boxes:
                if obj.conf > best_confidence:
                    best_confidence = obj.conf
                    best_obj = obj                                                                                                                                               

            if best_obj is not None:
                class_name = model.names[int(best_obj.cls.item())]

                left   = int(best_obj.xyxy[0][0])
                top    = int(best_obj.xyxy[0][1])
                right  = int(best_obj.xyxy[0][2])
                bottom = int(best_obj.xyxy[0][3])

                # Get the center and shape of the object
                x = int(best_obj.xyxy[0][0])
                y = int(best_obj.xyxy[0][1])
                w = int(best_obj.xyxy[0][2] - best_obj.xyxy[0][0])
                h = int(best_obj.xyxy[0][3] - best_obj.xyxy[0][1])
                # shape_object = (int(x), int(y), int(x + w), int(y + h))
                center_object = (int(x + w // 2), int(y + h // 2))

                if class_name == class_target_name:
                    global_variable.object_detected = True
                    confidence_level = f"{float(best_confidence) * 100:.2f}%"  # Convert tensor to float

                    if class_name == "api": 
                        global_variable.api_counter += 1 #api counter

                    if center_target:                   
                        # Draw center target
                        cv2.circle(frame, center_target, 4, (0, 255, 255), 4)

                        # Draw line connecting center of annotation to the target
                        cv2.line(frame, center_target, center_object, (0, 0, 255), 2)

                        if global_variable.do_centering == True:
                            centering(vehicle, center_target, center_object, action, height_target, grid, confirmation, frame, current_height,)


                elif class_name == "tgtarea":
                    global_variable.object_detected = True
                    object_position(center_object, grid)


            cv2.imshow("Object Detection", frame)

            # Watermark
            cv2.putText(frame, "JENTAYU COPTER", (5, 40),
                         cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (0, 255, 0), 1)

            key = cv2.waitKey(1)
            if key == 27:  # Press 'ESC' to quit
                break

    # Release the resources
    cap.release()
    out.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    tracking()