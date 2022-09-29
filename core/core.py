#!/usr/bin/env python
# coding: utf-8

# Object Detection
#
# Object detection using a custom generated model
# 
# Arduino controlling stepper motor with webcam and proximity sensor


# For drawing onto the image.
import cv2 
import numpy as np
from PIL import Image
from PIL import ImageColor
from PIL import ImageDraw
from PIL import ImageFont
from PIL import ImageOps

# For time measurement
import time
from time import process_time_ns

# Serial for HEAD
import serial

# General Import
import os
import sys
from threading import Thread
import importlib.util
import matplotlib.pyplot as plt
import statistics


# Sensitivity for each detection
MIN_DECTETION_SCORE = 0.45

# Remove detection if they are on bounday. 
# It seems to be a good idea, since many faulty detections is occuring.
# Also the tracking will lock onto the window, so its best to be enabled.
BOUNDARY_SENSITIVITY = 0.01
BOUNDARY_DETECTIONS_ON = True

# Draw all detection areas
DRAW_ALL_DET_AREA = True

# Draw All detections with respect to MIN_DETECTION_SCORE
DRAW_ALL_DET = True

# Draw strongest detection if more than MIN_DETECTION_SCORE
DRAW_STRONGEST_DET = True

# Draw status
DRAW_STATUS = True

# Max detections to be processed per detector
global DET_MAX_PROC
DET_MAX_PROC = 7


# Model Selection

# Automatic model selection regarding if testing on PC or RaspberryPi+EdgeTPU
MODEL_AUTO_EN = True
if MODEL_AUTO_EN:
    # Assume that Linux is on RaspberryPi
    if os.name == 'posix':
        TFLITE_EN   = True
        TFLITE_PC   = False
        EDGE_TPU_EN = True
        MODEL_DIR   = '../../18_08_2022_efficientdet-lite1_e75_b32_s2000/'
    else:
        TFLITE_EN   = True
        TFLITE_PC   = True
        EDGE_TPU_EN = False
        MODEL_DIR   = '../../../../tflite_custom_models/good/18_08_2022_efficientdet-lite1_e75_b32_s2000/'
else:
    TFLITE_EN   = True
    TFLITE_PC   = True
    EDGE_TPU_EN = False
    MODEL_DIR   = '../../../../tflite_custom_models/good/18_08_2022_efficientdet-lite1_e75_b32_s2000/'


# TFLITE
if TFLITE_EN:
    # Import TensorFlow libraries
    # If tflite_runtime is installed, import interpreter from tflite_runtime, else import from regular tensorflow
    # If using Coral Edge TPU, import the load_delegate library
    pkg = importlib.util.find_spec('tflite_runtime')
    if pkg:
        from tflite_runtime.interpreter import Interpreter
        if EDGE_TPU_EN:
            from tflite_runtime.interpreter import load_delegate
    else:
        from tensorflow.lite.python.interpreter import Interpreter
        if EDGE_TPU_EN:
            from tensorflow.lite.python.interpreter import load_delegate
    # Load Model
    if TFLITE_PC: 
        if EDGE_TPU_EN:
            # Edge TPU TFLITE
            print('No support for Edge TPU on PC...')
            exit()
        else:
            # float16 TFLITE
            interpreter = Interpreter(model_path=os.path.join(MODEL_DIR,'model_float16.tflite'))
            print('Loading TFLITE Float16...')
    else:
        if EDGE_TPU_EN:
            # Edge TPU TFLITE
            interpreter = Interpreter(model_path=os.path.join(MODEL_DIR,'edge_tpu_2','model_default_edgetpu.tflite'),
                                      experimental_delegates=[load_delegate('libedgetpu.so.1.0')])
            print('Loading TFLITE for Edge TPU...')
        else:
            # Default TFLITE
            interpreter = Interpreter(model_path=os.path.join(MODEL_DIR,'model_default.tflite'))
            print('Loading TFLITE Default...')
    # Allocate    
    interpreter.allocate_tensors()
    # Get model details
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    height = input_details[0]['shape'][1]
    width = input_details[0]['shape'][2]
    print('H: '+str(height)+' W: '+str(width))
    tflite_model_height = height
    tflite_model_width  = width

    floating_model = (input_details[0]['dtype'] == np.float32)

    input_mean = 127.5
    input_std = 127.5

    # Check output layer name to determine if this model was created with TF2 or TF1,
    # because outputs are ordered differently for TF2 and TF1 models
    outname = output_details[0]['name']

    if ('StatefulPartitionedCall' in outname): # This is a TF2 model
        boxes_idx, classes_idx, scores_idx = 1, 3, 0
    else: # This is a TF1 model
        boxes_idx, classes_idx, scores_idx = 0, 1, 2
else:
    import tensorflow as tf
    # Print Tensorflow version
    print('Tensorflow version: '+tf.__version__)
    # Init Model
    detect_fn = tf.saved_model.load(MODEL_DIR+'saved_model/')


# Load Labels
category_index = {}
labels_file = open(MODEL_DIR+'labels.txt', 'r')
lines_labels_file = labels_file.readlines()
i = 1
for line in lines_labels_file:
    category_index[i] = {'id': i, 'name': line.split('\n')[0]}
    i = i + 1
print(category_index)
# Define a list of colors for visualization
np.random.seed(1931)
COLORS = np.random.randint(25, 230, size=(i, 3), dtype=np.uint8)
color_selection = []
for color in COLORS:
    color_selection.append('#{0:02x}{1:02x}{1:02x}'.format(color[2],color[1],color[0]))
print(color_selection)



# Functions - Drawing

# Set general font
try:
    if os.name == 'posix':
        # Linux
        font = ImageFont.truetype("/usr/share/fonts/truetype/ttf-bitstream-vera/VeraBd.ttf",15)
    else:
        # Windows
        font = ImageFont.truetype("C:/Windows/Fonts/Arial/ariblk.ttf",15)
except IOError:
    print("Font not found, using default font.")
    font = ImageFont.load_default() 

def draw_boxes(image, index, boxes, class_names, scores, max_boxes=10, min_score=0.1):
    
    # Only check a limited number of boxes
    global DET_MAX_PROC
    if DET_MAX_PROC == -1:
        loop_range = range(min(boxes.shape[0], max_boxes))
    else:
        loop_range = range(DET_MAX_PROC)
    
    global font
    """Overlay labeled boxes on an image with formatted scores and label names."""
    image_pil = Image.fromarray(np.uint8(image)).convert("RGB")
    for i in loop_range:
        if scores[i] >= min_score:
            ymin, xmin, ymax, xmax = tuple(boxes[i])
            display_str = "a{} {}: {}%".format(int(index),category_index[class_names[i]]['name'],int(100 * scores[i]))
            color = color_selection[class_names[i]]
            draw_bounding_box_on_image(
                image_pil,
                ymin,
                xmin,
                ymax,
                xmax,
                color,
                font,
                display_str_list=[display_str])
    np.copyto(image, np.array(image_pil))
    return image

def draw_bounding_box_on_image(
    image,
    ymin,
    xmin,
    ymax,
    xmax,
    color,
    font,
    thickness=2,
    display_str_list=()):

    """Adds a bounding box to an image."""
    draw = ImageDraw.Draw(image)
    im_width, im_height = image.size
    (left, right, top, bottom) = (xmin * im_width, xmax * im_width,
                                ymin * im_height, ymax * im_height)
    draw.line([(left, top), (left, bottom), (right, bottom), (right, top),
             (left, top)],
            width=thickness,
            fill=color)

    # If the total height of the display strings added to the top of the bounding
    # box exceeds the top of the image, stack the strings below the bounding box
    # instead of above.
    display_str_heights = [font.getsize(ds)[1] for ds in display_str_list]
    # Each display_str has a top and bottom margin of 0.05x.
    total_display_str_height = (1 + 2 * 0.05) * sum(display_str_heights)

    if top > total_display_str_height:
        text_bottom = top
    else:
        text_bottom = top + total_display_str_height
    # Reverse list and print from bottom to top.
    for display_str in display_str_list[::-1]:
        text_width, text_height = font.getsize(display_str)
        margin = np.ceil(0.05 * text_height)
        draw.rectangle([(left, text_bottom - text_height - 2 * margin),
                    (left + text_width, text_bottom)],
                   fill=color)
        draw.text((left + margin, text_bottom - text_height - margin),
              display_str,
              fill="black",
              font=font)
        text_bottom -= text_height - 2 * margin

def draw_detections_list(detections_list, start_index, img):
    i = start_index
    for detections in detections_list:
        image_with_boxes = draw_boxes(
            img, 
            i,
            detections["detection_boxes"],
            detections["detection_classes"], 
            detections["detection_scores"],
            max_boxes=10, 
            min_score=MIN_DECTETION_SCORE)
        i = i + 1
    img = image_with_boxes

def find_strongest_detection(detections_list, start_index):
    
    # Only check a limited number of boxes
    global DET_MAX_PROC
    if DET_MAX_PROC == -1:
        loop_range = range(len(detections['detection_scores']))
    else:
        loop_range = range(DET_MAX_PROC)
        
    i_tmp = 0
    j_tmp = 0
    
    i = start_index
    max_score = 0.0
    found_max = False
    for detections in detections_list:
        for j in loop_range:
            if detections["detection_scores"][j] > max_score:
                max_score = detections["detection_scores"][j]
                i_tmp = i
                j_tmp = j
                found_max = True
        i = i + 1
     
    return found_max, max_score, i_tmp, j_tmp

def draw_strongest_detection(detections_list, max_score, i_tmp, j_tmp, img, color_in, area_index_offset):
    
    boxes = []
    scores = []
    classes = []
    boxes.append(detections_list[i_tmp]['detection_boxes'][j_tmp])
    scores.append(detections_list[i_tmp]['detection_scores'][j_tmp])
    classes.append(detections_list[i_tmp]['detection_classes'][j_tmp])
    
    global font
    image_pil = Image.fromarray(np.uint8(img)).convert("RGB")
    if max_score >= MIN_DECTETION_SCORE:
        ymin, xmin, ymax, xmax = tuple(boxes[0])
        display_str = "a{} {}: {}%".format(int(i_tmp+area_index_offset),category_index[classes[0]]['name'],int(100 * scores[0]))
        if color_in == -1:
            color = color_selection[classes[0]]
        else:
            color = color_in
        image_pil = Image.fromarray(np.uint8(img)).convert("RGB")
        draw_bounding_box_on_image(
                image_pil,
                ymin,
                xmin,
                ymax,
                xmax,
                color,
                font,
                display_str_list=[display_str])
        np.copyto(img, np.array(image_pil))

    return img

def draw_detection_areas(area_coord_list, start_index, img):
    i = start_index
    for area_coord in area_coord_list:
        display_str = "a"+str(i)
        color = (255,255,255)
        image_pil = Image.fromarray(np.uint8(img)).convert("RGB")
        ymin = area_coord[4]/cam_window_height
        xmin = area_coord[2]/cam_window_width
        ymax = area_coord[5]/cam_window_height
        xmax = area_coord[3]/cam_window_width
        draw_bounding_box_on_image(
            image_pil,
            ymin,
            xmin,
            ymax,
            xmax,
            color,
            font,
            display_str_list=[display_str])
        np.copyto(img, np.array(image_pil))
        i=i+1
            
    return img

def draw_general_status(img):
   
    # Cycle Time
    global cycle_time_total 
    global cycle_time_avg  
    str_cycle_time_total = "Calc Time: "
    str_cycle_time_avg   = "Avg. Time: "
    str_cycle_time_total = str_cycle_time_total+str(round(cycle_time_total))
    str_cycle_time_avg = str_cycle_time_avg+str(round(cycle_time_avg))
    
    # Head Status
    global head_logic_steps
    global head_step_index 
    global head_distance
    str_head_angle       = "Angle:     "
    str_head_distance    = "Distance:  "
    if HEAD_EN == 1:    
        head_angle = round(1/head_logic_steps * head_step_index * 360, 1)
        str_head_angle    = str_head_angle+str(head_angle)
        str_head_distance = str_head_distance+str(head_distance)
    else:
        str_head_angle = str_head_angle+"Na"
        str_head_distance = str_head_distance+"Na"
    
    # Target
    global control_state_mean_x           
    global control_state_var_x            
    global control_state_mean_y           
    global control_state_var_y            
    str_mean_x = 'Mean X: {0:.3f}'.format(control_state_mean_x)
    str_var_x  = 'Var X: {0:.6f}'.format(control_state_var_x)
    
    # Concat all strings
    lines = [
        str_cycle_time_total,
        str_cycle_time_avg,
        str_head_angle,
        str_head_distance,
        str_mean_x,
        str_var_x]
    
    # Generate text locations
    text_locations = []
    font_size = 2.5
    for i in range(len(lines)):
        text_locations.append((int(debug_window_width*0.01), int(font_size*14*(i+1))))

    # Draw strings on image
    for i in range(len(lines)):
        cv2.putText(img,lines[i],text_locations[i],cv2.FONT_HERSHEY_PLAIN,font_size,(0,0,255),3,2)
        
    return img


# Functions - Detection

def run_detector(img):

    # Run detector

    if TFLITE_EN:
        
        # Run detector
        frame_rgb = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb, (tflite_model_width, tflite_model_height))
        input_data = np.expand_dims(frame_resized, axis=0)

        # Normalize pixel values if using a floating model (i.e. if model is non-quantized)
        if floating_model:
            input_data = (np.float32(input_data) - input_mean) / input_std

        # Perform the actual detection by running the model with the image as input
        interpreter.set_tensor(input_details[0]['index'],input_data)
        interpreter.invoke()

        # Retrieve detection results
        boxes = interpreter.get_tensor(output_details[boxes_idx]['index'])[0] # Bounding box coordinates of detected objects
        classes = interpreter.get_tensor(output_details[classes_idx]['index'])[0] # Class index of detected objects
        scores = interpreter.get_tensor(output_details[scores_idx]['index'])[0] # Confidence of detected objects
        
        # Add one to classes to match MODEL
        for i in range(len(classes)):
            classes[i] = classes[i] + 1
        
        # Convert to NP
        boxes = np.array(boxes)
        classes = np.array(classes).astype(np.int64)
        scores = np.array(scores)

    else:

        # Run detector
        image_np = np.array(img.copy())
        input_tensor = tf.convert_to_tensor(image_np)
        input_tensor = input_tensor[tf.newaxis, ...]
        detections = detect_fn(input_tensor)
    
        # Normalize boxes
        boxes   = []
        for box in tf.get_static_value(detections[0][0]):
            boxes.append([box[0]/image_np.shape[0],
                          box[1]/image_np.shape[1],
                          box[2]/image_np.shape[0],
                          box[3]/image_np.shape[1]])  
        boxes = np.array(boxes)
        # Convert Classes to integers
        classes = tf.get_static_value(detections[2][0]).astype(np.int64)
        # Scores
        scores  = tf.get_static_value(detections[1][0])
        
    # Load in dict
    detections = {}
    detections['detection_boxes'] = boxes
    detections['detection_scores'] = scores
    detections['detection_classes'] = classes

    return detections

def run_detector_area(img, area):

    # Run detection
    detections = run_detector(img)
    
    # Only check a limited number of boxes
    global DET_MAX_PROC
    if DET_MAX_PROC == -1:
        loop_range = range(len(detections['detection_boxes']))
    else:
        loop_range = range(DET_MAX_PROC)
    
    # Remove detections on boundary
    if BOUNDARY_DETECTIONS_ON == True:
        for j in loop_range:
            if detections['detection_boxes'][j][0] <= BOUNDARY_SENSITIVITY:
                detections['detection_scores'][j] = 0.0
            if detections['detection_boxes'][j][1] <= BOUNDARY_SENSITIVITY:
                detections['detection_scores'][j] = 0.0
            if detections['detection_boxes'][j][2] >= (1.0 - BOUNDARY_SENSITIVITY):
                detections['detection_scores'][j] = 0.0
            if detections['detection_boxes'][j][3] >= (1.0 - BOUNDARY_SENSITIVITY):
                detections['detection_scores'][j] = 0.0
  
    # Scale detection boxes
    for j in loop_range: 
        detections['detection_boxes'][j][0] = (detections['detection_boxes'][j][0] - 0.5) * area[1] + 0.5 + area[3]
        detections['detection_boxes'][j][1] = (detections['detection_boxes'][j][1] - 0.5) * area[0] + 0.5 + area[2]
        detections['detection_boxes'][j][2] = (detections['detection_boxes'][j][2] - 0.5) * area[1] + 0.5 + area[3]
        detections['detection_boxes'][j][3] = (detections['detection_boxes'][j][3] - 0.5) * area[0] + 0.5 + area[2]
        
    # Check if any OK detections
    det_ok = 0
    for j in loop_range:
        if detections['detection_scores'][j] >= MIN_DECTETION_SCORE:
            det_ok = det_ok + 1
            
    return detections, det_ok

def calculate_detection_areas():
    print('Detection areas:')
    global DETECTION_AREA_IN
    global DETECTION_AREA
    global DET_AREA_COORD
    global cam_window_width
    global cam_window_height
    # Add and W zoom as product of webcam ratio
    for area in DETECTION_AREA_IN:
        DETECTION_AREA.append([area[0]*(cam_window_height/cam_window_width),area[0],area[1],area[2]])
    i = 0
    # Now, calculate coordinates
    for area in DETECTION_AREA:
        dim_w = int(cam_window_width*area[0])
        dim_h = int(cam_window_height*area[1])
        a = int(cam_window_width* (0.5-area[0]/2+area[2]))
        b = int(cam_window_width* (0.5+area[0]/2+area[2]))
        c = int(cam_window_height*(0.5-area[1]/2+area[3]))
        d = int(cam_window_height*(0.5+area[1]/2+area[3])) 
        DET_AREA_COORD.append([dim_w, dim_h , a, b, c, d])
        # Print coodinates for detection areas wiht its dimmentions
        print(str(i+1)+': '+str(dim_w)+'x'+str(dim_h)+' '+str(a)+','+str(b)+','+str(c)+','+str(d))
        i=i+1


# Functions - HEAD

def head_init(serialportin):
    global HEAD_SEE_COM_PORTS
    global head_ready       
    global head_logic_steps 
    global head_step_index  
    global head_step_recv
    global head_distance  
    global serialPort
    global head_write_en
    head_write_en = 1
    # See available serial ports
    if HEAD_SEE_COM_PORTS == 1:
        import serial.tools.list_ports as port_list
        ports = list(port_list.comports())
        head_print_info('Available Serial COM Ports:')
        for p in ports:
            head_print_info(str(p))
    # Try close serial port if its open
    try:
        serialPort.close()
    except NameError:
        head_print_info("SerialPort not opened yet")
    serialPort = serial.Serial(port=serialportin, baudrate=115200, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
    #serialPort = serial.Serial(port=serialportin, baudrate=115200, timeout=None)
    #self.port = serial.Serial(port_s, baudrate=9600, timeout=None)	
    
    # Wait until Ready and capture logical steps
    # (Programmed in arduino)
    head_ready       = 0
    head_logic_steps = 0
    head_step_index  = 0
    head_step_recv   = 1
    head_distance    = 0
    head_print_info('Waiting for HEAD to be up and running...')
    head_wait_for('ready')
    
def head_read():
    
    global head_ready
    global head_logic_steps
    global head_step_index
    global head_step_recv
    global head_distance
    
    # Read data out of the buffer until a carraige return / new line is found
    serialString = serialPort.readline()
    serialString = serialString.decode("Ascii")
      
    # Echo all
    serialString = serialString.strip()
    head_print_info(serialString)
        
    # Split string to find keywords
    serialStringSplit = serialString.split(" ")
    
    # Check for status and store
    if serialStringSplit[0] == 'ready':
        head_ready = 1
    if "STEPS_LOGIC:" == serialStringSplit[0]:
        head_logic_steps = int(serialStringSplit[1])
        head_step_index  = int(serialStringSplit[1])/2 # Starts at 180 degrees
    if "Step:" == serialStringSplit[0]:    
        head_step_index = int(serialStringSplit[1])
        head_step_recv = 1
    if "Dist:" == serialStringSplit[0]:       
        head_distance = int(serialStringSplit[1])

    return serialStringSplit

def head_read_all():
    global serialPort
    while serialPort.in_waiting > 0:
        head_read()

def head_wait_for(txt):
    while 1:
        serialStringSplit = head_read()
        if txt == serialStringSplit[0]:
            return serialStringSplit
        time.sleep(0.01)

def head_write_int(val):
    global serialPort
    head_read_all()
    serialPort.write(b'%d' % (val))
    serialPort.write(b'\n')

def head_print_info(txt):
    
    # This function should be able to print following
    # - In terminal
    # - On display
    # - Dump in file
    
    # Add header
    txt_dump = 'Head: '+txt

    # Dumb state info in log
    if EN_HEAD_PRINT_LOG == 1:
        print('ENABLE_STATE_LOG is not implemented')
        
    # Dumb state info in terminal
    if EN_HEAD_PRINT_TERMINAL == 1:
        print(txt_dump)
        
    # Dumb state into in window
    if EN_HEAD_PRINT_WINDOW == 1:
        PRINT_HEAD_INFO_WINDOW.insert(0,txt_dump)
        if len(PRINT_HEAD_INFO_WINDOW) > 50:
            PRINT_HEAD_INFO_WINDOW.pop(50)


# Functions - Time Measurement

def init_time():
    global cycle_time_start        
    global cycle_time_total        
    global cycle_time_avg_arr_size 
    global cycle_time_avg_arr      
    global cycle_time_avg          
    cycle_time_start        = process_time_ns()
    cycle_time_total        = 0
    cycle_time_avg_arr_size = 10
    cycle_time_avg_arr      = [700] * cycle_time_avg_arr_size
    cycle_time_avg          = 700

def take_time(en_print):
    global cycle_time_total
    global cycle_time_start
    global cycle_time_avg_arr
    global cycle_time_avg   
    cycle_time_total = round((process_time_ns() - cycle_time_start) / 1000000)
    cycle_time_start = process_time_ns()
    cycle_time_avg_arr.insert(0,cycle_time_total)
    cycle_time_avg_arr.pop(cycle_time_avg_arr_size)
    cycle_time_avg     = statistics.mean(cycle_time_avg_arr)
    if en_print == 1:
        print('cycle_time_total: '+str(cycle_time_total))
        print('cycle_time_avg:   '+str(cycle_time_avg))


# Functions - Keyboard Input

def keyboard_command(wait_key_in):
    global cap
    global HEAD_EN
    global head_logic_steps
    # Stop
    if wait_key_in == ord('q'):
        cap.release()
        cv2.destroyAllWindows()
        print('Exit print')
        return 0
    # If HEAD is connected
    if HEAD_EN == 1:
        # Stop/Start wrinting head
        global head_write_en
        if wait_key_in == ord('s'):
            if head_write_en == 1:
                head_write_en = 0
            else:
                head_write_en = 1
        # Rotate Right
        if wait_key_in == ord('a'):
            head_write_int(int(head_logic_steps/32))
            #head_wait_for('Step:')
        # Rotate Left
        if wait_key_in == ord('d'):
            head_write_int(-1*int(head_logic_steps/32))
            #head_wait_for('Step:')
            
    return 1

# Main Loop

# Set detection areas
# (Zoom,X,Y)

#DETECTION_AREA_IN = [[1.00,0.0,0.0]]

DETECTION_AREA_IN = [[1.00,0.0,0.0],
                     [0.75,0.0,0.00],
                     [0.50,0.0,-0.15],[0.50,0.0,0.00],[0.50,0.0,0.15]]

#DETECTION_AREA_IN = [[1.00,0.0,0.0],
#                  [0.75,0.0,-0.12],[0.75,0.0,0.00],[0.75,0.0,0.12],
#                  [0.50,0.0,-0.12],[0.50,0.0,0.00],[0.50,0.0,0.12],
#                  [0.25,0.0,-0.24],[0.25,0.0,-0.12],[0.25,0.0,0.00],[0.25,0.0,0.12],[0.25,0.0,0.24]]


#DETECTION_AREA = [[1.0,0.5,0.5], [0.75,0.5,0.5], [0.5,0.5,0.5], [0.25,0.5,0.5]]
#DETECTION_AREA = [[1.0,0.5,0.5], [0.5,0.5,0.5], [0.25,0.5,0.5]]
#DETECTION_AREA = [[0.25,0.5,0.5]]
#DETECTION_AREA = [[0.75,0.5,0.5]]
#DETECTION_AREA = [[1.0,0.5,0.5]]

# Dumb state info in log
EN_STATE_PRINT_LOG      = 0
# Dumb state info in terminal
EN_STATE_PRINT_TERMINAL = 1
# Dubm state ingo in window
EN_STATE_PRINT_WINDOW   = 0

# Dumb head info in log
EN_HEAD_PRINT_LOG      = 0
# Dumb head info in terminal
EN_HEAD_PRINT_TERMINAL = 1
# Dumb head ingo in window
EN_HEAD_PRINT_WINDOW   = 0


# Camera Select
CAM_SELECT = 0 # Built-in webcam
#CAM_SELECT = 1 # External webcam

# Set size of debug video window
#DEBUG_WINDOW_WIDTH_SET  = 640
DEBUG_WINDOW_WIDTH_SET  = 800
#DEBUG_WINDOW_WIDTH_SET  = 1280
#DEBUG_WINDOW_WIDTH_SET  = 1600
DEBUG_WINDOW_MAX_EN     = 0

# Enable connection to HEAD
# (Arduino controlling stepper mouted with webcam and proximity sensor)
HEAD_EN = 1
HEAD_SEE_COM_PORTS = 1

# Number of good detected frames in order to enter TRACKING state
GOOD_FRAMES_DETECTED = 3
global good_frames_detected_arr
# Number of bad frames in order to exit SCANNING state
BAD_FRAMES_DETECTED = 10
global bad_frames_detected_arr

SCANNING = 0
TRACKING = 1
SCANNING_EN = 1
TRACKING_EN = 1
current_state = SCANNING
tracking_area = 0

def init_good_frames_detected():
    global good_frames_detected_arr
    good_frames_detected_arr = []
    global DETECTION_AREA_IN
    for i in DETECTION_AREA_IN:
        tmp_arr = []
        for j in range(GOOD_FRAMES_DETECTED):
            tmp_arr.append(0)
        good_frames_detected_arr.append(tmp_arr)

def init_bad_frames_detected():
    global bad_frames_detected_arr
    bad_frames_detected_arr = []
    for i in range(BAD_FRAMES_DETECTED):
        bad_frames_detected_arr.append(0)



# Start video capture
cap = cv2.VideoCapture(CAM_SELECT)
# Set maximum dimmension
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(3, 10000)
cap.set(4, 10000)
# Check actual dimmension 
cam_window_width  = int(cap.get(3))
cam_window_height = int(cap.get(4))
# Enable real-time
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    
print('cam_window_width:  '+str(cam_window_width))
print('cam_window_height: '+str(cam_window_height))

# Init scanning parameters
init_good_frames_detected()
init_bad_frames_detected()

# States
current_state = SCANNING
tracking_area = 0

# Variables for calculating mean and variance 
global control_state_mean_x           
global control_state_var_x            
global control_state_mean_y           
global control_state_var_y            
control_state_sample_time      = 5000
control_state_sample_ticks_min = 10
control_state_sample_ticks_max = 500
control_state_sample_ticks     = control_state_sample_ticks_min
control_state_sample_arr_x     = []
control_state_mean_x           = -1
control_state_var_x            = -1
control_state_sample_arr_y     = []
control_state_mean_y           = -1
control_state_var_y            = -1

# Calculate detection Area coordinates
# (width=height)
DET_AREA_COORD = []
DETECTION_AREA = []
calculate_detection_areas()

# Set dimmentions of debug winddow
if DEBUG_WINDOW_MAX_EN == 0:
    debug_window_height = int(round(cam_window_height/cam_window_width*DEBUG_WINDOW_WIDTH_SET))
    debug_window_width  = DEBUG_WINDOW_WIDTH_SET
    debug_window_dim    = (debug_window_width,debug_window_height)
else: 
    debug_window_height = cam_window_height
    debug_window_width  = cam_window_width
    debug_window_dim    = (debug_window_width,debug_window_height)
print('debug_window_width:  '+str(debug_window_width))
print('debug_window_height: '+str(debug_window_height))
    
# Init Cycle time
cycle_time_total = 0
cycle_time_avg   = 0
init_time()

# Clear logs
PRINT_STATE_INFO_WINDOW = ['Start of state log']
PRINT_HEAD_INFO_WINDOW  = ['Start of head log']

# Enable HEAD
if HEAD_EN == 1:    
    global head_ready       
    global head_logic_steps 
    global head_step_index  
    global head_step_recv 
    global head_distance
    global head_write_en
    if os.name == 'posix':
        head_init("/dev/ttyUSB0")
    else:
        head_init("COM5")
    global serialPort
    
# Do one detections for all areas
detections_index  = 0
detections_number = 0
detections_list = []
i = 0
for area in DET_AREA_COORD:
    
    # Append to list
    detections_list.append([])
    
    # Get frame
    ret, frame = cap.read()
    image_np = np.array(frame)

    # Crop input image
    area_coord = DET_AREA_COORD[i]
    image_np_copy = image_np.copy()
    image_np_crop = image_np_copy[area_coord[4]:area_coord[5],area_coord[2]:area_coord[3]]

    # Only run one detection per frame
    area = DETECTION_AREA[i]
    detections_list[i], num_of_det = run_detector_area(image_np_crop, area)
    
    i = i + 1
    
# Run detection loop 
while True: 
    
    # Take time
    take_time(0)
    
    # Service HEAD
    if HEAD_EN == 1:   
        head_read_all()
    
    # When tracking only look at one area
    if current_state == TRACKING and TRACKING_EN:
        detections_index = tracking_area
    
    # Get frame
    ret, frame = cap.read()
    image_np = np.array(frame)

    # Crop input image
    area_coord = DET_AREA_COORD[detections_index]
    image_np_copy = image_np.copy()
    image_np_crop = image_np_copy[area_coord[4]:area_coord[5],area_coord[2]:area_coord[3]]

    # Only run one detection per frame
    area = DETECTION_AREA[detections_index]
    detections_list[detections_index], num_of_det = run_detector_area(image_np_crop, area)
    
    # Make new single entry detections list for tracking
    if current_state == TRACKING and TRACKING_EN:
        detection_list_tmp = []
        detection_list_tmp.append(detections_list[detections_index])
        detections_index_start = detections_index
    if current_state == SCANNING and SCANNING_EN:
        detection_list_tmp = detections_list
        detections_index_start = 0
    
    # Add if any good detections
    if current_state == SCANNING:
        good_frames_detected_arr[detections_index].pop(0)
        if num_of_det > 0:
            good_frames_detected_arr[detections_index].append(1)
        else:
            good_frames_detected_arr[detections_index].append(0)
    
    # Find strongest detection
    found_max, max_score, i_tmp, j_tmp = find_strongest_detection(detection_list_tmp, 0)
    
    # Send rotation to head
    if found_max and max_score >= MIN_DECTETION_SCORE:
        #if head_step_recv == 1 and head_write_en == 1:
        if head_write_en == 1:
            head_step_recv = 0
            detections_tmp = detection_list_tmp[i_tmp]
            size_x = detections_tmp['detection_boxes'][j_tmp][3] - detections_tmp['detection_boxes'][j_tmp][1]
            center_x = detections_tmp['detection_boxes'][j_tmp][1] + size_x/2
            center_x = center_x-0.5
            #center_x = int(center_x * head_logic_steps/8)
            center_x = int(center_x * head_logic_steps/12)
            #center_x = int(center_x * head_logic_steps/16)
            print("center_x: "+str(center_x))
            head_write_int(center_x)
            
    
    # Resize for debug window
    image_np = cv2.resize(image_np,debug_window_dim)

    # Draw all detections areas 
    if DRAW_ALL_DET_AREA:
        if current_state == SCANNING and SCANNING_EN:
            draw_detection_areas(DET_AREA_COORD, 0, image_np)
        if current_state == TRACKING and TRACKING_EN:
            DET_AREA_COORD_TMP = []
            DET_AREA_COORD_TMP.append(DET_AREA_COORD[detections_index])
            draw_detection_areas(DET_AREA_COORD_TMP, detections_index, image_np)
        
    # Draw all detections 
    if DRAW_ALL_DET:
        draw_detections_list(detection_list_tmp, detections_index_start, image_np) 
    
    # Draw strongest detection  
    #found_max, max_score, i_tmp, j_tmp = find_strongest_detection(detection_list_tmp, 0)
    if found_max and DRAW_STRONGEST_DET:
        draw_strongest_detection(detection_list_tmp, max_score, i_tmp, j_tmp, image_np, '#0000ff', detections_index_start)
    
    # Count bad frames in TRACKING
    if current_state == TRACKING:
        bad_frames_detected_arr.pop(0)
        if found_max and max_score >= MIN_DECTETION_SCORE:
            bad_frames_detected_arr.append(0)
        else:
            bad_frames_detected_arr.append(1)
            
            
    # Calculate Mean X,Y and Var X,Y
    if current_state == TRACKING:
        # Add distance X and Y to array
        detections_tmp = detection_list_tmp[i_tmp]
        size_x = detections_tmp['detection_boxes'][j_tmp][3] - detections_tmp['detection_boxes'][j_tmp][1]
        size_y = detections_tmp['detection_boxes'][j_tmp][2] - detections_tmp['detection_boxes'][j_tmp][0]
        center_x = detections_tmp['detection_boxes'][j_tmp][1] + size_x/2
        center_y = detections_tmp['detection_boxes'][j_tmp][0] + size_y/2
        control_state_sample_arr_x.insert(0,center_x-0.5)
        control_state_sample_arr_y.insert(0,center_y-0.5)
        # Trim arrays
        if len(control_state_sample_arr_x) > control_state_sample_ticks_max:
            control_state_sample_arr_x.pop(control_state_sample_ticks_max)
            control_state_sample_arr_y.pop(control_state_sample_ticks_max)
        # Calc how many frames in sample period
        control_state_sample_ticks_local = round(control_state_sample_time/cycle_time_avg)
        if control_state_sample_ticks_local < control_state_sample_ticks_min:
            control_state_sample_ticks = control_state_sample_ticks_min
        elif control_state_sample_ticks_local > control_state_sample_ticks_max:
            control_state_sample_ticks = control_state_sample_ticks_max
        else:
            control_state_sample_ticks = control_state_sample_ticks_local
        # When size is larget enough do mean and variance
        if len(control_state_sample_arr_x) > control_state_sample_ticks_min:
            # Take sub array
            control_state_sample_arr_x_sub = control_state_sample_arr_x[0:control_state_sample_ticks_min-1]
            control_state_sample_arr_y_sub = control_state_sample_arr_y[0:control_state_sample_ticks_min-1]
            # Do mean and variance
            control_state_var_x  = statistics.pvariance(control_state_sample_arr_x_sub)
            control_state_mean_x = statistics.mean(control_state_sample_arr_x_sub)
            control_state_var_y  = statistics.pvariance(control_state_sample_arr_y_sub)
            control_state_mean_y = statistics.mean(control_state_sample_arr_y_sub)
        else:
            control_state_mean_x           = -1
            control_state_var_x            = -1
            control_state_mean_y           = -1
            control_state_var_y            = -1    
       
    # Draw general status 
    if DRAW_STATUS:
        draw_general_status(image_np) 
            
    # Show
    cv2.imshow('object detection',image_np)
 
    # Read keyboard
    wait_key_in = cv2.waitKey(10) & 0xFF
    if keyboard_command(wait_key_in) == 0:
        break;
        
    # Check if SCANNING should be entered
    if current_state == TRACKING:
        if sum(bad_frames_detected_arr) == len(bad_frames_detected_arr):
            print('Go to scanning')
            current_state = SCANNING
            init_good_frames_detected()
            # Do detection on the first area next
            detections_index = 0 - 1
            # Clear mean and variance calculations
            control_state_sample_ticks     = control_state_sample_ticks_min
            control_state_sample_arr_x     = []
            control_state_mean_x           = -1
            control_state_var_x            = -1
            control_state_sample_arr_y     = []
            control_state_mean_y           = -1
            control_state_var_y            = -1
                     
    # Increment counters
    detections_number = detections_number + 1
    detections_index  = detections_index + 1
    if len(DET_AREA_COORD) == detections_index:
        detections_index = 0
        
        # Check that strongest score also has privious detections in same detection area
        if max_score >= MIN_DECTETION_SCORE and current_state != TRACKING and TRACKING_EN:
            good_frms_sum = 0
            for good_frms_index in range(GOOD_FRAMES_DETECTED):
                good_frms_sum = good_frms_sum + good_frames_detected_arr[i_tmp][good_frms_index] 
            if good_frms_sum == GOOD_FRAMES_DETECTED:
                print('Go to tracking area: a'+str(i_tmp))
                current_state = TRACKING
                tracking_area = i_tmp
                init_bad_frames_detected()
        
#### End of loop ####


#TODO:
# - Make sure step is not send when HEAD is not done rotating


## Init video recording
#video_out = cv2.VideoWriter('output_video.avi',cv2.VideoWriter_fourcc(*'DIVX'), 5, debug_window_dim)
## Video
#video_out.write(image_np)
##
#video_out.release()




