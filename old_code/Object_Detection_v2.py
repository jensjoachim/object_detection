#!/usr/bin/env python
# coding: utf-8

# # Object Detection v1
# Object detection using a general model
# 
# Arduino controlling stepper motor with webcam and prximity sensor

# In[4]:


#@title Imports and function definitions

# For running inference on the TF-Hub module.
import tensorflow as tf
import tensorflow_hub as hub

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

# Print Tensorflow version
print(tf.__version__)

# Check available GPU devices.
print("The following GPU devices are available: %s" % tf.test.gpu_device_name())


# In[5]:


#import cv2 
#import numpy as np
#from matplotlib import pyplot as plt
import matplotlib.pyplot as plt
get_ipython().run_line_magic('matplotlib', 'inline')


# In[6]:


#from time import process_time_ns
import statistics


# In[ ]:





# In[16]:


#model = tf.saved_model.load('../../tflite_custom_models/25_07_2022__20_41_efficientdet_lite0/saved_model/')


# ### Model Selection

# In[2]:


# "https://tfhub.dev/google/faster_rcnn/openimages_v4/inception_resnet_v2/1" 
# "https://tfhub.dev/google/openimages_v4/ssd/mobilenet_v2/1"
# "https://tfhub.dev/google/faster_rcnn/openimages_v4/inception_resnet_v2/1"
#module_handle = "https://tfhub.dev/google/openimages_v4/ssd/mobilenet_v2/1"
module_handle = '../../object_detection_model'
detector = hub.load(module_handle).signatures['default']


# In[ ]:





# ### Functions - Drawing

# In[5]:


# Set general font
try:
    font = ImageFont.truetype("C:/Windows/Fonts/Arial/ariblk.ttf",25)
except IOError:
    print("Font not found, using default font.")
    font = ImageFont.load_default() 
    exit()


# In[6]:


def draw_boxes(image, index, boxes, class_names, scores, max_boxes=10, min_score=0.1):
    global font
    """Overlay labeled boxes on an image with formatted scores and label names."""
    colors = list(ImageColor.colormap.values())
    image_pil = Image.fromarray(np.uint8(image)).convert("RGB")
    for i in range(min(boxes.shape[0], max_boxes)):
        if scores[i] >= min_score:
            ymin, xmin, ymax, xmax = tuple(boxes[i])
            display_str = "a{} {}: {}%".format(int(index),class_names[i].decode("ascii"),int(100 * scores[i]))
            color = colors[sum(class_names[i]) % len(colors)]
            image_pil = Image.fromarray(np.uint8(image)).convert("RGB")
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


# In[7]:


def draw_bounding_box_on_image(
    image,
    ymin,
    xmin,
    ymax,
    xmax,
    color,
    font,
    thickness=4,
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


# In[8]:


def draw_detections_list(detections_list, start_index, img):
    i = start_index
    for detections in detections_list:
        image_with_boxes = draw_boxes(
            img, 
            i,
            detections["detection_boxes"],
            detections["detection_class_entities"], 
            detections["detection_scores"],
            max_boxes=10, 
            min_score=0.3)
        i = i + 1

    img = image_with_boxes
    return img


# In[9]:


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


# In[10]:


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
    global mean_x 
    global var_x
    str_mean_x           = "Mean X:   "
    str_var_x            = "Var X:     "
    str_mean_x = str_mean_x+"Na"
    str_var_x  = str_var_x+"Na"
    
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
    font_size = 1.2
    for i in range(len(lines)):
        text_locations.append((int(debug_window_width*0.01), int(font_size*45*(i+1))))

    # Draw strings on image
    for i in range(len(lines)):
        cv2.putText(img,lines[i],text_locations[i],cv2.FONT_HERSHEY_SIMPLEX,font_size,(0,0,255),3,2)
        
    return img


# In[ ]:





# ### Functions - Detection

# In[11]:


def run_detector(detector, img):

    # Run detector
    converted_img  = tf.image.convert_image_dtype(img, tf.float32)[tf.newaxis, ...]
    result = detector(converted_img)
    detections = {key:value.numpy() for key,value in result.items()}

    return detections


# In[12]:


def run_detector_area(detector, img, area):
    
    # Run detection
    detections = run_detector(detector, img)
    
    # Scale detection boxes
    for j in range(len(detections['detection_boxes'])):
        detections['detection_boxes'][j][0] = detections['detection_boxes'][j][0] * area[0] + (1.0-area[0])*area[1]
        detections['detection_boxes'][j][1] = detections['detection_boxes'][j][1] * area[0] + (1.0-area[0])*area[1]
        detections['detection_boxes'][j][2] = detections['detection_boxes'][j][2] * area[0] + (1.0-area[0])*area[1]
        detections['detection_boxes'][j][3] = detections['detection_boxes'][j][3] * area[0] + (1.0-area[0])*area[1]
        
    # Add center point to detections (new item to dict)
    detections['detection_boxes_center'] = []
    for j in range(len(detections['detection_boxes'])):
        size_x = detections['detection_boxes'][j][3] - detections['detection_boxes'][j][1]
        size_y = detections['detection_boxes'][j][2] - detections['detection_boxes'][j][0]
        center_x = detections['detection_boxes'][j][1] + size_x/2
        center_y = detections['detection_boxes'][j][0] + size_y/2
        detections['detection_boxes_center'].append([center_x,center_y])
    
    return detections


# In[13]:


def calculate_detection_areas():
    print('Detection areas:')
    global DETECTION_AREA
    global DET_AREA_COORD
    global cam_window_width
    global cam_window_height
    i = 0
    for area in DETECTION_AREA:
        dim_w = int(cam_window_width*area[0])
        dim_h = int(cam_window_height*area[0])
        a = int(cam_window_width*(0.5-area[0]*area[1]))
        b = int(cam_window_width*(0.5+area[0]*area[1]))
        c = int(cam_window_height*(0.5-area[0]*area[2]))
        d = int(cam_window_height*(0.5+area[0]*area[2]))
        DET_AREA_COORD.append([dim_w, dim_h , a, b, c, d])
        # Print coodinates for detection areas wiht its dimmentions
        print(str(i+1)+': '+str(dim_w)+'x'+str(dim_h)+' '+str(a)+','+str(b)+','+str(c)+','+str(d))
        i=i+1


# In[ ]:





# ### Functions - HEAD

# In[14]:


def head_init(serialportin):
    global HEAD_SEE_COM_PORTS
    global head_ready       
    global head_logic_steps 
    global head_step_index  
    global head_distance  
    global serialPort
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
    # Wait until Ready and capture logical steps
    # (Programmed in arduino)
    head_ready       = 0
    head_logic_steps = 0
    head_step_index  = 0
    head_distance    = 0
    head_print_info('Waiting for HEAD to be up and running...')
    head_wait_for('ready')
    


# In[15]:


def head_read():
    
    global head_ready
    global head_logic_steps
    global head_step_index
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
    if "Dist:" == serialStringSplit[0]:       
        head_distance = int(serialStringSplit[1])

    return serialStringSplit


# In[16]:


def head_read_all():
    global serialPort
    while serialPort.in_waiting > 0:
        head_read()


# In[17]:


def head_wait_for(txt):
    while 1:
        serialStringSplit = head_read()
        if txt == serialStringSplit[0]:
            return serialStringSplit
        time.sleep(0.01)


# In[18]:


def head_write_int(val):
    global serialPort
    head_read_all()
    serialPort.write(b'%d' % (val))


# In[19]:


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


# In[ ]:





# ### Functions - Time Measurement

# In[20]:


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


# In[21]:


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


# In[ ]:





# ### Functions - Keyboard Input

# In[22]:


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
        # Rotate Right
        if wait_key_in == ord('a'):
            head_write_int(int(head_logic_steps/32))
            #head_wait_for('Step:')
        # Rotate Left
        if wait_key_in == ord('d'):
            head_write_int(-1*int(head_logic_steps/32))
            #head_wait_for('Step:')
            
    return 1


# In[ ]:





# ### Main Loop

# In[23]:


# Set detection areas
# (Zoom,X,Y)
DETECTION_AREA = [[1.0,0.5,0.5], [0.5,0.5,0.5], [0.25,0.5,0.5]]
#DETECTION_AREA = [[0.25,0.5,0.5]]
#DETECTION_AREA = [[0.75,0.5,0.5]]
#DETECTION_AREA = [[1.0,0.5,0.5]]


# In[24]:


# Dumb state info in log
EN_STATE_PRINT_LOG      = 0
# Dumb state info in terminal
EN_STATE_PRINT_TERMINAL = 0
# Dubm state ingo in window
EN_STATE_PRINT_WINDOW   = 0

# Dumb head info in log
EN_HEAD_PRINT_LOG      = 0
# Dumb head info in terminal
EN_HEAD_PRINT_TERMINAL = 0
# Dumb head ingo in window
EN_HEAD_PRINT_WINDOW   = 0


# In[25]:


# Camera Select
CAM_SELECT = 0 # Built-in webcam
#CAM_SELECT = 1 # External webcam

# Set size of debug video window
#DEBUG_WINDOW_WIDTH_SET  = 800
DEBUG_WINDOW_WIDTH_SET  = 1280
#DEBUG_WINDOW_WIDTH_SET  = 1600
DEBUG_WINDOW_MAX_EN     = 0

# Enable connection to HEAD
# (Arduino controlling stepper mouted with webcam and proximity sensor)
HEAD_EN = 0
HEAD_SEE_COM_PORTS = 1


# In[26]:


# Start video capture
cap = cv2.VideoCapture(CAM_SELECT)

# Set maximum dimmension
cap.set(3, 10000)
cap.set(4, 10000)
# Check actual dimmension 
cam_window_width  = int(cap.get(3))
cam_window_height = int(cap.get(4))
print('cam_window_width:  '+str(cam_window_width))
print('cam_window_height: '+str(cam_window_height))

# Calculate detection Area coordinates
# (width=height)
DET_AREA_COORD = []
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
    head_ready       = 0
    head_logic_steps = 0
    head_step_index  = 0
    head_distance    = 0
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
    detections_list[i] = run_detector_area(detector, image_np_crop, area)
    
    i = i + 1
    
# Run detection loop 
while cap.isOpened(): 
    
    # Take time
    take_time(0)
    
    # Get frame
    ret, frame = cap.read()
    image_np = np.array(frame)

    # Crop input image
    area_coord = DET_AREA_COORD[detections_index]
    image_np_copy = image_np.copy()
    image_np_crop = image_np_copy[area_coord[4]:area_coord[5],area_coord[2]:area_coord[3]]

    # Only run one detection per frame
    area = DETECTION_AREA[detections_index]
    detections_list[detections_index] = run_detector_area(detector, image_np_crop, area)
    
    # Service HEAD
    if HEAD_EN == 1:   
        head_read_all()
        
    # Draw all detections areas 
    draw_detection_areas(DET_AREA_COORD, 0, image_np)

    # Draw all detections      
    draw_detections_list(detections_list, 0, image_np)        
    
    # Draw general status 
    draw_general_status(image_np) 
            
    # Show
    cv2.imshow('object detection',  cv2.resize(
        image_np, 
        debug_window_dim))
    
    # Read keyboard
    wait_key_in = cv2.waitKey(10) & 0xFF
    if keyboard_command(wait_key_in) == 0:
        break;
                     
    # Increment counters
    detections_number = detections_number + 1
    detections_index  = detections_index + 1
    if len(DET_AREA_COORD) == detections_index:
        detections_index = 0
        
    #### End of loop ####
    


# In[ ]:





# In[27]:


#TODO:
# - add status: DEV, MEAN
# - Make sure step is not send when HEAD is not done rotating


# ### Old code

# In[28]:


print(detections_list[2])


# In[29]:


print(detections["detection_scores"])


# In[ ]:


print(detections_list[0]['detection_class_entities'])


# In[ ]:





# In[ ]:


print(detections["detection_boxes"])

