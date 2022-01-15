import asyncio
import pygazebo
import cv2
import numpy as np
import time
import math
import threading

prev_frame_time = 0

lk_params = dict( winSize  = (15, 15), 
                  maxLevel = 2, 
                  criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))    

feature_params = dict( maxCorners = 500, 
                       qualityLevel = 0.3,
                       minDistance = 7,
                       blockSize = 7 )



def select_point(event, x, y, flags, params):
    global point, point_selected, old_points
    if event == cv2.EVENT_LBUTTONDOWN:
        point = (x, y)
        print(point)
        point_selected = True
        old_points = np.array([[x, y]], dtype=np.float32)

cv2.namedWindow("LK")
cv2.setMouseCallback("LK", select_point)
point_selected = False
point = ()
old_points = np.array([[]])
old_gray = None
def process(frame):
    global old_points, old_gray

    if old_gray is None:
        old_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    new_frame_time = time.time()
    global prev_frame_time
    fps = math.floor(1/(new_frame_time-prev_frame_time))
    prev_frame_time = new_frame_time
    # cv2.putText(im, str(fps), (7, 70), cv2.FONT_HERSHEY_SIMPLEX, 3, (100, 255, 0), 3, cv2.LINE_AA)
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if point_selected is True:
        cv2.circle(frame, point, 5, (0, 0, 255), 2)

        new_points, status, error = cv2.calcOpticalFlowPyrLK(old_gray, gray_frame, old_points, None, **lk_params)
        old_gray = gray_frame.copy()
        old_points = new_points

        x, y = new_points.ravel()
        print(x,y)
        cv2.circle(frame, (int(x), int(y)), 5, (0, 255, 0), -1)
    cv2.imshow("LK", frame)
    cv2.waitKey(1)

def cb(data):
    image_stamped = pygazebo.msg.image_stamped_pb2.ImageStamped()
    image_stamped.ParseFromString(data)
    img_msg = image_stamped.image
    channels = 3
    im = np.ndarray(shape=(img_msg.height, img_msg.width, channels),
                            dtype=np.uint8,
                            buffer=img_msg.data)
    loop = asyncio.get_event_loop()
    loop.run_in_executor(None, process, im)

async def publish_loop():
    manager = await pygazebo.connect()
    subscriber = await manager.subscribe("/gazebo/default/camera/link/camera/image", 
    'gazebo.msgs.ImageStamped', cb)
    while True:
        await asyncio.sleep(1)

loop = asyncio.get_event_loop()
loop.run_until_complete(publish_loop())