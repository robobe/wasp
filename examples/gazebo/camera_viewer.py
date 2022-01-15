import asyncio
import pygazebo
import cv2
import numpy as np
import time
import math
import threading

prev_frame_time = 0

def process(im):
    print(threading.current_thread().name)
    new_frame_time = time.time()
    global prev_frame_time
    fps = math.floor(1/(new_frame_time-prev_frame_time))
    prev_frame_time = new_frame_time
    cv2.putText(im, str(fps), (7, 70), cv2.FONT_HERSHEY_SIMPLEX, 3, (100, 255, 0), 3, cv2.LINE_AA)
    cv2.imshow("view", im)
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