import cv2
import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2 as cv
import time
import imutils
from decimal import Decimal, ROUND_HALF_UP
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 30)
# pipeline_wrapper = rs.pipeline_wrapper(pipeline)
# pipeline_profile = config.resolve(pipeline_wrapper)
profile = pipeline.start(config)
intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
alpha = 0
prev_time = 0
frame100 = 0
framesum = 0
while True:
    frames = pipeline.wait_for_frames()
    align_to = rs.stream.color
    align = rs.align(align_to)
    aligned_frames = align.process(frames)
    color_frame = aligned_frames.get_color_frame()
    depth_frame = aligned_frames.get_depth_frame()
    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())
    if not depth_frame or not color_frame:
        continue
    blurred = cv.GaussianBlur(color_image, (11,11), 0)
    hsv = cv.cvtColor(blurred, cv.COLOR_BGR2HSV)
    colorLower = np.array([84, 77, 69])
    colorUpper = np.array([255, 255, 255])
    mask = cv.inRange(hsv, colorLower, colorUpper)
    mask = cv.erode(mask, None, iterations=2)
    mask = cv.dilate(mask, None, iterations=2)
    cnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    if len(cnts) > 0:
        c = max(cnts, key=cv.contourArea)
        ((x, y), radius) = cv.minEnclosingCircle(c)
        M = cv.moments(c)
        chosen = (int(M["m10"]/ M["m00"]), int(M["m01"] / M["m00"]))
        cv.circle(color_image, (chosen[0], chosen[1]), 1, (255,255,0), 3)
        cv.circle(color_image, (chosen[0], chosen[1]), int(radius), (255, 0, 255), 3)
        #for i in range(len(cnts)):
        distance = depth_frame.get_distance(int(chosen[0]), int(chosen[1]))*100
        xaxis = distance * (chosen[0]-intr.ppx)/intr.fx
        yaxis = distance * (chosen[1]-intr.ppy)/intr.fy
        zaxis = distance
        # xtilted= xaxis-#tilted angle
        # ytilted = -(zaxis*math.sin(alpha)+yaxis * math.cos(alpha))
        # ztilted = zaxis* math.cos(alpha)+yaxis*math.sin(alpha)
        coordinate_text = "("  + str(Decimal(str(xaxis)).quantize(Decimal('0'), rounding=ROUND_HALF_UP)) + \
                         ", " + str(Decimal(str(yaxis)).quantize(Decimal('0'), rounding=ROUND_HALF_UP)) + \
                         ", " +str(Decimal(str(zaxis)).quantize(Decimal('0'),rounding=ROUND_HALF_UP)) + ")"
        cv.putText(color_image, "  {}cm".format(int(distance)), (chosen[0], chosen[1]), cv.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1)
        cv.putText(color_image, text= coordinate_text, org=(chosen[0],chosen[1]), fontFace=cv.FONT_HERSHEY_PLAIN, fontScale= 4, color=(0,0,255), thickness=2, lineType=cv2.LINE_AA)
        currenttime =time.time()
        elapsed_time = currenttime - prev_time
        prev_time  = currenttime
        fps = int(1 / elapsed_time)
        print("x, y pixel : " + str((chosen[0], chosen[1]))+ "          x y z in 3d:" +coordinate_text+ \
              "         fps:", fps, "    ", elapsed_time)
        # frame100 += 1
        # framesum = framesum + fps
        # if frame100 == 100:
        # print(framesum / 100)
    cv.imshow("normal", color_image)
    cv.imshow("changed", mask)
    if cv.waitKey(1) & 0xFF == ord('q'): break

cv.destroyAllWindows()

