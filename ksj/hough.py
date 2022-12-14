import cv2
import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2 as cv
import time
from decimal import Decimal, ROUND_HALF_UP

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
# pipeline_wrapper = rs.pipeline_wrapper(pipeline)
# pipeline_profile = config.resolve(pipeline_wrapper)
profile = pipeline.start(config)
intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

prevCircle = None
alpha = 0
dist = lambda x1,y1,x2, y2: (x1-x2)**2+(y1-y2)**2
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
    if not depth_frame or not color_frame:
        continue


    color_image = np.asanyarray(color_frame.get_data())
    depth_image = np.asanyarray(depth_frame.get_data())

    grayscale = cv.cvtColor(color_image, cv.COLOR_BGR2GRAY)
    blurFrame = cv.GaussianBlur(grayscale, (17,17), cv.BORDER_DEFAULT)

    circles = cv.HoughCircles(blurFrame, cv.HOUGH_GRADIENT, 1, 1000, param1=100, param2=15, minRadius=5, maxRadius=100)
    if circles is not None:
        circles = np.uint16(np.around(circles))
        chosen = None
        for i in circles[0, :]:
            if chosen is None: chosen = i
            if prevCircle is not None:
                if dist(chosen[0], chosen[0], prevCircle[0], prevCircle[1] <= dist(i[0],i[1], prevCircle[0], prevCircle[1])):
                    chosen = i
        prevCircle= chosen

        # cv.circle(color_image, (chosen[0], chosen[1]), 1, (255, 255, 0), 3)
        # cv.circle(color_image, (chosen[0], chosen[1]), chosen[2], (255, 0, 255), 3)
        # if(len(chosen)>0):
        #     for i in range(len(chosen)):
        distance = depth_frame.get_distance(int(chosen[0]), int(chosen[1]))*100
        xaxis = distance * (chosen[0]-intr.ppx)/intr.fx
        yaxis = distance * (chosen[1]-intr.ppy)/intr.fy
        zaxis = distance

        # xtilted= xaxis-#tilted angle
        # ytilted = -(zaxis*math.sin(alpha)+yaxis * math.cos(alpha))
        # ztilted = zaxis* math.cos(alpha)+yaxis*math.sin(alpha)
        #
        coordinate_text = "("  + str(Decimal(str(xaxis)).quantize(Decimal('0'), rounding=ROUND_HALF_UP)) + \
                         ", " + str(Decimal(str(yaxis)).quantize(Decimal('0'), rounding=ROUND_HALF_UP)) + \
                         ", " +str(Decimal(str(zaxis)).quantize(Decimal('0'),rounding=ROUND_HALF_UP)) + ")"
        # cv.putText(color_image, "  {}cm".format(int(distance)), (chosen[0], chosen[1]), cv.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1)
        # cv.putText(color_image, text= coordinate_text, org=(chosen[0],chosen[1]), fontFace=cv.FONT_HERSHEY_PLAIN, fontScale= 1, color=(0,0,255), thickness=2, lineType=cv2.LINE_AA)
        print("x, y pixel : " + str((chosen[0], chosen[1])) + "          x y z in 3d:" + coordinate_text )
    currenttime = time.time()
    elapsed_time = currenttime - prev_time
    prev_time = currenttime
    fps = int(1 / elapsed_time)
    # frame100 += 1
    # framesum = framesum + fps
    # if frame100==100:
    #     print(framesum/100)
    print(elapsed_time, "    ",fps)

    # cv.imshow("circles", color_image)
    if cv.waitKey(1) & 0xFF == ord('q'): break

cv.destroyAllWindows()

