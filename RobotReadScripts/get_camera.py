## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#####################################################
##              Align Depth to Color               ##
#####################################################

# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
import time
import open3d as o3d
import matplotlib.pyplot as plt
import os, datetime


def open_pointCloud_from_rgb_and_depth(color_raw, depth_raw, meters_trunc=5, showImages = True):
    depth_scale=1/1000
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw, depth_scale=1/depth_scale, depth_trunc=meters_trunc, convert_rgb_to_intensity=False)
    if(showImages):
        plt.subplot(1, 2, 1)
        plt.title('RGB image')
        plt.imshow(rgbd_image.color)
        plt.subplot(1, 2, 2)
        plt.title('Depth image')
        plt.imshow(rgbd_image.depth)
        plt.show()

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image,o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
    # Flip it, otherwise the pointcloud will be upside down
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    return pcd


mydir = os.path.join(os.getcwd(), datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S'))
os.makedirs(mydir)

# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
clipping_distance_in_meters = 3 #1 meter
clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
align_to = rs.stream.color
align = rs.align(align_to)
c_image = 53
travou = False
# Streaming loop

vis = o3d.visualization.Visualizer()
vis.create_window('PCD', width=640, height=480)
geom_added = False
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector([])

exibe3d = False
grava_img = False

try:
    while True:
        key = cv2.waitKey(1)
        if key & 0xFF == ord('l'):
            if not travou:
                time.sleep(10)
                grava_img = True
                exibe3d = True
        frames = []
        for x in range(10):
            frameset = pipeline.wait_for_frames()

            # Align the depth frame to color frame
            aligned_frames = align.process(frameset)
            frames.append(aligned_frames.get_depth_frame())
        temporal = rs.temporal_filter()

        for x in range(10):
            frame = temporal.process(frames[x])

        # frameset = pipeline.wait_for_frames()
        # # Align the depth frame to color frame
        # aligned_frames = align.process(frameset)
        # frame = aligned_frames.get_depth_frame()

        # Get aligned frames
        aligned_depth_frame = frame # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            continue

        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        

        # Remove background - Set pixels further than clipping_distance to grey
        grey_color = 153
        depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
        bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)
        cv2.putText(bg_removed,"Cena "+str(c_image)+" travado: "+str((bool(travou))), (50,50), cv2.FONT_HERSHEY_SIMPLEX, 1, 255,5)
        # Render images:
        #   depth align to color on left
        #   depth on right
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        images = np.hstack((bg_removed, depth_colormap))


        cv2.namedWindow('Align Example', cv2.WINDOW_NORMAL)
        cv2.imshow('Align Example', images)

        # Press esc or 'q' to close the image window
        if key & 0xFF == ord('q') or key == 27:
            cv2.destroyAllWindows()
            vis.destroy_window()
            break

        if key & 0xFF == ord('a'):
            if not travou:
                grava_img = True




        if key & 0xFF == ord('t'):
            travou = False

        if key & 0xFF == ord('s'):
            exibe3d = True

        if grava_img:
            # depth_array = np.array(depth_image, dtype=np.float32)
            # Normalize the depth image to fall between 0 (black) and 1 (white)
            # cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
            cv2.imwrite(mydir+'/'+str(c_image)+'_depth.png',depth_image)
            cv2.imwrite(mydir+'/'+str(c_image)+'_rgb.png',color_image)
            c_image = c_image+1
            travou = True
            grava_img = False

        if exibe3d:
            vis.destroy_window()
            vis = o3d.visualization.Visualizer()
            vis.create_window('PCD', width=640, height=480)
            b,g,r = cv2.split(color_image)           # get b, g, r
            rgb_img1 = cv2.merge([r,g,b])     # switch it to r, g, b
            irgb = o3d.geometry.Image((rgb_img1))
            idepth = o3d.geometry.Image((depth_image))
            pcd = open_pointCloud_from_rgb_and_depth(
                    irgb, idepth, meters_trunc=3, showImages = False)
            vis.add_geometry(pcd) 
            exibe3d = False

        vis.poll_events()
        vis.update_renderer()
        #     color_raw = o3d.io.read_image(mydir+'/'+str(c_image-1)+'_rgb.png')
        #     depth_raw = o3d.io.read_image(mydir+'/'+str(c_image-1)+'_depth.png')
            



finally:
    pipeline.stop()