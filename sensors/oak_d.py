"""
Data collection using OAK-D.

Based on example https://docs.luxonis.com/projects/api/en/latest/samples/StereoDepth/stereo_depth_video/
"""
import os
from pathlib import Path
from typing import Optional
from datetime import datetime
import subprocess

import io
import cv2
from PIL import Image
import numpy as np
import depthai as dai

from PIL.ExifTags import TAGS
TAG2ENUM = {v:k for (k,v) in TAGS.items()}
TIME_TAG = TAG2ENUM['DateTime']

def du(path):
    """disk usage in human readable format (e.g. '2,1GB')"""
    return subprocess.check_output(['du','-sh', path]).split()[0].decode('utf-8')

def collect(folder: Path,
            resolution: Optional[str] = 720,
            mesh_dir: Optional[str] = None,
            load_mesh: Optional[bool] = False,
            out_rectified: Optional[bool] = False,
            lrcheck: Optional[bool] = False,
            extended: Optional[bool] = False,
            subpixel: Optional[bool] = False,
            depth: Optional[bool] = False,
            median: Optional[str] = '7x7'):
    """
    Parameters:
        folder: Output folder to save images and mesh files.
        resolution: Sets the resolution on mono cameras. Options: 800 | 720 | 400.
        mesh_dir: Output directory for mesh files. If not specified mesh files won't be saved
        load_mesh: Read camera intrinsics, generate mesh files and load them into the stereo node.
        out_rectified: Generate and display rectified streams
        lrcheck: Better handling for occlusions
        extended: Closer-in minimum depth, disparity range is doubled.
        subpixel: Better accuracy for longer distance, fractional disparity 32-levels
        depth: Display depth frames.
        median: Choose the size of median filtering. Options: OFF | 3x3 | 5x5 | 7x7 (default)
    """

    resolutionMap = {"800": (1280, 800), "720": (1280, 720), "400": (640, 400)}
    if resolution not in resolutionMap:
        exit("Unsupported resolution!")

    resolution = resolutionMap[resolution]
    meshDirectory = mesh_dir  # Output dir for mesh files
    generateMesh = load_mesh  # Load mesh files

    if meshDirectory is None:
        meshDirectory = str(folder / "undistort_rectify_maps")
        os.makedirs(meshDirectory)

    outRectified = out_rectified  # Output and display rectified streams
    lrcheck = lrcheck  # Better handling for occlusions
    extended = extended  # Closer-in minimum depth, disparity range is doubled
    subpixel = subpixel  # Better accuracy for longer distance, fractional disparity 32-levels
    depth = depth  # Display depth frames

    medianMap = {
        "OFF": dai.StereoDepthProperties.MedianFilter.MEDIAN_OFF,
        "3x3": dai.StereoDepthProperties.MedianFilter.KERNEL_3x3,
        "5x5": dai.StereoDepthProperties.MedianFilter.KERNEL_5x5,
        "7x7": dai.StereoDepthProperties.MedianFilter.KERNEL_7x7,
    }
    if median not in medianMap:
        exit("Unsupported median size!")

    median = medianMap[median]

    print("StereoDepth config options:")
    print("    Resolution:  ", resolution)
    print("    Left-Right check:  ", lrcheck)
    print("    Extended disparity:", extended)
    print("    Subpixel:          ", subpixel)
    print("    Median filtering:  ", median)
    print("    Generating mesh files:  ", generateMesh)
    print("    Outputting mesh files to:  ", meshDirectory)


    def getMesh(calibData):
        M1 = np.array(calibData.getCameraIntrinsics(dai.CameraBoardSocket.LEFT, resolution[0], resolution[1]))
        d1 = np.array(calibData.getDistortionCoefficients(dai.CameraBoardSocket.LEFT))
        R1 = np.array(calibData.getStereoLeftRectificationRotation())
        M2 = np.array(calibData.getCameraIntrinsics(dai.CameraBoardSocket.RIGHT, resolution[0], resolution[1]))
        d2 = np.array(calibData.getDistortionCoefficients(dai.CameraBoardSocket.RIGHT))
        R2 = np.array(calibData.getStereoRightRectificationRotation())
        mapXL, mapYL = cv2.initUndistortRectifyMap(M1, d1, R1, M2, resolution, cv2.CV_32FC1)
        mapXR, mapYR = cv2.initUndistortRectifyMap(M2, d2, R2, M2, resolution, cv2.CV_32FC1)

        meshCellSize = 16
        meshLeft = []
        meshRight = []

        for y in range(mapXL.shape[0] + 1):
            if y % meshCellSize == 0:
                rowLeft = []
                rowRight = []
                for x in range(mapXL.shape[1] + 1):
                    if x % meshCellSize == 0:
                        if y == mapXL.shape[0] and x == mapXL.shape[1]:
                            rowLeft.append(mapYL[y - 1, x - 1])
                            rowLeft.append(mapXL[y - 1, x - 1])
                            rowRight.append(mapYR[y - 1, x - 1])
                            rowRight.append(mapXR[y - 1, x - 1])
                        elif y == mapXL.shape[0]:
                            rowLeft.append(mapYL[y - 1, x])
                            rowLeft.append(mapXL[y - 1, x])
                            rowRight.append(mapYR[y - 1, x])
                            rowRight.append(mapXR[y - 1, x])
                        elif x == mapXL.shape[1]:
                            rowLeft.append(mapYL[y, x - 1])
                            rowLeft.append(mapXL[y, x - 1])
                            rowRight.append(mapYR[y, x - 1])
                            rowRight.append(mapXR[y, x - 1])
                        else:
                            rowLeft.append(mapYL[y, x])
                            rowLeft.append(mapXL[y, x])
                            rowRight.append(mapYR[y, x])
                            rowRight.append(mapXR[y, x])
                if (mapXL.shape[1] % meshCellSize) % 2 != 0:
                    rowLeft.append(0)
                    rowLeft.append(0)
                    rowRight.append(0)
                    rowRight.append(0)

                meshLeft.append(rowLeft)
                meshRight.append(rowRight)

        meshLeft = np.array(meshLeft)
        meshRight = np.array(meshRight)

        return meshLeft, meshRight


    def saveMeshFiles(meshLeft, meshRight, outputPath):
        print("Saving mesh to:", outputPath)
        meshLeft.tofile(outputPath + "/left_mesh.calib")
        meshRight.tofile(outputPath + "/right_mesh.calib")


    def applyDisparityColorMap(frame):
        maxDisp = stereo.initialConfig.getMaxDisparity()
        disp = (frame * (255.0 / maxDisp)).astype(np.uint8)
        disp = cv2.applyColorMap(disp, cv2.COLORMAP_JET)
        return disp


    print("Creating Stereo Depth pipeline")
    pipeline = dai.Pipeline()

    camLeft = pipeline.create(dai.node.MonoCamera)
    camRight = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)
    xoutLeft = pipeline.create(dai.node.XLinkOut)
    xoutRight = pipeline.create(dai.node.XLinkOut)
    xoutDisparity = pipeline.create(dai.node.XLinkOut)
    xoutDepth = pipeline.create(dai.node.XLinkOut)
    xoutRectifLeft = pipeline.create(dai.node.XLinkOut)
    xoutRectifRight = pipeline.create(dai.node.XLinkOut)

    camLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
    camRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    res = (
        dai.MonoCameraProperties.SensorResolution.THE_800_P
        if resolution[1] == 800
        else dai.MonoCameraProperties.SensorResolution.THE_720_P
        if resolution[1] == 720
        else dai.MonoCameraProperties.SensorResolution.THE_400_P
    )
    for monoCam in (camLeft, camRight):  # Common config
        monoCam.setResolution(res)
        # monoCam.setFps(20.0)

    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.initialConfig.setMedianFilter(median)  # KERNEL_7x7 default
    stereo.setRectifyEdgeFillColor(0)  # Black, to better see the cutout
    stereo.setLeftRightCheck(lrcheck)
    stereo.setExtendedDisparity(extended)
    stereo.setSubpixel(subpixel)

    xoutLeft.setStreamName("left")
    xoutRight.setStreamName("right")
    xoutDisparity.setStreamName("disparity")
    xoutDepth.setStreamName("depth")
    xoutRectifLeft.setStreamName("rectifiedLeft")
    xoutRectifRight.setStreamName("rectifiedRight")

    camLeft.out.link(stereo.left)
    camRight.out.link(stereo.right)
    stereo.syncedLeft.link(xoutLeft.input)
    stereo.syncedRight.link(xoutRight.input)
    stereo.disparity.link(xoutDisparity.input)
    if depth:
        stereo.depth.link(xoutDepth.input)
    if outRectified:
        stereo.rectifiedLeft.link(xoutRectifLeft.input)
        stereo.rectifiedRight.link(xoutRectifRight.input)

    streams = ["left", "right"]
    if outRectified:
        streams.extend(["rectifiedLeft", "rectifiedRight"])
    streams.append("disparity")
    if depth:
        streams.append("depth")
    print(f"Created the following streams: {streams}.")

    # Output folder for each stream
    streams_out_dirs = {}
    for stream in streams:
        out_dir = folder / stream
        os.makedirs(out_dir.absolute(), exist_ok=False)
        streams_out_dirs[stream] = out_dir

    calibData = dai.Device().readCalibration()
    leftMesh, rightMesh = getMesh(calibData)
    if generateMesh:
        meshLeft = list(leftMesh.tobytes())
        meshRight = list(rightMesh.tobytes())
        stereo.loadMeshData(meshLeft, meshRight)

    if meshDirectory is not None:
        saveMeshFiles(leftMesh, rightMesh, meshDirectory)

    IMG_TIME_FORMAT = '%Hh-%Mm-%Ss-%fus'
    IMG_EXIF_TIME_FMT = "%Y:%m:%d %H:%M:%S %fus"

    def write_img_simple(frame, stream_name: str, num: int, time: datetime):
        """Write image to jpeg without exif metadata"""
        timestamp = time.strftime(IMG_TIME_FORMAT)
        img_path = streams_out_dirs[stream_name] / f"{stream_name}-{num}_t{timestamp}.jpg"
        cv2.imwrite(str(img_path.absolute()), frame)


    def write_img_with_metadata(frame, stream_name: str, num: int, time: datetime):
        """Write image to jpeg with exif metadata"""
        img_pil = Image.fromarray(frame)
        exif_dict = img_pil.getexif()
        exif_dict[TIME_TAG] = time.strftime(IMG_EXIF_TIME_FMT)
        img_path = streams_out_dirs[stream_name] / f"{stream_name}-{num}.jpg"
        img_pil.save(str(img_path.absolute()), format='JPEG', exif=exif_dict)


    print("Creating DepthAI device")
    with dai.Device(pipeline) as device:
        # Create a receive queue for each stream
        stream_queues = [device.getOutputQueue(stream, 8, blocking=False) for stream in streams]
        print("Capturing frames ...")
        i = 0
        try:
            while True:
                i += 1
                for q in stream_queues:
                    stream_name = q.getName()
                    frame = q.get().getCvFrame()
                    time = datetime.now()
                    if stream_name == "depth":
                        frame = frame.astype(np.uint16)
                    # elif stream_name == "disparity":
                    #     frame = applyDisparityColorMap(frame)

                    # cv2.imshow(stream_name, frame)
                    write_img_simple(frame, stream_name, i, time)
        except KeyboardInterrupt:
            print("Stopping data capture due to keyboard interrupt.")
        else:
            exc_info = sys.exc_info()
            print(f"Stopping data capture due to uncaught exception: {exc_info}.")
        finally:
            data_size = du(str(folder))
            print(f"Saved {i+1} frames to {folder}. "
                  f"Total size is {data_size}.")

            # if cv2.waitKey(1) == ord("q"):
            #     break
