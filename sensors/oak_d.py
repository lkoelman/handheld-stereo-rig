"""
Data collection using OAK-D.

Based on example https://docs.luxonis.com/projects/api/en/latest/samples/StereoDepth/stereo_depth_video/
"""
import os
from pathlib import Path
from typing import Optional, Tuple
from datetime import datetime
import subprocess

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

def getMesh(
        calibData,
        resolution: Tuple[int, int])-> Tuple[np.array, np.array]:
    """
    Generate calibration mesh from camera calibration parameters.
    """
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

def collect(folder: Path,
            resolution: Optional[str] = "720",
            mesh_dir: Optional[str] = None,
            gen_calib_meshes: Optional[bool] = False,
            out_rectified: Optional[bool] = False,
            lrcheck: Optional[bool] = False,
            extended_disparity: Optional[bool] = False,
            subpixel: Optional[bool] = False,
            depth: Optional[bool] = False,
            median: Optional[str] = '7x7',
            capture_rgb: Optional[bool] = True,
            apply_colormap: Optional[bool] = False):
    """
    Parameters:
        folder: Output folder to save images and mesh files.
        resolution: Sets the resolution on mono cameras. Options: 800 | 720 | 400.
        mesh_dir: Output directory for mesh files. If not specified mesh files won't be saved
        gen_calib_meshes: Read camera intrinsics from device, generate mesh files and load them into the stereo node.
        out_rectified: Generate and display rectified streams
        lrcheck: Better handling for occlusions
        extended_disparity: Closer-in minimum depth, disparity range is doubled.
        subpixel: Better accuracy for longer distance, fractional disparity 32-levels
        depth: Display depth frames.
        median: Choose the size of median filtering. Options: OFF | 3x3 | 5x5 | 7x7 (default)
        apply_colormap: Apply colormap to disparity image before saving.
    """

    resolutionMap = {"800": (1280, 800), "720": (1280, 720), "400": (640, 400)}
    if resolution not in resolutionMap:
        exit("Unsupported resolution!")

    resolution = resolutionMap[resolution]
    meshDirectory = mesh_dir  # Output dir for mesh files

    timestamp = datetime.now().isoformat().replace(':', '_')[:-3]
    folder = folder / timestamp
    if meshDirectory is None:
        meshDirectory = str(folder / "undistort_rectify_maps")
        os.makedirs(meshDirectory)

    outRectified = out_rectified  # Output and display rectified streams

    medianMap = {
        "OFF": dai.StereoDepthProperties.MedianFilter.MEDIAN_OFF,
        "3x3": dai.StereoDepthProperties.MedianFilter.KERNEL_3x3,
        "5x5": dai.StereoDepthProperties.MedianFilter.KERNEL_5x5,
        "7x7": dai.StereoDepthProperties.MedianFilter.KERNEL_7x7,
    }
    if median not in medianMap:
        exit("Unsupported median size!")

    median_filter = medianMap[median]

    print("StereoDepth config options:")
    print("    Resolution:  ", resolution)
    print("    Left-Right check:  ", lrcheck)
    print("    Extended disparity:", extended_disparity)
    print("    Subpixel:          ", subpixel)
    print("    Median filtering:  ", median_filter)
    print("    Generating mesh files:  ", gen_calib_meshes)
    print("    Outputting mesh files to:  ", meshDirectory)


    def applyDisparityColorMap(frame):
        maxDisp = stereo.initialConfig.getMaxDisparity()
        disp = (frame * (255.0 / maxDisp)).astype(np.uint8)
        disp = cv2.applyColorMap(disp, cv2.COLORMAP_JET)
        return disp

    pipeline = dai.Pipeline()

    ##########################################################################
    # SETPARAM: mono RGB configuration
    print("Creating mono RGB pipeline")
    # RGB source node
    camRgb = pipeline.create(dai.node.ColorCamera)
    camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
    camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_4_K)
    
    # RGB output node
    xoutRgb = pipeline.create(dai.node.XLinkOut)
    xoutRgb.setStreamName("rgb_raw")
    camRgb.video.link(xoutRgb.input)

    # RGB jpeg-encoded node
    # videoEnc = pipeline.create(dai.node.VideoEncoder)
    # videoEnc.setDefaultProfilePreset(1, dai.VideoEncoderProperties.Profile.MJPEG)
    # camRgb.still.link(videoEnc.input)
    # xoutStill = pipeline.create(dai.node.XLinkOut)
    # xoutStill.setStreamName("rgb_enc")
    # videoEnc.bitstream.link(xoutStill.input)

    # SETPARAM: RGB camera settings
    # You can configure ColorCamera ISP values such as sharpness, luma denoise and chroma denoise, which can improve IQ. We have noticed that sometimes these values provide better results:
    camRgb.initialControl.setSharpness(0)     # range: 0..4, default: 1
    camRgb.initialControl.setLumaDenoise(0)   # range: 0..4, default: 1
    camRgb.initialControl.setChromaDenoise(4) # range: 0..4, default: 1

    ##########################################################################
    print("Creating Stereo Depth pipeline")

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

    # SETPARAM: stereo resolution
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

    # SETPARAM: stereo depth parameters
    # see https://docs.luxonis.com/projects/api/en/latest/samples/StereoDepth/depth_post_processing/
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    # Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
    stereo.initialConfig.setMedianFilter(median_filter)
    stereo.setRectifyEdgeFillColor(0)  # Black, to better see the cutout
    stereo.setLeftRightCheck(lrcheck)
    stereo.setExtendedDisparity(extended_disparity)
    stereo.setSubpixel(subpixel)

    # See https://docs.luxonis.com/projects/api/en/latest/samples/StereoDepth/depth_post_processing/?highlight=decimationfilter#depth-filters
    config = stereo.initialConfig.get()
    config.postProcessing.speckleFilter.enable = False
    config.postProcessing.speckleFilter.speckleRange = 50
    config.postProcessing.temporalFilter.enable = True
    config.postProcessing.spatialFilter.enable = True
    config.postProcessing.spatialFilter.holeFillingRadius = 2
    config.postProcessing.spatialFilter.numIterations = 1
    config.postProcessing.thresholdFilter.minRange = 400
    config.postProcessing.thresholdFilter.maxRange = 15000
    config.postProcessing.decimationFilter.decimationFactor = 1
    stereo.initialConfig.set(config)

    ##########################################################################
    # Stream names
    xoutLeft.setStreamName("left")
    xoutRight.setStreamName("right")
    xoutDisparity.setStreamName("disparity")
    xoutDepth.setStreamName("depth")
    xoutRectifLeft.setStreamName("rectifiedLeft")
    xoutRectifRight.setStreamName("rectifiedRight")

    # Link nodes (data flow topology)
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

    # Identify nodes we're interested in as output streams
    streams = ["left", "right"]
    if outRectified:
        streams.extend(["rectifiedLeft", "rectifiedRight"])
    streams.append("disparity")
    if depth:
        streams.append("depth")
    if capture_rgb:
        streams.append("rgb_raw")
    print(f"Created the following streams: {streams}.")

    # Output folder for each stream
    streams_out_dirs = {}
    for stream in streams:
        out_dir = folder / stream
        os.makedirs(out_dir.absolute(), exist_ok=False)
        streams_out_dirs[stream] = out_dir

    # See documentation
    # - https://docs.luxonis.com/projects/api/en/latest/references/python/?highlight=loadMeshData#depthai.node.StereoDepth.loadMeshData
    # Fetch calibration from EEPROM data
    calibData = dai.Device().readCalibration()  
    leftMesh, rightMesh = getMesh(calibData, resolution)
    if gen_calib_meshes:
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

    def write_enc_img_bytes(data, stream_name: str, num: int, time: datetime):
        """Write raw img bytes (already encoded)"""
        timestamp = time.strftime(IMG_TIME_FORMAT)
        img_path = streams_out_dirs[stream_name] / f"{stream_name}-{num}_t{timestamp}.jpg"
        with open(str(img_path.absolute()), "wb") as f:
            f.write(data)

    # def write_img_with_metadata(frame, stream_name: str, num: int, time: datetime):
    #     """Write image to jpeg with exif metadata"""
    #     img_pil = Image.fromarray(frame)
    #     exif_dict = img_pil.getexif()
    #     exif_dict[TIME_TAG] = time.strftime(IMG_EXIF_TIME_FMT)
    #     img_path = streams_out_dirs[stream_name] / f"{stream_name}-{num}.jpg"
    #     img_pil.save(str(img_path.absolute()), format='JPEG', exif=exif_dict)


    print("Creating DepthAI device")
    with dai.Device(pipeline) as device:
        # Create a receive queue for each stream
        stream_queues = [
            device.getOutputQueue(stream, 8, blocking=False)
            for stream in streams if stream != "rgb_raw"
        ]
        if capture_rgb:
            # rgb_queue = device.getOutputQueue(name="rgb_enc", maxSize=30, blocking=True)
            rgb_queue = device.getOutputQueue(name="rgb_raw", maxSize=30, blocking=False)
            stream_queues.append(rgb_queue)
        else:
            rgb_queue = None
        print("Capturing frames ...")
        i = 0
        t = datetime.now()
        try:
            while True:
                i += 1

                # Save raw streams
                for q in stream_queues:
                    stream_name = q.getName()
                    frame = q.get().getCvFrame()
                    if stream_name == "depth":
                        frame = frame.astype(np.uint16)
                    elif apply_colormap and (stream_name == "disparity"):
                        frame = applyDisparityColorMap(frame)

                    # cv2.imshow(stream_name, frame)
                    write_img_simple(frame, stream_name, i, datetime.now())

                # RGB stream is already encoded: write bytes directly
                # if rgb_queue and rgb_queue.has():
                #     write_enc_img_bytes(rgb_queue.get().getData(), 'rgb', i,
                #                         datetime.now())
                tn = datetime.now()
                dt = (tn - t).total_seconds()
                freq = 1. / dt
                print(f"Processed frame {i} in {dt * 1e3} ms ({freq} Hz)")
                t = tn
            
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

if __name__ == '__main__':
    collect(Path('.').absolute(), capture_rgb=False)