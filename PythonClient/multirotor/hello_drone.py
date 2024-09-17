import setup_path
import airsim

import numpy as np
import os
import tempfile
import pprint
import cv2
import time
import uuid
import random

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)

state = client.getMultirotorState()
s = pprint.pformat(state)
print("state: %s" % s)

imu_data = client.getImuData()
s = pprint.pformat(imu_data)
print("imu_data: %s" % s)

barometer_data = client.getBarometerData()
s = pprint.pformat(barometer_data)
print("barometer_data: %s" % s)

magnetometer_data = client.getMagnetometerData()
s = pprint.pformat(magnetometer_data)
print("magnetometer_data: %s" % s)

gps_data = client.getGpsData()
s = pprint.pformat(gps_data)
print("gps_data: %s" % s)

client.simEnableWeather(True)

client.simSetWeatherParameter(airsim.WeatherParameter.Snow, 0.75);

tmp_dir = os.path.join(tempfile.gettempdir(), f"airsim_drone__{uuid.uuid4()}")

vehicle_pos = client.simGetVehiclePose()


assets = client.simListAssets()
for asset in assets:
    print(asset)

for obj_count in range(1,20):
    scale = airsim.Vector3r(1.0, 1.0, 1.0)
    asset_name = '1M_Cube_Chamfer'
    desired_name = f"{asset_name}_spawn_{random.randint(0, 100)}_{obj_count}"
    object_location = airsim.Vector3r(
            vehicle_pos.position.x_val+(obj_count*10),
            vehicle_pos.position.y_val+(obj_count*10),
            vehicle_pos.position.z_val+(obj_count*10)
    )
    pose = airsim.Pose( position_val=object_location,orientation_val=vehicle_pos.orientation)
    print("##############")
    print(pprint.pformat(vehicle_pos))
    print(pprint.pformat(pose))
    print("##############")
    obj_name = client.simSpawnObject(desired_name, asset_name, pose, scale, True)
    print(obj_name)

default_fov = 90

for frame_count in range(1,31):
    print(f"Vehicle Position  :: {pprint.pformat(client.simGetVehiclePose())}")
    gps_data = client.getGpsData()
    s = pprint.pformat(gps_data)
    print("gps_data: %s" % s)
    time.sleep(10)

    if frame_count % 3 == 0:
        client.simSetCameraFov("0", default_fov * 1.5)
    elif frame_count % 5 == 0:
        client.simSetCameraFov("0", default_fov * 0.5)
    else:
        client.simSetCameraFov("0", default_fov)

    print(client.simGetCameraInfo("0"))


    responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene)])
    #print('Retrieved images: %d' % len(responses))

    #print ("Saving images to %s" % tmp_dir)
    try:
        os.makedirs(tmp_dir)
    except OSError:
        if not os.path.isdir(tmp_dir):
            raise

    for idx, response in enumerate(responses):

        filename = os.path.join(tmp_dir, str(frame_count)+"__"+str(idx))

        if response.pixels_as_float:
            print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
            airsim.write_pfm(os.path.normpath(filename + '.pfm'), airsim.get_pfm_array(response))
        elif response.compress: #png format
            #print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
            airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
        else: #uncompressed array
            #print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
            img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) # get numpy array
            img_rgb = img1d.reshape(response.height, response.width, 3) # reshape array to 4 channel image array H X W X 3
            cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb) # write to png

'''
airsim.wait_key('Press any key to takeoff')
print("Taking off...")
client.armDisarm(True)
client.takeoffAsync().join()

state = client.getMultirotorState()
print("state: %s" % pprint.pformat(state))

airsim.wait_key('Press any key to move vehicle to (-10, 10, -10) at 5 m/s')
client.moveToPositionAsync(-10, 10, -10, 5).join()

client.hoverAsync().join()

state = client.getMultirotorState()
print("state: %s" % pprint.pformat(state))

airsim.wait_key('Press any key to take images')
# get camera images from the car
responses = client.simGetImages([
    airsim.ImageRequest("0", airsim.ImageType.DepthVis),  #depth visualization image
    airsim.ImageRequest("1", airsim.ImageType.DepthPerspective, True), #depth in perspective projection
    airsim.ImageRequest("1", airsim.ImageType.Scene), #scene vision image in png format
    airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)])  #scene vision image in uncompressed RGBA array
print('Retrieved images: %d' % len(responses))

tmp_dir = os.path.join(tempfile.gettempdir(), "airsim_drone")
print ("Saving images to %s" % tmp_dir)
try:
    os.makedirs(tmp_dir)
except OSError:
    if not os.path.isdir(tmp_dir):
        raise

for idx, response in enumerate(responses):

    filename = os.path.join(tmp_dir, str(idx))

    if response.pixels_as_float:
        print("Type %d, size %d" % (response.image_type, len(response.image_data_float)))
        airsim.write_pfm(os.path.normpath(filename + '.pfm'), airsim.get_pfm_array(response))
    elif response.compress: #png format
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        airsim.write_file(os.path.normpath(filename + '.png'), response.image_data_uint8)
    else: #uncompressed array
        print("Type %d, size %d" % (response.image_type, len(response.image_data_uint8)))
        img1d = np.fromstring(response.image_data_uint8, dtype=np.uint8) # get numpy array
        img_rgb = img1d.reshape(response.height, response.width, 3) # reshape array to 4 channel image array H X W X 3
        cv2.imwrite(os.path.normpath(filename + '.png'), img_rgb) # write to png

airsim.wait_key('Press any key to reset to original state')

client.reset()
client.armDisarm(False)

# that's enough fun for now. let's quit cleanly
'''
client.enableApiControl(False)
