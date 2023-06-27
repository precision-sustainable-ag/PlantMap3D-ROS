import depthai as dai
import subprocess

def check_camera_connection(ip_address):

    command = ["ping", "-c", "1", f"{ip_address}"]
    ret = subprocess.call(command)

    if ret == 0:
        # If the device connection is successful, the camera is connected
        print("Camera is connected.")
        return True
    else:
        # If an error occurs during the connection, the camera is not connected
        print(f"Camera connection error")
        return False

