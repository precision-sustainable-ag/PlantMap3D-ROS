#!/usr/bin/env python3
from flask_restful import Resource
from flask import make_response, send_file    
import io
import sys
import os
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.append(parent_dir)
print(sys.path)
from common.camera_data_loader import get_camera_nodes
from common.image_collector import Collector
from common.device_maker import CameraDevice
from common.check_camera import check_camera_connection

class Preview(Resource):
    def get(self, camera_id):
        camera_device = CameraDevice()
        img_collector = Collector()
        node_data, node_names = get_camera_nodes()
        camera_ip = node_data[f"camera_{camera_id}"]['ip']

        if not check_camera_connection(camera_ip):
            return {"status": "Error","info": "Camera is offline"} , 400

        camera_device.upload_pipeline(camera_ip)
        rgb_out, _ = camera_device.get_camera_image()
        if rgb_out is None:
            return {"status": "Error","info": "Frame capture failed"} , 400
        
        img_stream = img_collector.encode_frame(rgb_out)
        response = make_response(send_file(img_stream, download_name="preview.jpg", mimetype="image/jpeg"))

        return response




