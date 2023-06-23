#!/usr/bin/env python3
from flask_restful import Resource
from flask import make_response, send_file    
import io
import sys
import os
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.append(parent_dir)
from common.camera_data_loader import get_camera_nodes
from common.image_collector import Collector
from common.device_maker import CameraDevice

class Preview(Resource):
    def get(self, camera_id):
        cd = CameraDevice()
        c = Collector()
        node_data, node_names = get_camera_nodes()
        camera_ip = node_data[f"camera_{camera_id}"]['ip']
        if cd.upload_pipeline(camera_ip):
            byte_stream = c.get_frame(c.preview_queue, "preview")
            if byte_stream is None:
                return {'status': 'error', 'info' :"Frame capture failed"}, 400
            
            response = make_response(send_file(io.BytesIO(byte_stream), download_name="preview.jpg", mimetype="image/jpeg"))
            return response, 200

        else:
            return {'status': 'error', 'info' :"Camera is offline"}, 400
            
