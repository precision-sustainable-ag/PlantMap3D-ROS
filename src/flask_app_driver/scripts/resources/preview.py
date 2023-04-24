#!/usr/bin/env python3
from flask_restful import Resource
from flask import make_response, send_file    
import io

# from common.device_maker import CameraDevice
# from common.image_collector import Collector

class Preview(Resource):
    def get(self):
        cd = CameraDevice()
        c = Collector()

        byte_stream = c.get_frame(c.preview_queue, "preview")

        if byte_stream is None:
            return {'status': 'error', 'info': 'no image!'}, 400

        response = make_response(send_file(io.BytesIO(byte_stream), download_name="preview.jpg", mimetype="image/jpeg"))

        return "Heres a preview", 200