from flask_restful import Resource, abort
from flask import make_response, send_file   
import io
import sys
import os
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.append(parent_dir)
from common.device_maker import CameraDevice
from common.camera_data_loader import get_camera_nodes
from common.check_camera import check_camera_connection

class Cameras(Resource):
  def get(self):
    try:
      cd = CameraDevice()
      node_data, node_names = get_camera_nodes()
      cameras = []
      for node_name in node_names :
        cameraid = node_data[node_name]['camera_id']
        camera_ip = node_data[node_name]['ip']
        if check_camera_connection(camera_ip):
            cameras.append(cameraid)

        # if cd.upload_pipeline(camera_ip):
        #   cameras.append(cameraid)
        
      response = {'status': 'success', 'cameras': cameras}

      if len(cameras):
        return response, 200
      else:
        return response, 204
      
    except Exception as e:
      return {'status': 'error', 'info' :e}, 400
