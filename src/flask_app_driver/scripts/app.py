#!/usr/bin/env python3
"""
@author: MathewAaron
"""

import rospy
from flask import Flask, send_file
from flask_restful import Resource, Api
import io
import os 
os.chdir(os.path.dirname(os.path.abspath(__file__)))

from resources.status import Status
from resources.cameras import Cameras
from resources.preview import Preview
from resources.biomap import Biomap
from resources.segmentation import Segmentation

def register_endpoints(api):
  api.add_resource(Status, '/status', '/status/<string:action>')
  api.add_resource(Cameras, '/cameras')
  api.add_resource(Preview, '/preview/<string:camera_id>')
  api.add_resource(Segmentation, '/segmentation/<string:camera_id>')
  api.add_resource(Biomap, '/biomap')

def create_app():
    # Create app
    app = Flask(__name__)

    # Create api
    api = Api(app)

    register_endpoints(api)

    return app

if __name__ == '__main__':
  app = create_app()
  app.run(debug=True)


    