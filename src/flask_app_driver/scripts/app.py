#!/usr/bin/env python3
"""
@author: MathewAaron
"""

import rospy

from flask import Flask, send_file
from flask_restful import Resource, Api
import io

# from common.image_collector import Collector
# from common.device_maker import CameraDevice
import os 
os.chdir(os.path.dirname(os.path.abspath(__file__)))

from resources.status import Status
from resources.preview import Preview
# from resources.rgb import RGB
# from resources.depth import Depth
# from resources.mono import Mono



def register_endpoints(api):
  api.add_resource(Status, '/status', '/status/<string:action>')
  api.add_resource(Preview, '/preview')
#   api.add_resource(RGB, '/rgb')
#   api.add_resource(Depth, '/depth')
#   api.add_resource(Mono, '/mono', '/mono/<string:side>')

def create_app(debug=True):
    # Create app
    app = Flask(__name__)

    # Create api
    api = Api(app)

    register_endpoints(api)

    return app

# if __name__ == '__main__':

#     rospy.init_node("flask_app_node")
    # os.system("roslaunch pm3d_system_config system_start.launch")

    