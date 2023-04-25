#!/usr/bin/env python3
from flask_restful import Resource
import os 
# from common.device_maker import CameraDevice
# from common.image_collector import Collector

class Status(Resource):
    def get(self, action):
        if action == 'start':
            # cd = CameraDevice()
            # c = Collector()
           
            # if cd.status == 'inactive':
            #     # cd.upload_pipeline()
            #     # c.initialize_queues(cd)
           
            os.system("roslaunch pm3d_system_config system_start.launch")
            print("Starting ros")
            return {'status': "Successful start"}, 200
            # else:
            #     return {'status': 'error', 'info': 'pipeline already uploaded!'}, 400

        elif action == 'stop':
        #     # cd = CameraDevice()
        #     if cd.status == 'active':
        #         # cd.close_pipeline()
            os.system("roslaunch pm3d_system_config system_shutdown.launch")
            return {'status': "Successful stop"}, 200
        #     else:
        #         return {'status': 'error', 'info': 'pipeline not uploaded!'}, 400

        else:
            return {'status': 'error', 'info': 'invalid option specified. use start or stop!'}, 400