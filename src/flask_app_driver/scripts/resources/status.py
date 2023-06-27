#!/usr/bin/env python3
from flask import Flask, request, jsonify
from flask_restful import Resource, reqparse
import os 
import time
import subprocess
import json

class Status(Resource):

    def post(self, action):

        parser = reqparse.RequestParser()
        parser.add_argument("TypeOfCrop", type = str)
        parser.add_argument("TypeOfCashCrop",type=str)
        parser.add_argument("TypeOfCoverCrops",type=str)

        data = parser.parse_args()

        type_of_crop = data['TypeOfCrop']
        type_of_cash_crop = data['TypeOfCashCrop']
        type_of_cover_crop = data['TypeOfCoverCrops']

        
        if action == 'start':      
            start_time= time.time()
            command = ["roslaunch", "pm3d_system_config", "oakd_dummygps_system_start.launch"]
            process = subprocess.Popen(command)
            # os.system("roslaunch pm3d_system_config oakd_dummygps_system_start.launch")
            print("Starting ros")
            print(data)
            response =  {
                "status": "Successful start",
                "args": data,
                # 'args': {'TypeOfCrop': type_of_crop,
                # 'TypeOfCashCrop': type_of_cash_crop,
                # 'TypeOfCoverCrops':type_of_cover_crop},
                "time_elapsed":  time.time() - start_time
                }
            return response, 200

        elif action == 'stop':
            start_time= time.time()
            command = ["roslaunch", "pm3d_system_config", "system_shutdown.launch"]
            process = subprocess.Popen(command)
            # os.system("roslaunch pm3d_system_config system_shutdown.launch")
            reponse = {'status': "Successful stop", "time_elapsed":  time.time() - start_time}
            return reponse, 200, 

        else:
            return {'status': 'error', 'info': 'invalid option specified. use start or stop!'}, 400
        
# curl -i -X POST -H "Content-Type: application/json"  -d  '{"TypeOfCrop":"CASHCROP","TypeOfCashCrop":"Soybeans"}' http://localhost:5000/status/start
