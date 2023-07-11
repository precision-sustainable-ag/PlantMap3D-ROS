from flask_restful import Resource
from flask import make_response, send_file    
import io
import os
import sys
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.append(parent_dir)
from common.map_visualization import LinearNDInterpolatorExt,mapInterpolator
from common.image_collector import Collector

class Biomap(Resource):
    def get(self):
        cvs_path = "/home/plantmap3d/Desktop/ROS_dev_ws/PlantMap3D-ROS/src/flask_app_driver/scripts/common/test_plantmap_2023-04-20.csv"
        save_path = '/home/plantmap3d/Desktop/ROS_dev_ws/PlantMap3D-ROS/src/flask_app_driver/scripts'
        img_collector = Collector()
        map_obj = mapInterpolator(csv_path=cvs_path, data_column='live_biomass_pixels')
        image_arr = None
        try:
            image_arr = map_obj.generate_all_maps_from_today(save_path)
            
            img_stream = img_collector.encode_frame(image_arr[:,:,i])
        except Exception as e:
            return {"status": 400,"info": "No map available"}
        
        for i in range(4):
            try:
                response = make_response(send_file(img_stream, download_name=f"map_{i}.jpg", mimetype="image/jpeg"))
            except Exception as e:
                return {"status": 400,"info": "Failed to generate map"}

        return response


        