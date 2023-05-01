#!/usr/bin/env python3
"""
Created on Fri Apr 28 19:53:57 2023

@author: skovsen
"""

import roslib; roslib.load_manifest('oakd_camera_driver')
import rospy
import rospkg

rospack_species = rospkg.RosPack()
__test_data_path = rospack_species.get_path('configs') + '/config/test_plantmap_2023-04-20.csv'

rospack_datasave = rospkg.RosPack()
__biomass_data_save_path = rospack_datasave.get_path('data_saver_driver') + '/biomass_estimation/'

import numpy as np
import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt
import pandas
import utm
from scipy import spatial
import scipy
import io
from sklearn.cluster import AgglomerativeClustering
from matplotlib_scalebar.scalebar import ScaleBar
import imageio

class LinearNDInterpolatorExt(object):
    def __init__(self, points, values):
        self.funcinterp = scipy.interpolate.LinearNDInterpolator(points, values)
        self.funcnearest = scipy.interpolate.NearestNDInterpolator(points, values)

    def __call__(self, *args):
        t = self.funcinterp(*args)
        u = self.funcnearest(*args)

        t[np.isnan(t)] = u[np.isnan(t)]
        return t
    

class mapInterpolator:
    def __init__(self, csv_path, data_column):
        self.csv_path = csv_path
        self.data_column = data_column
    
    def generate_all_maps_from_today(self, save_directory):
        data= pandas.read_csv(self.csv_path, sep=';')
        
        
        im_size = 5328.*4608. #This needs to be changed to our processing im size from our real-time inference
        min_points_per_cluster = 40
        DPI = 600
        
        
        coordinateUTM = utm.from_latlon(data['latitude'].to_numpy(),data['longitude'].to_numpy())
        
        x = coordinateUTM[0]
        
        y = coordinateUTM[1]
        
        z = data[self.data_column].to_numpy()/im_size
            
        X = np.vstack((x, y)).T
        
        clust = AgglomerativeClustering(n_clusters=None, distance_threshold=40, linkage='single').fit(X)
        clust.labels_
        num_clusters = np.max(clust.labels_)
        print(np.sum(clust.labels_ == -1))
        for clusterId in range(num_clusters+1):
            print(np.sum(clust.labels_ == clusterId))
            if(np.sum(clust.labels_ == clusterId) > min_points_per_cluster): #only process if more than 30 image samples is part of the cluster
                xk = x[clust.labels_ == clusterId]
                yk = y[clust.labels_ == clusterId]
                zk = z[clust.labels_ == clusterId]
                
                resolution = 5  # grid is 5 x 5 meters per cell
                X_grid_lin = np.arange(min(xk)-5, max(xk)+5, step=resolution)
                Y_grid_lin = np.arange(min(yk)-5, max(yk)+5, step=resolution)
            
                X_grid, Y_grid = np.meshgrid(X_grid_lin, Y_grid_lin)  # 2D grid for interpolation
                
               
                mesh_flat_dim = len(X_grid_lin)*len(Y_grid_lin)
                
                a = np.reshape(X_grid, mesh_flat_dim)
                b = np.reshape(Y_grid, mesh_flat_dim)
                ab = np.vstack((a,b)).T
                
                distance,index = spatial.KDTree(np.vstack((xk,yk)).T).query(ab)
                
                X_grid_closest_point = np.reshape(distance, (len(Y_grid_lin),len(X_grid_lin)))
                Y_grid_closest_point = np.reshape(distance, (len(X_grid_lin),len(Y_grid_lin)))
        
                interp = LinearNDInterpolatorExt(list(zip(xk, yk)), zk)
            
            
                
                X_grid_masked = X_grid
                Y_grid_masked = Y_grid
                Z_grid_masked = interp(X_grid_masked, Y_grid_masked)
                Z_grid_masked[X_grid_closest_point>12]=np.nan
                
                fig = plt.figure(figsize=(4, 3), dpi=DPI)
                plt.magma()
                plt.pcolormesh(X_grid_masked, Y_grid_masked, Z_grid_masked, shading='auto')
                
                plt.clim(0, 1)  # manually setup the range of the colorscale and colorbar
            
                plt.plot(xk, yk, color='green', marker='o', label="input point", markersize=0.8, linestyle="None")
                #ax = nybb.plot()
                ax = plt.gca()
                ax.add_artist(ScaleBar(resolution))
                #plt.legend()
            
                #plt.colorbar()
                from matplotlib.ticker import FuncFormatter
        
                fmt = lambda x, pos: '{:.0%}'.format(x)
                cbar = plt.colorbar(format=FuncFormatter(fmt))
        
                plt.title(self.data_column.replace('_', ' '))
            
                plt.axis("equal")
                plt.axis('off')
            

                fig = plt.gcf()
                plt.switch_backend('agg')
                fig.tight_layout(pad=0)
                fig.canvas.draw()

                
                io_buf = io.BytesIO()
                fig.savefig(io_buf, format='raw', dpi=DPI)
                io_buf.seek(0)
                img_arr = np.reshape(np.frombuffer(io_buf.getvalue(), dtype=np.uint8),
                                     newshape=(int(fig.bbox.bounds[3]), int(fig.bbox.bounds[2]), -1))
                io_buf.close()
                imageio.imwrite(save_directory + "/map_" + str(clusterId) + ".png", img_arr[:,:,0:4])

    def generate_live_map(self, save_path, lat, lon):
        data= pandas.read_csv(self.csv_path, sep=';')
        
        
        im_size = 5328.*4608. #This needs to be changed to our processing im size from our real-time inference
        min_points_per_cluster = 50
        DPI = 600
        
        
        coordinateUTM = utm.from_latlon(data['latitude'].to_numpy(),data['longitude'].to_numpy())
        
        x = coordinateUTM[0]
        
        y = coordinateUTM[1]
        
        current_positionUTM = utm.from_latlon(lat,lon)
        
        current_distance_to_samples_x = np.abs(current_positionUTM[0]-x)
        current_distance_to_samples_y = np.abs(current_positionUTM[1]-y)
        current_distance_to_samples_squared = np.square(current_distance_to_samples_x)+np.square(current_distance_to_samples_y)
        closest_point_index = np.argmin(current_distance_to_samples_squared)
        
        z = data[self.data_column].to_numpy()/im_size
            
        X = np.vstack((x, y)).T
        
        clust = AgglomerativeClustering(n_clusters=None, distance_threshold=40, linkage='single').fit(X)
        nearest_cluser_id = clust.labels_[closest_point_index]
        num_clusters = np.max(clust.labels_)
        print(np.sum(clust.labels_ == -1))
        clusterId = nearest_cluser_id
        
        print(np.sum(clust.labels_ == clusterId))
        if(np.sum(clust.labels_ == clusterId) > min_points_per_cluster): #only process if more than 30 image samples is part of the cluster
            xk = x[clust.labels_ == clusterId]
            yk = y[clust.labels_ == clusterId]
            zk = z[clust.labels_ == clusterId]
            
            resolution = 5  # grid is 5 x 5 meters per cell
            X_grid_lin = np.arange(min(xk)-5, max(xk)+5, step=resolution)
            Y_grid_lin = np.arange(min(yk)-5, max(yk)+5, step=resolution)
        
            X_grid, Y_grid = np.meshgrid(X_grid_lin, Y_grid_lin)  # 2D grid for interpolation
            
           
            mesh_flat_dim = len(X_grid_lin)*len(Y_grid_lin)
            
            a = np.reshape(X_grid, mesh_flat_dim)
            b = np.reshape(Y_grid, mesh_flat_dim)
            ab = np.vstack((a,b)).T
            
            distance,index = spatial.KDTree(np.vstack((xk,yk)).T).query(ab)
            
            X_grid_closest_point = np.reshape(distance, (len(Y_grid_lin),len(X_grid_lin)))
            Y_grid_closest_point = np.reshape(distance, (len(X_grid_lin),len(Y_grid_lin)))
    
            interp = LinearNDInterpolatorExt(list(zip(xk, yk)), zk)
        
        
            
            X_grid_masked = X_grid
            Y_grid_masked = Y_grid
            Z_grid_masked = interp(X_grid_masked, Y_grid_masked)
            Z_grid_masked[X_grid_closest_point>12]=np.nan
            
            fig = plt.figure(figsize=(4, 3), dpi=DPI)
            plt.magma()
            plt.pcolormesh(X_grid_masked, Y_grid_masked, Z_grid_masked, shading='auto')
            
            plt.clim(0, 1)  # manually setup the range of the colorscale and colorbar
        
            plt.plot(xk, yk, color='green', marker='o', label="input point", markersize=0.8, linestyle="None")
            plt.plot(current_positionUTM[0], current_positionUTM[1], color='red', marker='o', label="You are here", markersize=3, linestyle="None")
            #ax = nybb.plot()
            ax = plt.gca()
            ax.add_artist(ScaleBar(resolution))
            #plt.legend()
        
            #plt.colorbar()
            from matplotlib.ticker import FuncFormatter
    
            fmt = lambda x, pos: '{:.0%}'.format(x)
            cbar = plt.colorbar(format=FuncFormatter(fmt))
    
            plt.title(self.data_column.replace('_', ' '))
        
            plt.axis("equal")
            plt.axis('off')
        
            fig = plt.gcf()
            plt.switch_backend('agg')
            fig.tight_layout(pad=0)
            fig.canvas.draw()

            
            io_buf = io.BytesIO()
            fig.savefig(io_buf, format='raw', dpi=DPI)
            io_buf.seek(0)
            img_arr = np.reshape(np.frombuffer(io_buf.getvalue(), dtype=np.uint8),
                                 newshape=(int(fig.bbox.bounds[3]), int(fig.bbox.bounds[2]), -1))
            io_buf.close()
            imageio.imwrite(save_path, img_arr[:,:,0:4])
    
if __name__ == "__main__":
    rospy.loginfo("biomass_visualization_node")
    rospy.init_node("map_interpolator")
    #mapInterpolator(csv_path='analysis_plantmap_2023-04-20.csv', data_column='live_biomass_pixels').generate_all_maps_from_today(save_directory='D:\post.doc\map_interpolation')
    mapInterpolator(csv_path=__test_data_path, data_column='live_biomass_pixels').generate_live_map(save_path=__biomass_data_save_path+'live_map.png', lat=39.0127, lon=-76.822)
