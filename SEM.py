import glob
import sys
import numpy as np
import os
sys.path.append(os.path.abspath(__file__).split("05_code")[0]+"05_code")
from ros_ws.src.transformation.transformation import Transformer
from ros_ws.src.utils import cfg
# platform specific imports

from ros_ws.src.data_handler.pcd.pcd_datahandler import LidarPointCloud

# 
class SEM:
    def __init__(self, data_path, reigon = 25.0, FOV= 70.0):
        self.objects_points_db = {}
        self.reigon = reigon
        self.FOV = FOV
        self.FOV_depth = 25.0
        self.transformer = Transformer(data_path, cfg.LOCAL_LOC_JSON)
        

    def create_object_type(self, object_name, alpha = 0.9, max_hits = 50.0, min_hits = -6, R_max_cov = 0.7, R_min_cov = 0.05, add_hit = 5.5, sub_hit = 1):
        self.objects_points_db[object_name]={}
        self.objects_points_db[object_name]["alpha"] = alpha         #Fraction by which new samples should influence the cluster of an object
        self.objects_points_db[object_name]["max_hits"] = max_hits   #Maximum hits in which we are certain that the object exists at its indicated poition
        self.objects_points_db[object_name]["min_hits"] = min_hits   #Minimum hits in which we are totally uncertain of the object location and we can delete it from db
        self.objects_points_db[object_name]["R_max_cov"] = R_max_cov #Maximum covariance that the objects covariances can't exceed after multiple misses 
        self.objects_points_db[object_name]["R_min_cov"] = R_min_cov #Minimum covariance that the objects covariances can't be less than when hits is between 0 and max_hits
        self.objects_points_db[object_name]["delta_cov"] = R_max_cov * (max_hits - 1) / max_hits**2 #Difference in covariance added when there is a hit or a miss happen
        self.objects_points_db[object_name]["add_hit"] = add_hit #hits added when an object is detected
        self.objects_points_db[object_name]["sub_hit"] = sub_hit #hits subtracted when an object is missed
        self.objects_points_db[object_name]["objects"] = np.array([])
        print(self.objects_points_db)



    
    def update_db(self, object_name, points, timestamp, auto_create_new_types = False, log=False):
        if auto_create_new_types:
            if object_name not in self.objects_points_db.keys():
                self.create_object_type(object_name)
        else:
            assert object_name in self.objects_points_db.keys(), "No object type with the specified name "+str(object_name)+" exists in the database please use create_object_type first."
        
        if log:
             print("points local in "+timestamp+" :",points) 
        #Pre 1: Clip points that is outside the reigon of the vehicle
        points = points[np.argwhere((np.abs(points[:,0]) <= self.reigon) & (np.abs(points[:,1]) <= self.reigon))]
        if points.shape[0] == 0:
            return self.objects_points_db[object_name]["objects"]
        points = points[0]
        #Pre 2: Clip points that is outside the FOV of the cameras
        clipped_points = []
        for point in points:
            angle = np.arctan2(point[1],point[0]) * 180 / np.pi
            # print(angle)
            if angle <= self.FOV and angle >= -self.FOV:
               clipped_points.append(point)
            elif (angle+180)%360 <= self.FOV and (angle+180)%360 >= -self.FOV:
               clipped_points.append(point)
        points = np.array(clipped_points)
        if log:
             print("Clipped points local in "+timestamp+" :",points) 
                
        if points.shape[0] == 0:
            return self.objects_points_db[object_name]["objects"]

        pcdhandler = LidarPointCloud(3, points)
        points_world = self.transformer.transform_sweep_stamp_base2world(timestamp,pcdhandler.points).T
        if log:
            print("points in world: ", points_world)
      
        if self.objects_points_db[object_name]["objects"].shape[0] == 0: #No objects are in the database for this type
            track_ids = np.arange(points_world.shape[0]).reshape(points_world.shape[0],1)

            self.objects_points_db[object_name]["objects"] = np.hstack((points_world,np.zeros((points_world.shape[0],1)),np.repeat(np.array([[self.objects_points_db[object_name]["R_max_cov"]]]),points_world.shape[0], axis=0),track_ids)) #x, y, z, hits, cov, track_id
            return self.objects_points_db[object_name]["objects"]

        reactive_points_indices = []
        updated_reactive_objects = []
        new_reactive_objects = []
        new_reactive_objects_indices = []
        next_id = self.objects_points_db[object_name]["objects"].shape[0]

        for i in range(len(points_world)):
            point = points_world[i]
            dist_db = np.linalg.norm(self.objects_points_db[object_name]["objects"][:,0:2] - point[0:2], axis=1) #Euclidean distance in 2D position
            if log:
                print("Distance db: ",np.amin(dist_db))
            if np.amin(dist_db) <= self.objects_points_db[object_name]["R_max_cov"]: #Associate points if the distance is less than the maximum covariance 
                reactive_points_indices.append(np.argmin(dist_db))
                updated_reactive_objects.append(point)
            else: 
                track_id = next_id
                max_cov =self.objects_points_db[object_name]["R_max_cov"]
                if log:
                    print("adding new object: ",next_id)
                new_reactive_objects_indices.append(next_id)
                new_reactive_objects.append(np.hstack((point, 0.0,max_cov, track_id)))
                next_id +=1
        new_reactive_objects = np.array(new_reactive_objects)  
        updated_reactive_objects = np.array(updated_reactive_objects)    
        if log:
            print("Mapped reactives: ", len(reactive_points_indices))
            print("New reactives: ", len(new_reactive_objects))

        #TODO: Delete objects that are not in the FOV of the ego (elliminate noise)

        if len(reactive_points_indices) != 0: #if there are objects to update
            # 1. Add hits to db objects seen in the reactive map, assure that hits are within max_hits and min_hits (HITS)
            self.objects_points_db[object_name]["objects"][reactive_points_indices,3] = self.objects_points_db[object_name]["objects"][reactive_points_indices,3] + self.objects_points_db[object_name]["add_hit"]
            self.objects_points_db[object_name]["objects"][np.argwhere(self.objects_points_db[object_name]["objects"][:,3] > self.objects_points_db[object_name]["max_hits"]),3] = self.objects_points_db[object_name]["max_hits"]
            self.objects_points_db[object_name]["objects"][np.argwhere(self.objects_points_db[object_name]["objects"][:,3] < self.objects_points_db[object_name]["min_hits"]),3] = self.objects_points_db[object_name]["min_hits"]   
            # 2. Update objects positions with moving average "updated_pos = (1-alpha)*new_pos + alpha*old_pos" (POSITIONS)
            self.objects_points_db[object_name]["objects"][reactive_points_indices, :3] = (1-self.objects_points_db[object_name]["alpha"])*updated_reactive_objects + self.objects_points_db[object_name]["alpha"]*self.objects_points_db[object_name]["objects"][reactive_points_indices, :3]

        # 3. Update covariance of all objects in db, assure that covariance is between R_cov_max and R_cov_min (COVARIANCE)
        self.objects_points_db[object_name]["objects"][:, 4] = self.objects_points_db[object_name]["R_max_cov"] - self.objects_points_db[object_name]["delta_cov"]*self.objects_points_db[object_name]["objects"][:, 3]

        self.objects_points_db[object_name]["objects"][np.argwhere(self.objects_points_db[object_name]["objects"][:,4] > self.objects_points_db[object_name]["R_max_cov"]),4] = self.objects_points_db[object_name]["R_max_cov"]
        self.objects_points_db[object_name]["objects"][np.argwhere(self.objects_points_db[object_name]["objects"][:,4] < self.objects_points_db[object_name]["R_min_cov"]),4] = self.objects_points_db[object_name]["R_min_cov"]


        # 4. Add new objects to database
        if new_reactive_objects.shape[0] != 0: #if there are new objects
            self.objects_points_db[object_name]["objects"] = np.vstack((self.objects_points_db[object_name]["objects"], new_reactive_objects))
        # 5. Update old objects in db if local
        #get local objects
        #TODO: Get local objects in car fov using ego localization R and T 
        car_loc = self.get_ego_loca(timestamp)

        local_conds = np.logical_and(self.objects_points_db[object_name]["objects"][:,0:2]>=(car_loc[0,0]-self.reigon, car_loc[0,1]-self.reigon),self.objects_points_db[object_name]["objects"][:,0:2]<=(car_loc[0,0]+self.reigon, car_loc[0,1]+self.reigon))
        local_objects_indices = np.argwhere(np.logical_and(local_conds[:,0],local_conds[:,1])).T[0]
        
        #get missed objects
        missed_objects_indices = [idx for idx in local_objects_indices if idx not in reactive_points_indices+new_reactive_objects_indices]
        if log:
            print("------------------------------------------------")
            print("local_objects_indices: ",local_objects_indices)
            print("reactive_points_indices: ",reactive_points_indices)
            print("new_reactive_objects_indices: ",new_reactive_objects_indices)
            print("missed_objects_indices: ",missed_objects_indices)
            print("------------------------------------------------")
        #update missed objects covariances
        if len(missed_objects_indices)!=0:
            self.objects_points_db[object_name]["objects"][missed_objects_indices,3] = self.objects_points_db[object_name]["objects"][missed_objects_indices,3] - self.objects_points_db[object_name]["sub_hit"]
        #6. Delete objects where hits are less than minimum hits
        print("objects to be removed: ",  np.argwhere(self.objects_points_db[object_name]["objects"][:,3] <= self.objects_points_db[object_name]["min_hits"]))
        self.objects_points_db[object_name]["objects"] = np.delete(self.objects_points_db[object_name]["objects"], np.argwhere(self.objects_points_db[object_name]["objects"][:,3] <= self.objects_points_db[object_name]["min_hits"]), 0)
        return self.objects_points_db[object_name]["objects"]

    def get_ego_loca(self, timestamp):
        return self.transformer.transform_sweep_stamp_base2world(timestamp,np.array([[0.0,0.0,0.0]]).T).T
    
    def get_front_fov_loca(self, timestamp):
        front_angle = (self.FOV+90) % 360
        origin = self.transformer.transform_sweep_stamp_base2world(timestamp,np.array([[0.0,0.0,0.0]]).T).T
        b = self.transformer.transform_sweep_stamp_base2world(timestamp,np.array([[self.FOV_depth * np.cos(front_angle * np.pi / 180) , self.FOV_depth * np.sin(front_angle * np.pi / 180) ,0]]).T).T
        c = self.transformer.transform_sweep_stamp_base2world(timestamp,np.array([[self.FOV_depth * np.cos((front_angle-self.reigon) * np.pi / 180) , self.FOV_depth * np.sin((front_angle-self.reigon) * np.pi / 180),0]]).T).T
        d = self.transformer.transform_sweep_stamp_base2world(timestamp,np.array([[self.FOV_depth * np.cos((front_angle-2*self.reigon) * np.pi / 180) , self.FOV_depth * np.sin((front_angle-2*self.reigon) * np.pi / 180),0]]).T).T
        e = self.transformer.transform_sweep_stamp_base2world(timestamp,np.array([[self.FOV_depth * np.cos((front_angle-3*self.reigon) * np.pi / 180) , self.FOV_depth * np.sin((front_angle-3*self.reigon) * np.pi / 180),0]]).T).T
        f = self.transformer.transform_sweep_stamp_base2world(timestamp,np.array([[self.FOV_depth * np.cos((front_angle-4*self.reigon) * np.pi / 180) , self.FOV_depth * np.sin((front_angle-4*self.reigon) * np.pi / 180),0]]).T).T
        
        
        pts = [list(origin[0,:2]),list(b[0,:2]),list(c[0,:2]),list(d[0,:2]),list(e[0,:2]),list(f[0,:2]),list(origin[0,:2])]
        return pts
    
        
    def get_rear_fov_loca(self, timestamp):
        rear_angle = (self.FOV-90) % 360
        origin = self.transformer.transform_sweep_stamp_base2world(timestamp,np.array([[0.0,0.0,0.0]]).T).T
        b = self.transformer.transform_sweep_stamp_base2world(timestamp,np.array([[self.FOV_depth * np.cos(rear_angle * np.pi / 180) , self.FOV_depth * np.sin(rear_angle * np.pi / 180) ,0]]).T).T
        c = self.transformer.transform_sweep_stamp_base2world(timestamp,np.array([[self.FOV_depth * np.cos((rear_angle-self.reigon) * np.pi / 180) , self.FOV_depth * np.sin((rear_angle-self.reigon) * np.pi / 180),0]]).T).T
        d = self.transformer.transform_sweep_stamp_base2world(timestamp,np.array([[self.FOV_depth * np.cos((rear_angle-2*self.reigon) * np.pi / 180) , self.FOV_depth * np.sin((rear_angle-2*self.reigon) * np.pi / 180),0]]).T).T
        e = self.transformer.transform_sweep_stamp_base2world(timestamp,np.array([[self.FOV_depth * np.cos((rear_angle-3*self.reigon) * np.pi / 180) , self.FOV_depth * np.sin((rear_angle-3*self.reigon) * np.pi / 180),0]]).T).T
        f = self.transformer.transform_sweep_stamp_base2world(timestamp,np.array([[self.FOV_depth * np.cos((rear_angle-4*self.reigon) * np.pi / 180) , self.FOV_depth * np.sin((rear_angle-4*self.reigon) * np.pi / 180),0]]).T).T
        pts = [list(origin[0,:2]),list(b[0,:2]),list(c[0,:2]),list(d[0,:2]),list(e[0,:2]),list(f[0,:2]),list(origin[0,:2])]
        return pts
        


if __name__ == "__main__":
    data_path = "/home/mnabail/BrightSkies/Continental/annotation/05_code/dataset/Conti/2021-SEP-23_Highway/kitti"
    object_tracker = SEM("/home/mnabail/BrightSkies/Continental/annotation/05_code/dataset/Conti/2021-SEP-23_Highway/kitti")
    object_tracker.create_object_type("test_object")
    
    # loop on frame ids and update the db with a static object
    merged_lidar_path = os.path.join(data_path,cfg.FRAME_MERGED_LIDAR)
    list_of_lidar_frames = sorted(glob.glob(merged_lidar_path+"/*"+cfg.PCD_FORMAT))[-20:]
    for lidar_frame in list_of_lidar_frames:
        frame_id = lidar_frame.split("/")[-1]
        frame_id = frame_id.replace(cfg.PCD_FORMAT,'')
        print(frame_id)
        object_tracker.update_db("test_object",np.array([[0.1,0.3,0.4],[0.6,0.677,0.4],[0.5,0.456,0.4],[0.5,0.22,0.4]]), frame_id)
        print(object_tracker.objects_points_db["test_object"]["objects"][:,:3])