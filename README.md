##Static Environment Mapping

The map of the environment is created with respect to a certain starting frame.
The localization module gives the position of the center of the car with respect to global frame at any time ‘t’.
On the other hand, the perception module gives the positions of static objects in the environment with respect to the ego frame using lidar points or stereo vision.
This mapping module takes the static objects list provided by perception module and uses the filtered odometry given by localization module to place the detected objects in the global frame.


###Global Map Database
Global map of static objects has to be absolutely correct, because all the long term decisions of the car are based on this map. If a car sees a static object somewhere it shouldn't, it might divert towards it or stop pre-maturely. In worst case, it might crash into something which was perhaps missed or make false decisions. Reactive mapping provides a pre-filtering layer for entry to global database. But for global db, we need to go further.
First thought is that a reactive object should be added to global db only if it was seen repeatedly. But then the map creation would be slow and disjointed (objects appear and disappear without obvious reason). Also, navigation suggested keeping a covariance measure of position of these. But in the covariance measure, we also wanted to encode the logic that if an object is seen many times, the covariance decreases. So we divided covariance measure into two parts:

Every new object in global db starts with max covariance radius. With time, if it's seen again, its 'hits' increase and proportionately, the covariance radius decreases.
The object position is updated using exponential moving average. Hence, every object is characterized by its position (moving on an avg) and its covariance radius

But that's not enough to remove spurious outliers that might creep into global map (they always do). Mapping needs to have a temporary memory. Or in other words, a map should be able to forget. Hence, if a global object (i.e. an object in global db) is in field of view of car (field of detection) but is not seen frequently, then its hits reduce. Consequently, its covariance radius increases, till it's removed from global db. An analogy might be that every new object starts off as a fat balloon which loses air if it's seen frequently. But if it isn't seen anymore, then it inflates once more and 'pops' out of global map.
Every object is also given a unique id, which helps in further tracking of the object.

###Workflow
With this background in mind, the update_db(static_objects, ego_pose) function has following logic:
Firstly, only those reactive objects which are in field of view (FOV) of car are chosen.
The list of objects in camera frame is transformed to global frame using R and T from the localization module.
If the database is empty, add all reactive objects in the FOV to it.
reactive_in_fov_map: reactive objects lies in the fov of the vehicle and has an association in the global map db)
db_in_reactive: successfully associated objects between reactive objects and database objects
db_local_in_distance_index: objects which are within FOV circle of car as distance
db_local_in_angle_index: objects which are within FOV circle of car as angle
Else, for reactive_in_fov_map, we do data association in global db. In other words, for each object in reactive_in_fov_map, try to find an object in global db which is close enough (closeness is defined by self.objects_points_db[object_name]["R_max_cov"]). If you find a close enough object in the global database, that means it was seen again by the car. For this object, add the hits, update its position (moving average) and covariance radius. The list of global objects which are successfully associated is stored in db_in_reactive.
All the objects in reactive_in_fov_map which were not associated with an object in global db are new. So, these are added as new objects to global db. Every new object is also assigned a unique id.
Now, to encode the 'map forgetting' logic, we need to find all those objects in global db, which are within field of view (FOV) of car, but were not associated with any reactive object. To do this, we transform the object db to local coordinates (again, using R and T but the other way this time). Then we first find those objects which are within FOV circle of car (stored in db_local_in_distance_index). Then we find a subset in db_local_in_distance_index which is within FOV angle of car (stored in db_local_in_angle_index). Lastly, those objects which are in db_local_in_angle_index, but not db_in_reactive are the ones for which hits are subtracted. If hits hit a minimum, those objects are removed from global db.
