# motorai_tasks

## Task1
### Pre-Challenge Question-:
##### 1. Please mention three significant advantages and disadvantages of the camera and Lidar.
##### 2. Why is Sensor Fusion necessary?
1.
    -> Camera has higher resolution and no depth information when compared to LiDAR.
    -> Camera gives color information and is low cost. LiDAR doesn't give color information and is expensive.
    -> LiDAR does not have the ability to see through bad weather.
    -> Camera is effected by sunlight.
2.  
    As every sensor has its own pros and cons. Making using of each others pros gives roboust data. This can be effectively done through sensor fusion.


### Post-Challenge Questions-:
##### Do you feel that the projection of Lidar points on an image is necessary? If yes, How can it help in performing the scene understanding? If not also, explain Why?

Yes, because Image has higher resolution than lidar, classifying the image is more efficient because it has more features and utilizing the depth information from LiDAR. All these factors help in seeing the scene in 3D more effectively. 

## Task2
### Pre-Challenge Question-:
##### Give some advantages and disadvantages of RADAR over LiDAR:
1. RADAR has long-range compared to LiDAR.
2. It works at night and during the cloudy weather conditions.
3. It can calculate the velocity of the object.

### Post-Challenge Questions-:
##### Please provide the reasoning behind the filter you selected. Are there some other filters as well, which could have been selected?

I am choosing Extended Kalman Filter (EKF). As we know Lidar gives pose_x and pose_y which needs kalman filter (as it is linear). But Radar gives additional vel_x, vel_y which is non-linear state equation, Therefore making using of non-linear filter like EKF helps to solve this problem effectively. There is some other filter Unscented kalman filter (UKF) can be used.




# Comments

I have couple of questions regarding tasks.

1. Both the task instructions are not clear.
2. In Task1, Co-ordinate system is not specified. Where I spent whole day thinking what is wrong with my procedure.
3. In Task1, You mentioned datas are synchronised. But they are not. (I think you, should give image data after two frames.)
4. Last this is huge. You gave me the data for Task2 which is truly unclear. When some data is given, There should be proper instructions.
5. I didn't have time to finish the Task2. As instructions are not clear. 

