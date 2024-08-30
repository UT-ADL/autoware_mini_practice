[< Previous practice](../practice_7) -- [**Main Readme**](../README.md)

# Practice 8 - CARLA simulator

Task for this practice is to drive full demo lap with the help of the nodes that you have written during previous practices:
* `localizer`
* `pure_pursuit_follower`
* `lanelet2_global_planner`
* `points_clusterer` and `cluster_detector`
* `simple_local_planner`
* `camera_traffic_light_detector`

You are provided with final launch file [launch/practice_8.launch](https://owncloud.ut.ee/owncloud/s/MPgPQTKjLQDXgnr) that utilizes all previously created nodes and substitutes other useful nodes and CARLA simulator setup from `autoware_mini`. 

![rviz_carla](images/rviz_carla.png)

## 1. Run CARLA simulation

[Launching CARLA simulation](https://gitlab.cs.ut.ee/autonomous-driving-lab/autoware_mini#launch-instructions).

1. Run Carla. 
    ```
    $CARLA_ROOT/CarlaUE4.sh -prefernvidia -quality-level=Low -RenderOffScreen
    ```
2. From `autoware_mini_practice_solutions` launch `practice_8.launch`
    ```
    roslaunch autoware_mini_practice_solutions practice_8.launch tfl_detector:=camera detector:=lidar_cluster
    ```
3. Drive through the Demo lap
    - [Map of the demo lap](https://adl.cs.ut.ee/lab/demo-track)

4. If you are not able to demonstrate it in person at the practice session, please record the video and send the link.
    - You can use [SimpleScreenRecorder](https://www.geeksforgeeks.org/how-to-install-simplescreenrecorder-ubuntu/)
    - As Carla tends to run very slowly on laptop you can adjust recording framerate to be smaller, like 5Hz (or even less). Then the video won't be so big.