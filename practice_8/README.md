[< Previous practice](../practice_7) -- [**Main Readme**](../README.md)

# Practice 8 - CARLA simulator with Scenario Runner

The task for this practice is to test your autonomous driving framework in the Carla simulator by driving through the demo lap and then evaluating on the automated Route Scenario. You will use all the following nodes representing different modules of the autonomous driving stack written during previous practices:

* `localizer` - Localization module
* `points_clusterer` and `cluster_detector` - Perception module
* `camera_traffic_light_detector` - Traffic light detection module
* `lanelet2_global_planner` - Global planner module
* `local_path_extractor`,`collision_points_manager`, and `simple_speed_planner` - Local path extraction, collision points management, and speed planning modules
* `pure_pursuit_follower` - Controller module

You are provided with final launch files and RViz config file [rviz/practice_8.rviz](rviz/practice_8.launch) that utilizes all mentioned nodes and substitutes other useful nodes and CARLA simulator setup from `autoware_mini`. 

#### Additional files
- [practice_8_full_sensors.launch](launch/practice_8_full_sensors.launch) - Launch file that includes all modules from `autoware_mini_practice_solutions` package.
- [practice_8_route_scenario.launch](launch/practice_8_route_scenario.launch) - Launch file that includes all modules from `autoware_mini_practice_solutions` package and launches Scenario Runner with pre-set route scenario, however, substitutes perception and traffic light detection modules with the ground truth readings provided by CARLA simulator.
- [rviz/practice_8.rviz](rviz/practice_8.rviz) - RViz config file for visualizing of the topics.

![rviz_carla](images/rviz_carla.png)

## 1. Run the CARLA simulation and record the demo lap

[Launching CARLA simulation](https://github.com/UT-ADL/autoware_mini?tab=readme-ov-file#launching-carla-simulation).

1. Run Carla. 
    ```
    $CARLA_ROOT/CarlaUE4.sh -prefernvidia -quality-level=Low -RenderOffScreen
    ```
2. From `autoware_mini_practice_solutions` launch `practice_8_full_sensors.launch`. The launch file will initialize all the nodes from your autonomous driving framework and connect to the CARLA simulator sensors that your perception modules will process.
    ```
    roslaunch autoware_mini_practice_solutions practice_8_full_sensors.launch tfl_detector:=camera detector:=cluster
    ```
3. Drive through the Demo lap
    - [Map of the demo lap](https://adl.cs.ut.ee/lab/demo-track)

4. If you cannot demonstrate it in person at the practice session, please record the video and send the link.
    - You can use [SimpleScreenRecorder](https://www.geeksforgeeks.org/how-to-install-simplescreenrecorder-ubuntu/)
    - As Carla tends to run very slowly on old hardware, you can adjust the recording framerate to be smaller, like 5Hz (or even less). Then the video won't be so big.
   
## 2. Run the CARLA simulation with Scenario Runner and evaluate your framework

1. Re-run Carla. It is good practice to restart the Carla simulator between every execution of the launch files.
    ```
    $CARLA_ROOT/CarlaUE4.sh -prefernvidia -quality-level=Low -RenderOffScreen
    ```
2. From `autoware_mini_practice_solutions` launch `practice_8_route_scenario.launch` to launch Carla simulator with Scenario Runner and the pre-set route scenario from `autoware_mini` package. As the route scenario is running too slow with all sensors and perception module enabled, the launch file will substitute perception modules from your autonomous driving framework with the ground truth readings provided by the CARLA simulator.
    ```
    roslaunch autoware_mini_practice_solutions practice_8_route_scenario.launch use_scenario_runner:=true route_id:=1
    ```
3. You may notice that in the scenario when the first vehicle is cutting us from the left, there could be a collision. Navigate to the scenario file in the main autoware mini repository `tartu_demo.xml`, find the route with `id=1` and there:
   - Find the first instance of scenario with type `VehicleFromTheLeft1`.
   - Look at the trigger point coordinates, these coordinates are in the map frame and corresponds to the ego vehicle positions.
   - In the RViz, in the "View" tab. you can change the target frame to "map" and then you will see the ego vehicle coordinates in the map frame.
   - Change the trigger point coordinates in the scenario file so it triggers a bit earlier and doesn't cause a collision. (hint, you need to decrease slightly both `x` and `y` coordinates)
4. Rerun the scenario and confirm that the vehicle is still cutting from left but the collision is avoided.
5. Once the route scenario finishes, it will output final results (see below for example). Save the simulation information and the Criteria information in "practice_8_simulation.txt".
![results_example](images/results_example.png)

6. Commit the results in your repo and send the video to the instructor if necessary.