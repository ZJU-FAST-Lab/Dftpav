roslaunch phy_simulator rviz.launch            & sleep 1; # vis
roslaunch phy_simulator phy_simulator_parking.launch & sleep 1; # simulator
roslaunch ai_agent_planner moving_obstacle.launch & sleep 1.0;
roslaunch planning_integrated map.launch & sleep 1; # just for visualization
roslaunch planning_integrated park.launch & sleep 1; # focus this!
wait

