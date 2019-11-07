%% Implement Online Simultaneous Localization And Mapping (SLAM) with Lidar Scans
%% Introduction
% This example demonstrates how to implement the Simultaneous Localization And 
% Mapping (SLAM) algorithm on lidar scans using pose graph optimization. 
% 
% The goal of this example is to build a map of the environment using the lidar 
% scans and retrieve the trajectory of the robot. To obtain the map, the robot 
% must be teleoperated throughout the environment during the execution of this 
% script, going through the same place several times.
%% Create Lidar Slam Object
% Create a <docid:robotics_ref.mw_d7ab99c9-fd98-4516-8932-2a69004eaaba |robotics.LidarSLAM|> 
% object and set the map resolution and the max lidar range. Set the max lidar 
% range smaller than the max scan range, as the laser readings are less accurate 
% near max range. Set the grid map resolution to 20 cells per meter, which gives 
% a 5cm precision. 

maxLidarRange = 8;
mapResolution = 20;
slamAlg = robotics.LidarSLAM(mapResolution, maxLidarRange);
%% 
% The loop closure parameters are set empirically. Using a higher loop closure 
% threshold helps reject false positives in loop closure identification process. 
% Keep in mind that a high-score match may still be a bad match. For example, 
% scans collected in an environment that has similar or repeated features are 
% more likely to produce false positive. Using a higher loop closure search radius 
% allows the algorithm to search a wider range of the map around the current pose 
% estimate for loop closures.

slamAlg.LoopClosureThreshold = 200;
slamAlg.LoopClosureSearchRadius = 8;
%% Observe the Effect of Loop Closure and Optimization Process
% Create a loop to map the environment while localizing the robot (SLAM process). 
% The robot position is updated in the loop from the lidar scans. The scans are 
% obtained from the robot as robot navigates through the environment.
% 
% Loop closures are automatically detected as the robot moves. The pose graph 
% optimization is performed whenever a loop closure is detected. This can be checked 
% using the output |optimizationInfo.IsPerformed| value from |addScan|.
% 
% A snapshot is shown to demonstrate of the scans and poses when the first loop 
% closure is identified and verify the results visually. This plot shows overlaid 
% scans and an optimized pose graph for the first loop closure.
% 
% The final built map would be presented after all the scans are collected and 
% processed.
% 
% The plot is updated continuously as robot navigates through virtual scene

firstLoopClosure = false;
sub_laser = rossubscriber('/scan', 'sensor_msgs/LaserScan');
r = robotics.Rate(20);
figure
while (1)
    % Read the laser message
    msg_laser = sub_laser.LatestMessage();

    % Create lidarScan object from scan message
    scan = lidarScan(msg_laser);
      
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scan);
    if isScanAccepted
        % Visualize how scans plot and poses are updated as robot navigates
        % through virtual scene
        show(slamAlg);
        
        % Visualize the first detected loop closure
        % firstLoopClosure flag is used to capture the first loop closure event
        if optimizationInfo.IsPerformed && ~firstLoopClosure
            firstLoopClosure = true;
            show(slamAlg, 'Poses', 'off');
            hold on;
            show(slamAlg.PoseGraph);
            hold off;
            title('First loop closure');
            snapnow
        end
    end

    waitfor(r);
end
%%
% Plot the final built map after all scans are added to the |slamAlg|
% object.
show(slamAlg, 'Poses', 'off'); 
hold on;
show(slamAlg.PoseGraph); 
hold off;
title({'Final Built Map of the Environment', 'Trajectory of the Robot'});
%Build Occupancy Grid Map
% The optimized scans and poses can be used to generate a <docid:robotics_ref.bvaw60t-1 
% |robotics.OccupancyGrid|> which represents the environment as a probabilistic 
% occupancy grid.

[scans, optimizedPoses]  = scansAndPoses(slamAlg);
map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);

display("DONE")
%% 
% Visualize the occupancy grid map populated with the laser scans and the optimized 
% pose graph.

figure; 
show(map);
hold on
show(slamAlg.PoseGraph, 'IDs', 'off');
hold off
title('Occupancy Grid Map Built Using Lidar SLAM');


%% 
% Save map in a .mat file 

save mi_mapa.mat map