
close all
clear all

global sub_laser sub_odom msg_vel pub_vel

connect_to_robot = 1;
rosconnect(connect_to_robot);
if connect_to_robot
    robotInit();
    tf_laser = 'laser_frame';
    tf_robot = 'base_link';
    load slam_modificado.mat
    map = map_cleaned;
else
    simuladorInit();
    tf_laser = 'robot0_laser_1';
    tf_robot = 'robot0';
    %load simulador_slam.mat
    load slam_modificado.mat
    map = map_cleaned;
end

fig_laser = figure('Name','LASER'); title('LASER');
fig_vfh = figure('Name','VFH'); title('VFH');

fig_map = figure('Name', 'MAPA'); title('MAPA');
figure(fig_map); show(map);

VFH=robotics.VectorFieldHistogram;
VFH.RobotRadius= 0.15;
VFH.SafetyDistance= 0.25;
VFH.NumAngularSectors= 200;
VFH.MinTurningRadius= 0.10;
VFH.DistanceLimits = [0.15 3];     % Lecturas mínima y máxima del laser
VFH.HistogramThresholds = [1 4];   % Tamño de los círculos morados que indican ocupación
VFH.TargetDirectionWeight=0;
VFH.CurrentDirectionWeight=1;
VFH.PreviousDirectionWeight=1;
VFH.UseLidarScan=true;

odometryModel = robotics.OdometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];
rangeFinderModel = robotics.LikelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0 8];
rangeFinderModel.Map = map;
tftree = rostf;
waitForTransform(tftree,tf_laser, tf_robot);
sensorTransform = getTransform(tftree,tf_laser, tf_robot);
laserQuat = [sensorTransform.Transform.Rotation.W sensorTransform.Transform.Rotation.X ...
    sensorTransform.Transform.Rotation.Y sensorTransform.Transform.Rotation.Z];
laserRotation = quat2eul(laserQuat, 'ZYX');
rangeFinderModel.SensorPose = ...
    [sensorTransform.Transform.Translation.X sensorTransform.Transform.Translation.Y laserRotation(1)];
amcl = robotics.MonteCarloLocalization;
amcl.UseLidarScan = true;
amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;
amcl.UpdateThresholds = [0.2,0.2,0.2];
amcl.ResamplingInterval = 1;
amcl.ParticleLimits = [50000, 10000];
amcl.GlobalLocalization = true;
amcl.InitialPose = [6 4.5 0];
amcl.InitialCovariance = diag([1 1 1])*0.6;
visualizationHelper = ExampleHelperAMCLVisualization(map);

umbralx = 0.065;
umbraly = umbralx;
umbralyaw = umbraly;

k_rotate = 0.12;
k_vel = 0.08;
vel = 0.1;
max_vel = 0.2;
min_vel = 0.05;
safe_distance = 1;

rate = robotics.Rate(20);
display(' - Iniciado - ')
while(1)
    msg_laser = receive(sub_laser);
    odompose = sub_odom.LatestMessage;
    scans = lidarScan(msg_laser);
    
    theta = VFH(scans,0);
    min_range = min(msg_laser.Ranges);
    error_d = safe_distance - min_range;
    goto_vel = vel - error_d*k_vel;
    if goto_vel > max_vel
        goto_vel = max_vel;
    elseif goto_vel < min_vel
        goto_vel = min_vel;
    end
    move(goto_vel, 0)
    rotate (theta*k_rotate);
    send(pub_vel, msg_vel)
    
    odomQuat = [odompose.Pose.Pose.Orientation.W, odompose.Pose.Pose.Orientation.X, ...
        odompose.Pose.Pose.Orientation.Y, odompose.Pose.Pose.Orientation.Z];
    odomRotation = quat2eul(odomQuat);
    pose = [odompose.Pose.Pose.Position.X, odompose.Pose.Pose.Position.Y odomRotation(1)];
    [isUpdated,estimatedPose, estimatedCovariance] = amcl(pose, scans);
    
    if (estimatedCovariance(1,1)<umbralx && estimatedCovariance(2,2)<umbraly && estimatedCovariance(3,3)<umbralyaw)
        disp('LOCALIZADO!!!')
    end
    
    figure(fig_vfh);VFH.show();
    figure(fig_laser);plot(msg_laser);
    if isUpdated
        i = i + 1;
        plotStep(visualizationHelper, amcl, estimatedPose, scans, i)
    end
    
    waitfor(rate);
end

rosdisconnect()
