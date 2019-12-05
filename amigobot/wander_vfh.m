
close all
clear all

global msg_laser msg_vel pub_vel

connect_to_robot = 0;
rosconnect(connect_to_robot);
if connect_to_robot
    robotInit();
else
    simuladorInit();
end

fig_laser = figure('Name','LASER'); title('LASER');
fig_vfh = figure('Name','VFH'); title('VFH');

VFH=robotics.VectorFieldHistogram;
VFH.RobotRadius= 0.15;
VFH.SafetyDistance= 0.22;
VFH.NumAngularSectors= 200;
VFH.MinTurningRadius= 0.15;
VFH.DistanceLimits = [0.15 3];     % Lecturas mínima y máxima del laser
VFH.HistogramThresholds = [2 8];   % Tamño de los círculos morados que indican ocupación
VFH.TargetDirectionWeight=0;
VFH.CurrentDirectionWeight=1;
VFH.PreviousDirectionWeight=1;
VFH.UseLidarScan=true;

rate = robotics.Rate(20);
display(' - Iniciado - ')
while(1)
    updateState();
    figure(fig_laser);plot(msg_laser);
    theta = VFH(lidarScan(msg_laser),0);
    display(theta);
    figure(fig_vfh);VFH.show();
    move(0.1, 0)
    rotate (theta*0.1);
    send(pub_vel, msg_vel)
    
    waitfor(rate);
end

rosdisconnect()
