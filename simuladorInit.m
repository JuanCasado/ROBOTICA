
global sub_odom sub_laser ...
    sub_sonar0 sub_sonar1 sub_sonar2 sub_sonar3 sub_sonar4 sub_sonar5 sub_sonar6 sub_sonar7 ...
    pub_vel msg_vel msg_init_pos acc_distance yawinit

%Odometría SIN ruido
%sub_odom = rossubscriber('/robot0/odom', 'nav_msgs/Odometry');
%Odometría CON ruido
sub_odom = rossubscriber('/robot0/local_odom', 'nav_msgs/Odometry');
sub_laser = rossubscriber('/robot0/laser_1', 'sensor_msgs/LaserScan');
sub_sonar0 = rossubscriber('/robot0/sonar_0', 'sensor_msgs/Range');
sub_sonar1 = rossubscriber('/robot0/sonar_1', 'sensor_msgs/Range');
sub_sonar2 = rossubscriber('/robot0/sonar_2', 'sensor_msgs/Range');
sub_sonar3 = rossubscriber('/robot0/sonar_3', 'sensor_msgs/Range');
sub_sonar4 = rossubscriber('/robot0/sonar_4', 'sensor_msgs/Range');
sub_sonar5 = rossubscriber('/robot0/sonar_5', 'sensor_msgs/Range');
sub_sonar6 = rossubscriber('/robot0/sonar_6', 'sensor_msgs/Range');
sub_sonar7 = rossubscriber('/robot0/sonar_7', 'sensor_msgs/Range');

pub_vel = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');
msg_vel = rosmessage(pub_vel);
stop();

pause(0.5);

msg_init_pos = sub_odom.LatestMessage();
initori = msg_init_pos.Pose.Pose.Orientation;
euler = quat2eul([initori.W initori.X initori.Y initori.Z]);
yawinit = euler(1);
acc_distance = 0;