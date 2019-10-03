

global sub_odom sub_laser ...
    sub_sonar0 sub_sonar1 sub_sonar2 sub_sonar3 sub_sonar4 sub_sonar5 sub_sonar6 sub_sonar7 ...
    pub_vel msg_vel msg_init_pos acc_distance pub_motor_state msg_motor_state yawinit

%Odometría SIN ruido
%sub_odom = rossubscriber('/robot0/odom', 'nav_msgs/Odometry');
%Odometría CON ruido
sub_odom = rossubscriber('/pose', 'nav_msgs/Odometry');
sub_laser = rossubscriber('/scan', 'sensor_msgs/LaserScan');
sub_sonar0 = rossubscriber('/sonar_0', 'sensor_msgs/Range');
sub_sonar1 = rossubscriber('/sonar_1', 'sensor_msgs/Range');
sub_sonar2 = rossubscriber('/sonar_2', 'sensor_msgs/Range');
sub_sonar3 = rossubscriber('/sonar_3', 'sensor_msgs/Range');
sub_sonar4 = rossubscriber('/sonar_4', 'sensor_msgs/Range');
sub_sonar5 = rossubscriber('/sonar_5', 'sensor_msgs/Range');
sub_sonar6 = rossubscriber('/sonar_6', 'sensor_msgs/Range');
sub_sonar7 = rossubscriber('/sonar_7', 'sensor_msgs/Range');

pub_vel = rospublisher('/cmd_vel', 'geometry_msgs/Twist');
msg_vel = rosmessage(pub_vel);
pub_motor_state = rospublisher('/cmd_motor_state', 'std_msgs/Int32');
msg_motor_state = rosmessage(pub_motor_state);
stop();

pause(0.5);

msg_motor_state.Data = 1;
send(pub_motor_state, msg_motor_state);
msg_init_pos = sub_odom.LatestMessage();
initori = msg_init_pos.Pose.Pose.Orientation;
euler = quat2eul([initori.W initori.X initori.Y initori.Z]);
yawinit = euler(1);
acc_distance = 0;