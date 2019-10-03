
global msg_pos msg_laser msg_sonar0 msg_sonar1 msg_sonar2 msg_sonar3 ...
    msg_sonar4 msg_sonar5 msg_sonar6 msg_sonar7 sub_laser sub_sonar0 ...
    sub_sonar1 sub_sonar2 sub_sonar3 sub_sonar4 sub_sonar5 sub_sonar6 ...
    sub_sonar7  sub_odom yaw

msg_pos = sub_odom.LatestMessage();
msg_laser = sub_laser.LatestMessage();
msg_sonar0 = sub_sonar0.LatestMessage();
msg_sonar1 = sub_sonar1.LatestMessage();
msg_sonar2 = sub_sonar2.LatestMessage();
msg_sonar3 = sub_sonar3.LatestMessage();
msg_sonar4 = sub_sonar4.LatestMessage();
msg_sonar5 = sub_sonar5.LatestMessage();
msg_sonar6 = sub_sonar6.LatestMessage();
msg_sonar7 = sub_sonar7.LatestMessage();

ori = msg_pos.Pose.Pose.Orientation;
euler = quat2eul([ori.W ori.X ori.Y ori.Z]);
yaw = euler(1);