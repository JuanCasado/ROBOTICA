global pub_vel msg_vel keep_looping

rosconnect();
amigobotInit();

rate = robotics.Rate(10);

keep_looping = 1;
while(keep_looping)
    updateState();
    displayRobot();
    control();
    send(pub_vel, msg_vel);
    waitfor(rate);
end

clear;
rosshutdown()