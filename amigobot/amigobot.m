
function amigobot(id)
    global pub_vel msg_vel keep_looping
    
    rosconnect(id);
    if id == 0
        simuladorInit();
    elseif id == 1
        robotInit();
    end
    rate = robotics.Rate(20);
    
    keep_looping = 1;
    while(keep_looping)
        updateState();
        displayRobot();
        %control();
        move(1,0);
        send(pub_vel, msg_vel);
        waitfor(rate);
    end
    
    rosdisconnect()
end

