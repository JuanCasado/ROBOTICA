function rosdisconnect ()
    global pub_vel msg_vel
    stop()
    send(pub_vel, msg_vel);
    rosshutdown()
    display("DISCONNECTED FROM ROS")
end