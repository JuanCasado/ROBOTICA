
global msg_init_pos msg_pos msg_laser acc_distance keep_looping yawinit yaw...
msg_sonar0 msg_sonar1 msg_sonar2 msg_sonar3 msg_sonar4 msg_sonar5 msg_sonar6 msg_sonar7

dist = distance();
acc_distance = acc_distance + dist;
ang_diff = angdiff(yawinit, yaw);
display(acc_distance)
display(ang_diff)

if  ang_diff > 1
    stop()
    keep_looping = 0;
else
    rotate(2)
    %move(1, 1)
end
