
global msg_init_pos msg_pos msg_laser acc_distance keep_looping yawinit yaw...
msg_sonar0 msg_sonar1 msg_sonar2 msg_sonar3 msg_sonar4 msg_sonar5 msg_sonar6 msg_sonar7

dist = distance();
acc_distance = acc_distance + dist;
ang_diff = angdiff(yawinit, yaw);
min_range = min(msg_laser.Ranges);
%display(acc_distance)
%display(ang_diff)
%display(min_range)

if  acc_distance > 50
    stop()
    keep_looping = 0;
else
    if min_range > 0.1
        %rotate(2)
        move(0.2, 0)
    else
        stop()
    end
end
