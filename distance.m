
function d = distance ()
    global msg_init_pos msg_pos
    d = ((msg_init_pos.Pose.Pose.Position.X - msg_pos.Pose.Pose.Position.X)^2 + ...
                (msg_init_pos.Pose.Pose.Position.Y - msg_pos.Pose.Pose.Position.Y)^2)^(1/2);
end