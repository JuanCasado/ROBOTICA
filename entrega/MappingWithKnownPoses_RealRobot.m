
tftree = rostf;

pause(1);

map = robotics.OccupancyGrid(100,100,20);
map.GridLocationInWorld = [-50, -50];

figureHandle = figure('Name', 'Map');
axesHandle = axes('Parent', figureHandle);
mapHandle = show(map, 'Parent', axesHandle);
title(axesHandle, 'OccupancyGrid: Update 0');
updateCounter = 1;
r = robotics.Rate(20);
while(1)
    msg_laser = sub_laser.LatestMessage;
    pose = getTransform(tftree, 'odom', 'base_link', msg_laser.Header.Stamp, 'Timeout', 10);
    
    position = [pose.Transform.Translation.X, pose.Transform.Translation.Y];
    orientation =  quat2eul([pose.Transform.Rotation.W, pose.Transform.Rotation.X, ...
        pose.Transform.Rotation.Y, pose.Transform.Rotation.Z], 'ZYX');
    robotPose = [position, orientation(1)];
    ranges = msg_laser.Ranges;
    len_ranges = size(ranges);
    angles = linspace(msg_laser.AngleMin,msg_laser.AngleMax,len_ranges(1));
    ranges(isinf(ranges)) = 8;  %Eliminar datos infinitos
    insertRay(map, robotPose, ranges, angles, 8);  %Es un método de la clase OccupancyGrid. Consular la ayuda.
    if ~mod(updateCounter,50)
        mapHandle.CData = occupancyMatrix(map);
        title(axesHandle, ['OccupancyGrid: Update ' num2str(updateCounter)]);
    end
    updateCounter = updateCounter+1;
    waitfor(r);
end
show(map, 'Parent', axesHandle);
title(axesHandle, 'OccupancyGrid: Final Map');

save mi_mapa.mat map