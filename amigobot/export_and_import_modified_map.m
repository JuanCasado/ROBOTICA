
clear all
close all

% GUARDAR EL MAP
load rooms_slam.mat

fig = figure(1);
show(map)

axis('off')
title('')
ylabel('')
xlabel('')


imwrite((1.-map.occupancyMatrix),'map_original.png')


axis('on')


%Editar con un editor de imagen (GIMP...) sin cambiar la resolucion, ni el
%tama�o

image = imread('map_borrado.png');

% Unknown areas (gray) should be removed and treated as free space. Create
% a logical matrix based on a threshold. Depending on your image, this value
% could be different. Occupied space should be set as 1 (white in image).

imageNorm = double(image)/255;
imageOccupancy = 1 - imageNorm;

% Create OccupancyGrid object using adjusted map image.

map_cleaned = robotics.OccupancyGrid(imageOccupancy ,map.Resolution);  %pixels/metros(medidos antes de convertir a imagen)

%locate the new map in the same word coordinates than the original
map_cleaned.GridLocationInWorld = map.GridLocationInWorld

figure(12)

show(map_cleaned)





