clear
clc
close all

map=imread('13.jpg');

% map=map(1:1730,1:2517,:);
map=map(1:732,1:1001,:);


START=map;
figure
d=imshow(map,[]);
impixelinfo(d)
% for i=1:size(map,1)
%     for j=1:size(map,2)
%         if map(i,j,1)<150 && map(i,j,2)<150 && map(i,j,3)<150
%             map(i,j,1)=255;
%             map(i,j,2)=255;
%             map(i,j,3)=255;
%         end
%     end
% end
% figure
% imshow(map,[]);
