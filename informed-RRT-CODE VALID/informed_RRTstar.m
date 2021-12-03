%***************************************
%Author:Salva N.OBAMA
%Date: 2021-09-12
%***************************************
%% Informed RRT*
clc;
clear all; close all;
tic

A=imread('22.png');
A=double(A);
B=zeros(size(A,2),size(A,2));
for i=1:size(A,1)
    for j=1:size(A,2)
        if A(i,j,1)>250 & A(i,j,2)>250 & A(i,j,2)>250
            B(i,j)=255;
            C(i,j)=255;
        end
        if abs(A(i,j,1)-A(i,j,2))<8 & abs(A(i,j,1)-A(i,j,3))<8 & abs(A(i,j,3)-A(i,j,2))<8
            B(i,j)=255;
            C(i,j)=255;
        end
    end
end

imBw = im2bw(B);                        %Convertir a imagen binaria
imLabel = bwlabel(imBw);                %Etiqueta cada dominio conectado
stats = regionprops(imLabel,'Area');    %Encuentra el tamaño de cada dominio conectado
area = cat(1,stats.Area);
index = find(area == max(area));        %Encuentre el índice del dominio conectado más grande
img = ismember(imLabel,index);          %Obtenga la imagen de dominio conectado más grande
% figure
% imshow(img,[]);
IMG=1-img;
imBw = im2bw(IMG);                        %Convertir a imagen binaria
imLabel = bwlabel(imBw);                %Etiqueta cada dominio conectado
stats = regionprops(imLabel,'Area');    %Encuentra el tamaño de cada dominio conectado
area = cat(1,stats.Area);
index = find(area >200);        %Encuentre el índice del dominio conectado más grande
img = ismember(imLabel,index);          %Obtenga la imagen de dominio conectado más grande
img=1-img;
se=strel('disk',2);
img=imdilate(img,se);

img = 255*uint8(img);
C=img(1:size(A,1),1:size(A,2));
% figure
% imshow(C,[]);
clearvars -except img C A
%  x_I=177; y_I=359;           
%  x_G=1832; y_G=856;  

x_I=10; y_I=355;      %inicio  
x_G=1626; y_G=518;

% x_I=10; y_I=355;      %inicio
% x_G=1256; y_G=359;

% x_I=10; y_I=355;      %inicio
% x_G=674; y_G=356;   %final
Thr=50; 
Delta= 90;  

%%
T.v(1).x = x_I;         
T.v(1).y = y_I; 
T.v(1).xPrev = x_I;     
T.v(1).yPrev = y_I;
T.v(1).dist=0;          
T.v(1).indPrev = 0;     
%% figura
figure;
Imp=img;
% imshow(Imp)
imshow(C,[])
title('Search informed RRT* path Map');
xL=size(Imp,1);
yL=size(Imp,2);
hold on
plot(x_I, y_I, 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');
count=1;
goal = [x_G,y_G];
start_goal_dist = 100000;
path.pos(1).x = 700;
path.pos(1).y = 700;
for iter = 1:100000
    x_rand=[];
    %%=========采样想x_rand========%%
    if start_goal_dist < 1000000
        while 1
            x_rand(2) = xL*rand; 
            x_rand(1) = yL*rand; 
            if new_node(x_rand(2),x_rand(1),start_goal_dist)
                break;
            end
        end
    else
        x_rand(1) = xL*rand; 
        x_rand(2) = yL*rand;
    end
    
    %%=======Buscar x_near===========%%
    x_near=[];
    min_dist = 10000000;
    near_iter = 1;
    near_iter_tmp = 1;
    [~,N]=size(T.v);
    for j = 1:N
       x_near(1) = T.v(j).x;
       x_near(2) = T.v(j).y;
       dist = norm(x_rand - x_near);
       if min_dist > dist
           min_dist = dist;
           near_iter = j;
       end
    end
    x_near(1) = T.v(near_iter).x;
    x_near(2) = T.v(near_iter).y;
    %%========Obtener x_new============%%
    x_new=[];
    near_to_rand = [x_rand(1)-x_near(1),x_rand(2)-x_near(2)];
    normlized = near_to_rand / norm(near_to_rand) * Delta;
    x_new = x_near + normlized;
    %%=======Detección de obstáculos===============%%
    if ~collisionChecking(x_near,x_new,Imp) 
       continue;
    end
    
    %%=======  nearC && chooseParent  =========%%
    nearptr = [];
    nearcount = 0;
    neardist =  norm(x_new - x_near) + T.v(near_iter_tmp).dist;  
    for j = 1:N
       if j == near_iter_tmp
           continue;
       end
       x_neartmp(1) = T.v(j).x;
       x_neartmp(2) = T.v(j).y;
       dist = norm(x_new - x_neartmp) + T.v(j).dist;
       norm_dist = norm(x_new - x_neartmp);
       if norm_dist < 120
           %nearC
           if collisionChecking(x_neartmp,x_new,Imp)
                nearcount = nearcount + 1;
                nearptr(nearcount,1) = j;
                if neardist > dist 
                    neardist = dist;
                    near_iter = j;
                end
           end
       end
    end
    
    x_near(1) = T.v(near_iter).x;
    x_near(2) = T.v(near_iter).y;
    count=count+1;
    %%========Agrega X_NEW al árbol========%%
    T.v(count).x = x_new(1);
    T.v(count).y = x_new(2); 
    T.v(count).xPrev = x_near(1);     
	T.v(count).yPrev = x_near(2);
    T.v(count).dist= norm(x_new - x_near) + T.v(near_iter).dist;          
    T.v(count).indPrev = near_iter;   
    %%========  rewirte  =========%%
    [M,~] = size(nearptr);
    for k = 1:M
        x_1(1) = T.v(nearptr(k,1)).x;
        x_1(2) = T.v(nearptr(k,1)).y;
        x1_prev(1) = T.v(nearptr(k,1)).xPrev;
        x1_prev(2) = T.v(nearptr(k,1)).yPrev;
        if T.v(nearptr(k,1)).dist >  (T.v(count).dist + norm(x_1-x_new))
            T.v(nearptr(k,1)).dist = T.v(count).dist + norm(x_1-x_new);
            T.v(nearptr(k,1)).xPrev = x_new(1);    
            T.v(nearptr(k,1)).yPrev = x_new(2);
            T.v(nearptr(k,1)).indPrev = count;
            plot([x_1(1),x1_prev(1)],[x_1(2),x1_prev(2)],'-w');
            hold on;
            plot([x_1(1),x_new(1)],[x_1(2),x_new(2)],'-g');
            hold on;
        end
    end
    
    plot([x_near(1),x_new(1)],[x_near(2),x_new(2)],'-r*');
% plot([x_near(1), x_new(1)], [x_near(2), x_new(2)], 'r', 'Linewidth', 2);
    hold on;
    plot(x_new(1),x_new(2),'r*');
% plot(x_new(1), x_new(2), 'ko', 'MarkerSize', 2, 'MarkerFaceColor','k');
    hold on;
    if norm(x_new - goal) < Thr
        if (T.v(count).dist + norm(x_new - goal)) < start_goal_dist
            start_goal_dist = (T.v(count).dist + norm(x_new - goal));
            if length(path.pos) > 2
                for j = 2 : length(path.pos)
                    plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'w', 'Linewidth', 2);
                end
            end
            path.pos = [];
            if iter < 100000
                path.pos(1).x = x_G; path.pos(1).y = y_G;
                path.pos(2).x = T.v(end).x; path.pos(2).y = T.v(end).y;
                pathIndex = T.v(end).indPrev; % 
                j=0;
                while 1
                    path.pos(j+3).x = T.v(pathIndex).x;
                    path.pos(j+3).y = T.v(pathIndex).y;
                    pathIndex = T.v(pathIndex).indPrev;
                    if pathIndex == 1
                        break
                    end
                    j=j+1;
                end  % ���յ���ݵ����
                path.pos(end+1).x = x_I; path.pos(end).y = y_I; % ������·��
                for j = 2:length(path.pos)
                    plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'b', 'Linewidth', 2);
                end
            else
                disp('Error, no path found!');
            end
        end
        continue;
    end
    pause(0.01); 
end

%% parte agregada por mi para visualizar las figuras
A=uint8(A);
figure
imshow(A,[]);
hold on
plot(x_I, y_I, 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
hold on
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');
for j = 2:length(path.pos)
    hold on
    plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'b', 'Linewidth', 2);
end
title('informed RRT* path ');
% legend('RRT*')
legend('start','goal','informed RRT* path')
toc

% samplingnodesx=T.v.x(:,);
% samplingnodesy=T.v.y;
samplingnodes=[];
pathcost=0;
for i=1:N
samplingnodes=[samplingnodes;[T.v(N).x,T.v(N).y]];
pathcost=[pathcost;T.v(N).dist];
end
Treenodes=path.pos;
pathlength=dist;
Cbest=dist;
clearvars -except samplingnodes pathcost Treenodes pathlength Cbest;

