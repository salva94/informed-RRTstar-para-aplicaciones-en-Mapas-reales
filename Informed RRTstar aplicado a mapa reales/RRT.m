clear
clc
close all

map=imread('1.1.jpg');
map=map(1:694,1:1006,:);% mapa 1
START=map;
MAP=zeros(size(map,1),size(map,2));
for i=1:size(map,1)
    for j=1:size(map,2)
        if map(i,j,1)>220 && map(i,j,2)>220 && map(i,j,3)>220
            MAP(i,j)=1;
        end
    end
end

se1=strel('disk',3);%Aquí se crea un elemento de estructura de disco plano con un radio de 5.
MAP=imdilate(MAP,se1);
imLabel = bwlabel(MAP);                %Etiqueta cada dominio conectado
stats = regionprops(imLabel,'Area');    %Encuentra el tamaño de cada dominio conectado
area = cat(1,stats.Area);
index = find(area == max(area));        %Encuentre el índice del dominio conectado más grande
MAP = ismember(imLabel,index);          %Obtenga la imagen de dominio conectado más grande

% figure

x_I=4; y_I=353;           
x_G=767; y_G=359;

Thr=30;                 % Establecer el umbral del punto de destino (qué tan lejos del punto de destino se considera el punto final)
Delta= 20;  

%%Inicialización del árbol
T.v(1).x = x_I;         % T es un árbol, v es un nodo, primero agregue el punto de partida al árbol
T.v(1).y = y_I; 
T.v(1).xPrev = x_I;     % El nodo padre del nodo inicial sigue siendo él mismo
T.v(1).yPrev = y_I;
T.v(1).dist=0;          % La distancia desde el nodo padre al nodo, que puede ser una distancia euclidiana.
T.v(1).indPrev = 0;     % El índice del nodo padre

figure(1);
MAP=255*MAP;
Imp=uint8(MAP);
% Imp=rgb2gray(ImpRgb);
imshow(Imp) %800*800
title(' Search RRT path Map');
xL=size(Imp,2);%Longitud del eje x del mapa
yL=size(Imp,1);%Longitud del eje Y del mapa
hold on
plot(x_I, y_I, 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');% Dibujar el punto de partida y el punto de destino
count=1;
% bFind = false;
goal = [x_G,y_G];
start_goal_dist = 10000;

path.pos(1).x = 700;
path.pos(1).y = 700;


for iter = 1:100000
    x_rand=[];
    x_rand(1) = xL*rand; 
    x_rand(2) = yL*rand;
    
    %%======= Buscar x_near===========%%
    x_near=[];
    min_dist = 1000000;
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
    count=count+1;
    %%========Agrega X_NEW al árbol========%%
    T.v(count).x = x_new(1);
    T.v(count).y = x_new(2); 
    T.v(count).xPrev = x_near(1);     
	T.v(count).yPrev = x_near(2);
    T.v(count).dist= norm(x_new - x_near);          
    T.v(count).indPrev = near_iter;   
%     plot([x_near(1),x_new(1)],[x_near(2),x_new(2)],'-r');
    plot([x_near(1), x_new(1)], [x_near(2), x_new(2)], 'r', 'Linewidth', 2);
    plot(x_new(1), x_new(2), 'ko', 'MarkerSize', 2, 'MarkerFaceColor','k');
%     hold on;
    %========Determinar si encontro o no la ruta==========%%
    if norm(x_new - goal) < Thr
        break;
    end
    pause(0.1);
end
%%
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
    end  % 
    path.pos(end+1).x = x_I; path.pos(end).y = y_I; % 
    for j=2:length(path.pos)
%       plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'b', 'Linewidth', 1);
        plot([x_near(1), x_new(1)], [x_near(2), x_new(2)], 'r', 'Linewidth', 2);
        plot(x_new(1), x_new(2), 'ko', 'MarkerSize', 4, 'MarkerFaceColor','k');
        plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'b', 'Linewidth', 1);
    end
else
    disp('Error, no path found!');
end

%% parte agregada por mi para visualizar las figuras
figure
imshow(START,[]);
hold on
plot(x_I, y_I, 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
hold on
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');
for j = 2:length(path.pos)
    hold on
%     plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'b', 'Linewidth', 1);
      plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'b', 'Linewidth', 2);
end
title('RRT path map');
% legend('RRT*')
legend('start','goal','RRT path')
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

