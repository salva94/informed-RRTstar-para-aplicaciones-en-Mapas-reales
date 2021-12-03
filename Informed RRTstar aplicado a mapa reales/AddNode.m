function T = AddNode(T, x_new, x_near, near_Idx)
%Agregue el nodo x_new de collision_free al ¨¢rbol T y use x_near como el nodo principal
    count = size(T.v,2) + 1;
    T.v(count).x = x_new(1);
    T.v(count).y = x_new(2);
    T.v(count).xPrev = T.v(near_Idx).x;
    T.v(count).yPrev = T.v(near_Idx).y;
    T.v(count).dist=distance(x_new, x_near) + T.v(near_Idx).dist;  % La distancia desde el nodo hasta el origen.
    T.v(count).indPrev = near_Idx;     %El ¨ªndice del nodo padre
end
