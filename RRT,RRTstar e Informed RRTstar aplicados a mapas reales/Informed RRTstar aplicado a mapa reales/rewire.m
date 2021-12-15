function T = rewire(T, X_nears, x_new, Imp)
% Para los nodos adyacentes, excepto el nodo m¨¢s cercano x_near, si la distancia al punto de inicio es m¨¢s corta por x_new, actualice x_new como su nodo padre,
count = size(X_nears, 2);
new_idx = size(T.v, 2);

for i = 2:count
    pre_cost = T.v(X_nears(i)).dist; 
    near_node(1) = T.v(X_nears(i)).x;
    near_node(2) = T.v(X_nears(i)).y;
    tentative_cost = distance(near_node, x_new) + T.v(new_idx).dist; % Coste desde x_new hasta el punto de partida
    if ~collisionChecking(near_node, x_new,Imp)  % La detecci¨®n de colisiones tambi¨¦n es necesaria durante el recableado.
        continue;
    end
    if tentative_cost<pre_cost  % Si el costo se reduce a trav¨¦s del punto de partida, cambie su nodo principal a x_new
        T.v(X_nears(i)).xPrev = x_new(1);
        T.v(X_nears(i)).yPrev = x_new(2);
        T.v(X_nears(i)).dist = tentative_cost;
        T.v(X_nears(i)).indPrev = new_idx;
    end
end

end