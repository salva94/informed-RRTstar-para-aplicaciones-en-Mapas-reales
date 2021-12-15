function [x_min, min_idx] = ChooseParent(X_nears, x_new, T)
% En el conjunto vecino X_nears, busque el nodo principal que hace que x_new sea la distancia m¨¢s corta al punto de partida
nearest = [];
min_cost = 10000; %Calcule el costo requerido por x_new-> nodo m¨¢s cercano-> punto de partida
count = size(X_nears, 2);

for i = 1:count
    nearest(1) = T.v(X_nears(i)).x;
    nearest(2) = T.v(X_nears(i)).y;
    cost = distanc(nearest, x_new) + T.v(X_nears(i)).dist; 
    if distanc(nearest, x_new)<25
        if cost<min_cost
            min_cost = cost;
            x_min = nearest;
            min_idx = X_nears(i);
        end
    end
end
end
    