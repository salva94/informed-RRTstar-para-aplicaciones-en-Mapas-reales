function nearNodes = NearC(T, x_new, near_idx)
%  Encuentre el ¨ªndice de todos los nodos en el ¨¢rbol cuya distancia desde x_new sea menor que el radio y col¨®quelos en nearNodes junto con near_idx
    nearNodes = [near_idx];
    num = 2;
    count = size(T.v,2);
    radius = 30;
    for Idx = 1: count
        x_near = [];
        x_near(1) = T.v(Idx).x;
        x_near(2) = T.v(Idx).y;
        dis = distance(x_near, x_new);
        if dis<radius
            nearNodes(num) = Idx;
            num = num+1;
        end
    end
end