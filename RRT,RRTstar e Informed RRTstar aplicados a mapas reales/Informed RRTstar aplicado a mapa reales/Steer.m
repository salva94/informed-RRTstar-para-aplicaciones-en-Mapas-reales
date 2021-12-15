function  x_new = Steer(x_rand, x_near, StepSize)
% Mueva el nodo x_m¨¢s cercano al punto aleatorio x_rand por la distancia de StepSize en la direcci¨®n x_rand para generar un nuevo nodo x_new
    dis = distanc(x_near, x_rand);
    % Trastorno obsesivo compulsivo, quiero convertir las coordenadas del nuevo nodo en un n¨²mero entero, corregir el redondeo (o no un n¨²mero entero)
    x_new(1) = fix(((dis-StepSize)*x_near(1) + StepSize*x_rand(1)) / dis);
    x_new(2) = fix(((dis-StepSize)*x_near(2) + StepSize*x_rand(2)) / dis);
end