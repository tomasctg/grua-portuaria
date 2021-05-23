%% Par�metros del modelo
clc, clear, close all

%% Traslaci�n de Carro
% Masa del carro
m_c = 50000;

% Radio de la rueda
R_w = 0.5;

% Inercia de la rueda
J_w = 2.0;

% Relaci�n de transmisi�n
r_t = 15;

% Inercia del motor
J_m = 10;

% Fricci�n mec�nica, en eje r�pido
b_eq_r = 30;  
b_eq = b_eq_r * (r_t^2);
b_t = b_eq / (R_w^2);




