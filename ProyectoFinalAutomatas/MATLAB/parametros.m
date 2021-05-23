%% Parámetros del modelo
clc, clear, close all

%% Traslación de Carro
% Masa del carro
m_c = 50000;

% Radio de la rueda
R_w = 0.5;

% Inercia de la rueda
J_w = 2.0;

% Relación de transmisión
r_t = 15;

% Inercia del motor
J_m = 10;

% Fricción mecánica, en eje rápido
b_eq_r = 30;  
b_eq = b_eq_r * (r_t^2);
b_t = b_eq / (R_w^2);




