function [s] = solve_profile_vel(deltay,deltat,amax)
syms ta ts vmax
% Sistema de 3 ecuaciones: Representa la solucion del perfil trapezoidal
% ta: Tiempo de subida y bajada del perfil 
% ts: Tiempo a velocidad constante
% deltay: Lo que quiero que se desplaze en y
% deltat: En cuanto tiempo quiero que se desplaze deltay
eq1 = vmax*(ts + ta) == deltay;
eq2 = 2*ta + ts == deltat;
eq3 = vmax/ta == amax;
eq4 = vmax<=3.0;
eq5 = ta>= 0;
eq6 = ts>= 0;
s = solve(eq1,eq2,eq3,eq4,eq5,eq6,'Real',true);
end

