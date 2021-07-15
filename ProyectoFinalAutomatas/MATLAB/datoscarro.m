clc
clear
%% Datos generales del carro

% Masa carro
mc = 50000; %[Kg]

% Vizcosidad dinamica carro
bc =1;

%Radio rueda
Rr = 0.5;

%Relacion de transmision
rt = 15.0;

%Inercia rotor
Jm = 10;

%Inercia rueda
Jr = 2.0;

%vizcosidad rotor
bm =1;

%vizcosidad rueda
br =1;


%Inercia rotacional equivalente referenciada a la rueda
Jeq = (Jr + Jm*(rt^2));

%Viscosidad dinamica equivalente referenciada al motor
% Beqrot = (br + bm*(rt^2));
Beqrot = 30;
Beqrueda = Beqrot * rt^2;
Beqtran = (0 + Beqrueda/(Rr^2));

%Masa equivalente de carro
Meq = (mc + Jeq/(Rr^2));



%% Datos generales izaje
Mh = 0;

Kw = 1800000; %[kN/m]
bw = 30000;

Rd = 0.75;

rtd = 30;

bh=0;

Jd = 8;

Jmd = 30;

beqd = 18;

Jeqd = (Jd + Jmd*(rtd^2));

Mheq = (Mh + (Jeqd/(Rd^2)));
Bheqd = (bh  + ((beqd*(rtd^2))/(Rd^2)));

g=9.82;

yt0 = 45;

bcx =1000;
bcy= 500;
Kcy = 1.3e6;

%%Condiciones iniciales en t0

%Caso suspendido:
%lh: Se tendra un longuitud de cable desenrrollada determianda por el
%encoder del sistema de izaje. => lh(t0)=lh0
%l: Longuitud de cable desenrrollado inicial dependera de la longuitud lh0
%sin enlogamiento mas el elongamiento debido a la fuerza peso. Entonces
%l(t0)=l0=lh0 + (ml*g)/Kw.
%Por otro lado, considerando una situacion de equilibrio estatico,
%xl(t0)=xt(t0)=xl0=xt0. Siendo el angulo theta0 = 0. Luego yt(t0)=yt0, una
%altura constante donde se encuentra el izaje.
%Por ultimo, yl(t0)  = yt0 - l0.

ml=15000;
yt0=45; %[m]
theta0 = 0;
xt0=-20;
xl0=xt0;
lh0=42;
l0 = lh0 + (ml*g)/Kw;
yl0 = yt0 - l0;

%%Controlador Carro
%Polo de sistema carro:l0
s=tf('s');
Hc = 1/(s*(s*Meq + Beqtran));
Polesc = pole(Hc)
%pzmap(Hc)
Pc = -Beqtran/Meq
wpos = -10*Pc;
n=2.5;
bac = (n*wpos*Meq )*(Rr/rt)
ksac=n*(wpos^2)*Meq*(Rr/rt)
ksiac = (wpos^3)*Meq*(Rr/rt)
tau = 1/1000;

%%Controlador Izaje
%Polo de sistema izaje
Hiz = 1/(s*(s*Mheq + Bheqd));
Polesiz = pole(Hiz)
%pzmap(Hiz)
Piz = -Bheqd/Mheq
wposiz = -10*Piz;
baiz = -(n*wposiz*Mheq)*(Rd/rtd)
ksaiz=-n*(wposiz^2)*Mheq*(Rd/rtd)
ksiaiz = -(wposiz^3)*Mheq*(Rd/rtd)

[vyt,vxt,x_end,vxt_end,vyt_end]=gen_traj_to_boat([10,15,10,5,1],xl0,yl0,2,0);
%[vyt,vxt,x_end,vxt_end,vyt_end]=gen_traj_to_dock([10,15,13,5,1],4,1,2,0);

% trayectoria_dy_to_boat = timeseries(trayectoria_dy_to_boat(:,1)',trayectoria_dy_to_boat(:,2)');
% trayectoria_dx_to_boat = timeseries(trayectoria_dx_to_boat(:,1)',trayectoria_dx_to_boat(:,2)');
% 
% 
% trayectoria_dy_to_boat_end = timeseries(trayectoria_dy_to_boat_end(:,1)',trayectoria_dy_to_boat_end(:,2)');
% trayectoria_dx_to_boat_end = timeseries(trayectoria_dx_to_boat_end(:,1)',trayectoria_dx_to_boat_end(:,2)');
% 
% % [trayectoria_dy_to_dock,trayectoria_dx_to_dock]=gen_traj_to_dock([10,15,10,5,1],4,-7.500,1,0);
% % 
% 
% % x_to_dock=cumtrapz(trayectoria_dx_to_dock(:,2),trayectoria_dx_to_dock(:,1))+7;
% % y_to_dock=-cumtrapz(trayectoria_dy_to_dock(:,2),trayectoria_dy_to_dock(:,1))-7.50;
% % 
% 
% % 
% % figure(6)
% % plot(x_to_dock,y_to_dock(1:length(y_to_dock)-1,1))
% % figure(7)
% % plot(x_to_boat,y_to_boat(1:length(y_to_boat)-1,1))
% % % 
% % figure(1)
% % plot(trayectoria_dx(:,2),trayectoria_dx(:,1))
% % % figure(2)
% % % plot(trayectoria_dy(:,2),trayectoria_dy(:,1))
% % % 
% % figure(3)
% % plot(trayectoria_dx(:,2),xt0 + cumtrapz(trayectoria_dx(:,2),trayectoria_dx(:,1)))
% % hold on
% % plot(trayectoria_dy(:,2),-cumtrapz(trayectoria_dy(:,2),trayectoria_dy(:,1))+yl0)
% % % 
