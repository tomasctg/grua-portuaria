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

%Viscosidad dinamica equivalente referenciada a la rueda
% Beqrot = (br + bm*(rt^2));
Beqrot = 30;


%Masa equivalente de carro
Meq = (mc + Jeq/(Rr^2));

%Rozamiento equivalente de rueda
Beqtran = (0 + Beqrot/(Rr^2));


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
Bheqd = (bh  + (beqd/(Rd^2)));

g=9.82;

yt0 = 45;
