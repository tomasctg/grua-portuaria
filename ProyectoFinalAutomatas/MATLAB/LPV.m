clc
clear
syms t xl(t) xt(t) yl(t) yt0 xlaux xtaux ylaux dxl dxt dyl Mt Ft bt Fw deltaxl deltaxt deltayl

%% Carro
ddxt = (1/Mt)*Ft - (bt/Mt)*diff(xt(t),t) + sin(atan((xl-xt)/(yt0 - yl)))*(Fw/Mt)
dxlaux1 = subs(ddxt,'diff(xt(t),t)','diffxt');
parcialxl = simplify(diff(dxlaux1,'diffxt'))
% parcialxl = subs(parcialxl,'ylaux','yl(t)');



%% Xl
eq1 = xt + sqrt((xt-xl)^2+(yt0-yl)^2)*sin(atan((xl-xt)/(yt0 - yl))) ;

dxl= diff(eq1,t)

dxlaux1 = subs(dxl,xl,xlaux)
parcialxl = simplify(diff(dxlaux1,xlaux))
%parcialxl=subs(parcialxl,'xlaux','xl(t)');


dxlaux2 = subs(dxl,'xt(t)','xtaux');
parcialxt =simplify(diff(dxlaux2,'xtaux'));
parcialxt=subs(parcialxt,'xtaux','xt(t)');
parcialxt = parcialxt*deltaxt

dxlaux3 = subs(dxl,'yl(t)','ylaux');
parcialyl = simplify(diff(dxlaux3,'ylaux'));
parcialyl=subs(parcialyl,'ylaux','yl(t)');
parcialyl = parcialyl*deltayl

deltadxl= parcialxl + parcialxt + parcialyl

%% 
 str = latex(deq1)
 axis off
 text(1, 1, ['$$' str '$$'], 'Interpreter','latex', 'FontSize',5, ...
    'HorizontalAlignment','center', 'VerticalAlignment','middle')