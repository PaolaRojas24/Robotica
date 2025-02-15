%Matriz Jacobiana
% Paola Rojas Domínguez A01737136

%Limpieza de pantalla
clear all
close all
clc

%Declaración de variables simbólicas
syms x y z

%Funciones
F1 = sin(5*x^3+3*y-4*y*x*z^2);
F2 =  -10*x^5-4*y*x*z+15*x*z^4;
F3 = cos(-x*y*z^5-6*x*y^(5)*z-7*y*x*z^2);

%Derivadas parciales de F1 respecto a x, y, z
Jv11 = diff(F1, x);
Jv12 = diff(F1, y);
Jv13 = diff(F1, z);

%Derivadas parciales de F2 y respecto a x, y, z
Jv21 = diff(F2, x);
Jv22 = diff(F2, y);
Jv23 = diff(F2, z);

%Derivadas parciales de F3 respecto a x, y, z
Jv31 = diff(F3, x);
Jv32 = diff(F3, y);
Jv33 = diff(F3, z);

disp('Matriz Jacobiana');

%Obtenemos la cinemática diferencial del péndulo a partir de la cinemática directa
jv_d = simplify([Jv11 Jv12 Jv13;
                 Jv21 Jv22 Jv23;
                 Jv31 Jv32 Jv33]);

pretty(jv_d)

%Evaluaando
x_ev = -5;
y_ev = -4;
z_ev = 1;

disp('Matriz Jacobiana Evaluada:');
jv_ev = double(subs(jv_d, {x, y, z}, {x_ev, y_ev, z_ev}));

jv_ev