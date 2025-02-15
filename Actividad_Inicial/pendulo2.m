%Robot planar de 2gdl
%Paola Rojas Domínguez A01737136

%Liempieza de consola
clear all
close all
clc

%Declaración de variables simbólicas
syms th1(t) l1 th2(t) l2

%Configuración del robot, 0 para junta rotacional, 1 para junta prismática
RP = [0 0];

%Creamos el vector de coordenadas generalizadas
Q = [th1 th2];
disp('Coordenadas generalizadas');
pretty(Q);

%Creamos el vector de velocidades generalizadas
Qp = diff(Q, t);
disp('Velocidades generalizadas');
pretty(Qp);

%Número de grado de libertad del robot
GDL = size(RP, 2);
GDL_str = num2str(GDL);

%Junta 1
%Posición de la junta 1 respecto a 0
P(:,:,1) = [l1*cos(th1);
            l1*sin(th1); 
            0];

%Matriz de rotación de la junta 1 respecto a 0
R(:,:,1) = [cos(th1) -sin(th1) 0;
            sin(th1)  cos(th1) 0;
            0         0        1];

%Junta 2
%Posición de la junta 1 respecto a 0
P(:,:,2) = [l2*cos(th2);
            l2*sin(th2);
            0];

%Matriz de rotación de la junta 1 respecto a 0
R(:,:,2) = [cos(th2) -sin(th2) 0;
            sin(th2)  cos(th2) 0;
            0         0        1];

%Vector de ceros
c = [0; 0; 0;];

%Matrizes de transformación Homogénea
H1 = [R(:,:,1) P(:,:,1);
      c'       1];
H2 = [R(:,:,2) P(:,:,2);
      c'       1];

%Matriz Global
MG = H1 * H2;

%Cinemática Directa
x = MG(1,4);
y = MG(2,4);
z = MG(3,4);

%Derivadas parciales de x
Jv11 = functionalDerivative(x, th1);
Jv12 = functionalDerivative(x, th2);

%Derivadas parciales de y
Jv21 = functionalDerivative(y, th1);
Jv22 = functionalDerivative(y, th2);

%Derivadas parciales de z
Jv31 = functionalDerivative(z, th1);
Jv32 = functionalDerivative(z, th2);

disp('Cinemática diferencial de la posición del péndulo');

%Obtenemos la cinemática diferencial del péndulo a partir de la cinemática directa
jv_d = simplify([Jv11 Jv12;
                 Jv21 Jv22;
                 Jv31 Jv32]);

pretty(jv_d)