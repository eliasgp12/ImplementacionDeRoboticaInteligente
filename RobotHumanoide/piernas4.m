clear
close all
clc

H0=SE3([3 2 7]);
H1=SE3(roty(-pi)*rotz(-pi/2)*rotx(-pi/8), [3 2 7]);
H2=SE3(rotx(-pi/3)*roty(pi/2), [0 0 0]);
H3=SE3(rotz(-151/225*pi), [0 0 0]); 
H4=SE3(rotx(pi/2), [0 0 0]);

H5=SE3([3 0 0]);
rotarz= rotz(-23/180*pi);
H6=SE3((rotz(pi/2)*roty(pi/2))*rotarz, [4.588 0 0]);
H7=SE3(rotz(pi/2)*rotx(pi/2), [0 0 0]);
H8=SE3([0 0 2.8284]);

H20= H1*H2;
H30= H20*H3;
H40= H30*H4;
H50= H40*H5;
H60= H50*H6;
H70= H60*H7;
H80= H70*H8;

%Coordenadas de translación y rotación
x=[ 2  0  3];
y=[ 2  2  2];
z=[-2  0  7];

plot3(x, y, z,'LineWidth', 1.5); axis([-1 7 -1 7 -3 9]); grid on;
hold on;

trplot(H0,'rgb','axis', [-1 7 -1 7 -3 9])
 pause;
  tranimate(H0, H1,'rgb','axis', [-1 7 -1 7 -3 9])
 pause;
 tranimate(H1, H20,'rgb','axis', [-1 7 -1 7 -3 9])
 pause;
  tranimate(H20, H30,'rgb','axis', [-1 7 -1 7 -3 9]);
 pause;
  tranimate(H30, H40,'rgb','axis', [-1 7 -1 7 -3 9]);
 pause;
  tranimate(H40, H50,'rgb','axis', [-1 7 -1 7 -3 9]);
 pause;
  tranimate(H50, H60,'rgb','axis', [-1 7 -1 7 -3 9]);
 pause;
  tranimate(H60, H70,'rgb','axis', [-1 7 -1 7 -3 9]);
 pause;
  tranimate(H70, H80,'rgb','axis', [-1 7 -1 7 -3 9]);
  disp('Matriz de transformación homogénea global T')
  disp(H80)