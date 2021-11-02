clc;clear;close all;

proj4init;

%% Wrist Singularity

q_1 = [0;0;0;0;pi/2;0];
figure(1);show(irb1200_rbt,q_1,'collision','on');
axis(r*[-1 1 -1 1 0 2]);axis('square');
view(120,10);
title("The Wrist Singularity Demo (Yuxin Hu)");
xlabel("X");
ylabel("Y");
zlabel("Z");

%% Boundary Singularity
q_2 = [0;0;atan2(-L4,L3);0;0;0];
figure(2);show(irb1200_rbt,q_2,'collision','on');
axis(r*[-1 1 -1 1 0 2]);axis('square');
view(120,10);
title("The Boundary Singularity Demo (Yuxin Hu)");
xlabel("X");
ylabel("Y");
zlabel("Z");

%% Boundary Singularity
q_3 = [0;atan2(-L2+L3, L4);0;0;0;0];
figure(3);show(irb1200_rbt,q_3,'collision','on');
axis(r*[-1 1 -1 1 0 3]);axis('square');
view(120,10);
title("The Interior Singularity Demo (Yuxin Hu)");
xlabel("X");
ylabel("Y");
zlabel("Z");