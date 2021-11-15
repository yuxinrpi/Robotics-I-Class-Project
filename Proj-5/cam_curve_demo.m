clc; clear; close all;

load S_letter_path.mat
cam_def;

ex=[1;0;0];ey=[0;1;0];ez=[0;0;1];

% Though the parameters are the same with the cam_def default values,
% I copied it here in case any parameters need to be changed
W = 1280; % width in pixels (1280)
H = 1024; % height in pixels (1024)
rhow = 1e-5; % width per pixel (10um)
rhoh = 1e-5; % height per pixel (10um)
f = .015; % focal length (0.015m)
u0=W/2; %center of image plane in pixel coordinate
v0=H/2; %center of image plane in pixel coordinate

%% Planar S curve
% Rectify the original plot so it would appear in the scene
% XY plane -> YZ plane, 
% Y range [1,3]->[-0.5,0.5], Z range [-1,1] -> [0,1]
S = [zeros(1,length(Sls));Sls(1,:)/2-1;Sls(2,:)/2+.5];

figure(6);plot3(S(1,:),S(2,:),S(3,:),'rx','linewidth',3);
xlabel('x');ylabel('y');zlabel('z');
title("The Planar S curve After Rectification(Yuxin Hu)");
hold on;
axis([-1 1 -1 1 -0.5 1.5]);axis('square');
view(120,10);


% First position
xc = 1.2;
camOrigin = [xc; 0; 0.5];
camFrame = [-ey ez -ex];
Tcam = [camFrame camOrigin;0 0 0 1];
[uv,uvw,P1]=cam_image(cam,Tcam,S);

figure(1);
plot(uv(1,:), uv(2,:),'rx');
xlabel("x");ylabel("y");
set(gca, 'XDir','reverse')
xlim([0,W]);
ylim([0,H]);
title("Planar S curve camera view (xc = "+xc+") (Yuxin Hu)");
xlabel("Image pixels X axis");
ylabel("Image pixels Y axis");

% Second position
xc = 2.5;
camOrigin = [xc; 0; 0.5];
camFrame = [-ey ez -ex];
Tcam = [camFrame camOrigin;0 0 0 1];
[uv,uvw,P1]=cam_image(cam,Tcam,S);

figure(2);
plot(uv(1,:), uv(2,:),'rx');
xlabel("x");ylabel("y");
set(gca, 'XDir','reverse')
xlim([0,W]);
ylim([0,H]);
title("Planar S curve camera view (xc = "+xc+") (Yuxin Hu)");
xlabel("Image pixels X axis");
ylabel("Image pixels Y axis");

%% Sphere S curve
load S_sphere_path.mat
% plot the spherical S

figure(3);plot3(p_S(1,:),p_S(2,:),p_S(3,:),'rx','linewidth',3);
xlabel('x');ylabel('y');zlabel('z');
title("The Original S curve(Yuxin Hu)");
hold on;
% add a 3d sphere
surf(X,Y,Z)
% make it transparent
alpha .5
axis(r*[-1 1 -1 1 0 2]);axis('square');
view(120,10);

% First position
xc = 1.2;
camOrigin = [xc; 0; 0.5];
camFrame = [-ey ez -ex];
Tcam = [camFrame camOrigin;0 0 0 1];
[uv,uvw,P1]=cam_image(cam,Tcam,p_S);

figure(4);
%plot(P1(1,:), P1(2,:))
plot(uv(1,:), uv(2,:), 'rx');
set(gca, 'XDir','reverse')
xlim([0,W]);
ylim([0,H]);
title("Sphere S curve camera view (xc = "+xc+") (Yuxin Hu)");
xlabel("Image pixels X axis");
ylabel("Image pixels Y axis");

% Second position
xc = 2.5;
camOrigin = [xc; 0; 0.5];
camFrame = [-ey ez -ex];
Tcam = [camFrame camOrigin;0 0 0 1];
[uv,uvw,P1]=cam_image(cam,Tcam,p_S);

figure(5);
%plot(P1(1,:), P1(2,:))
plot(uv(1,:), uv(2,:), 'rx');
set(gca, 'XDir','reverse')
xlim([0,W]);
ylim([0,H]);
title("Planar S curve camera view (xc = "+xc+") (Yuxin Hu)");
xlabel("Image pixels X axis");
ylabel("Image pixels Y axis");