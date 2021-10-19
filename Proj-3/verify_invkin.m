%% This code file is to verify if the inverse kinematics is working properly
clc;clear;close all;
%% Define a irb 1200 Robot

ex = [1;0;0];
ey = [0;1;0];
ez = [0;0;1];
zz = [0;0;0];
% ABB IRB 1200-5/0.9 parameters

L1=399.1;
L2=448;
L3=42;
L4=451;
L5=82;

% P
p01=0*ex+L1*ez;
p12=zz;
p23=L2*ez;
p34=L3*ez+L4*ex;
p45=zz;
p56=zz;
p6T=L5*ex;

% H
h1=ez;
h2=ey;
h3=ey;
h4=ex;
h5=ey;
h6=ex;

% Final transformation
R6T=eye(3,3);

% define abb 1200 robot using POE convention
irb1200.P=[p01 p12 p23 p34 p45 p56 p6T]/1000;
irb1200.H=[h1 h2 h3 h4 h5 h6];
irb1200.joint_type=[0 0 0 0 0 0];
irb1200.R6T=eye(3,3);

%% Generate random q for each joint
for i = 1:6
    q(i,1) = rand*2*pi - pi;
end

% Forward Kinematics
irb1200.q = q;
irb1200 = forwardkin(irb1200);

% Clear q
% Inverse Kinematics
irb1200.q = zeros(6);
irb1200 = invkinelbow(irb1200);

% Display before and after
q
irb1200.q

% Check if q is in the invkin result
f = 0;
for i = 1:8
    if norm(irb1200.q(:,i) - q) < 1e-5
        disp("Forward kinematics and inverse kinematics are working correctly!");
        f = 1;
        break
    end
end
if f == 0
    disp("Error! The result doesn't match");
end

%% Functions
% Foward Kinematics
% Input:  Robot (Everything set up except q)
% Output: Robot (Return with 8 possible q)
function robot = forwardkin(robot)
    R = eye(3,3);
    p = robot.P(:,1);
    for i = 1:6
        R = R * expm(robot.q(i,1)*hat(robot.H(:, i)));
        p = p + R*robot.P(:,i+1);
    end
    robot.T = [R p;[0 0 0 0]];
end

% Hat function
% Input:  k = 3x1 vector
% Output: M = kx
function M = hat(k)
    M = [0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
end