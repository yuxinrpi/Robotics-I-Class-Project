% Mini-Project 2
% Yuxin Hu
% Kinematic Control for an RRR robot or n-link robot to follow the S-shape
% Jacobian Inverse Kinematics
% 
% Note: This file was heavily modified from the class example

clear all;close all;clc;

%
% load the letter S as a curve
%
load S_letter_path

% specify end effector orientation
[xT,yT]=setR0T(Sls); % <<<< you need to provide this

% Unit vectors
zz=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];

% robot parameters
% For n-link robot, robot type = 1
% For RRR 3-link robot, robot type = 2
robot_type = 1;
if robot_type == 1
    n=10;l=3.5/n;
    %n=100;l=4/n;
    robot.P = zeros(3,n+1);
    for i=1:n;robot.P(:,i+1)=l*[1;0;0];end
elseif robot_type == 2
    n=3;%3-link
    % robot parameters
    l1 = 1.5; l2 = 1.5; l3 = 0.5;
    %l1 = 1.5; l2 = 1.5; l3 = 0.1;
    %l1 = 2; l2 = 1.5; l3 = 0.25;
    %l1 = 1.5; l2 = 1.5; l3 = 1;
    robot.P = [zz l1*ex l2*ex l3*ex];
end
robot.H = zeros(3,n);
robot.H(3,:)=ones(1,n);
robot.joint_type=zeros(1,n);

% radius of the link (as cylinders)
radius = .01;
robot_rb=defineRobot(robot,radius);

% Time and Step setup
dt = .1; % unit step
t=0:dt:200; % Time vector
N=length(t);

q=zeros(n,N);
% initial arm configuration
q0=[pi/2;-pi/n*ones(n-1,1)];
q(:,1)=q0;
% proportional feedback gain
Kp=diag(ones(n,1));

% The threshold for Initial and Final point
% Nthres - Frame dedicated for initial and final point
% FPD - The rest frames will be evenly distributed to other points
% N1 + FPD*(Number of points in Curve) + NF = Total Time steps
Nthres=round(N/10);
N1=Nthres;
Nf=Nthres;
FPD=round((N-2*Nthres)/(length(Sls)));%Frame per dot

% weighting in damped least square to avoid excessive speed
epsilon=1;
% maximum joint speed
umax=ones(n,1)*1;

for k=1:N-1
    % desired target
    if k<=N1
        qTd=atan2(xT(2,1),xT(1,1));
        pTd=Sls(:,1);
    elseif k>= N-Nf
        qTd=atan2(xT(2, end), xT(1, end));
        pTd=Sls(:,end);      
    else 
        id = floor((k-N1)/FPD)+1;
        qTd=atan2(xT(2,id),xT(1,id));
        pTd=Sls(:,id);         
    end
    
    % Codes in below section unchanged
    
    robot.q=q(:,k);
    % forward kinematics to find current pose and Jacobian
    robot=nlinkfwdkin(robot);
    qT(:,k)=atan2(robot.T(2,1),robot.T(1,1));
    pT(:,k)=robot.T(1:2,4);
    % form task space error 
    dX=[qT(:,k)-qTd;pT(:,k)-pTd];
    % Jacobian kinematics control (soft constraint on qdot)
    u(:,k)=-Kp*robot.J'*inv(robot.J*robot.J'+epsilon*eye(3,3))*dX;
    % constraint the maximum speed
    u(:,k)=(u(:,k)>umax).*umax+(u(:,k)<-umax).*(-umax)+...
        (u(:,k)<umax).*(u(:,k)>-umax).*u(:,k);
    % update robot motion using qdot (based on finite difference)
    qq=q(:,k)+(t(k+1)-t(k))*u(:,k);
    % restrict to be in -pi to pi
    q(:,k+1)=(qq>pi).*(-2*pi+qq)+(qq<-pi).*(2*pi+qq)+(qq<pi).*(qq>-pi).*qq;
end

%Set up the Animation Recording
if robot_type == 1
    v3 = VideoWriter('nlink_Jacob_demo.avi');
else
    v3 = VideoWriter('RRR_Jacob_demo.avi');
end
open(v3);

% show robot motion 
for k=1:FPD:N
    h=figure(10);
    plot(Sls(1,:),Sls(2,:),Sls(1,1),Sls(2,1),'o','linewidth',2);hold on;
    axis([-1,3,-2,2]);axis('square');
    xlabel('x');
    ylabel('y');
    title('Solution of Jacobian Inverse Kinematics (Yuxin Hu)');
    show(robot_rb,q(:,k),'Collision','on'); 
    view(0,90);
    pause(.1);
    frame = getframe; 
    writeVideo(v3,frame);
end
close(v3);
plot(Sls(1,:),Sls(2,:),Sls(1,1),Sls(2,1),'o','linewidth',2);
grid;
fprintf('max joint speed: %5.4f, %5.4f, %5.4f\n',max(abs(diff(q')')')');

%************** function *******************
%
% 2D rotation matrix
% Imported from class example.
%
% input = theta (angle in Rad)
% output = 2x2 SO(2) matrix
%
function R=rot2(theta)
    c=cos(theta);s=sin(theta);
    R=[c -s;s c];
end

%
% q=subprob0(k,p1,p2)
% q=find_angle(k,p1,p2)
%
% Directly borrowed from in class example Subproblem 0
% solve for q subtended between p1 and p2
%    k determines the sign of q
%
% input: k,p1,p2 as R^3 column vectors
% output: q (scalar)
%

function q=find_angle(k,p1,p2)
    if ((k'*p1)>sqrt(eps)|(k'*p2)>sqrt(eps))
      error('k must be perpendicular to p and q');
    end

    p1=p1/norm(p1);
    p2=p2/norm(p2);

    q=2*atan2(norm(p1-p2),norm(p1+p2));

    if k'*(cross(p1,p2))<0
      q=-q;
    end
end

%
% Forward Kinematics for n-Link robot
%
% Input = robot, with joint angle q
% Output = robot, the new robot with T and J updated
%
function robot=nlinkfwdkin(robot)
    q = robot.q;
    l = robot.P;

    % The qi = q1+q2+q3...qi
    for i = 2:length(q)
        q(i) = q(i) + q(i-1);
    end
    
    % Forward Kinematics
    % pi = pi-1 + R(qi)*p(i,i+1)
    p = zeros(3, length(q)+1);
    for i = 1:length(q)
        p(1:2,i+1) = p(1:2,i) + rot2(q(i))*l(1:2,i+1);
    end
    
    % Jacobian
    J = ones(3, length(q));
    for i = 1:length(q)
        piT = p(:, end) - p(:, i);
        J(2:3,i) =[0 -1;1 0]*piT(1:2,1);
    end
    
    % Set new value to robot
    robot.J = J;
    robot.T = [rot2(q(end)) [0;0] p(1:2,end)];
    robot.T(4,:) = 0;
    
end

%
% Set the task angle
%
% input = Sls, The path to follow
% output = [xT, yT], xT and yT are the desired task rotation,
%                    where [xT(:,i), yT(:,i)] conbined equals the
%                    rotation matrix R(i)
%
function [xT,yT]=setR0T(Sls)
    xT = zeros(3, length(Sls));
    yT = zeros(3, length(Sls));

    for i = 2:length(Sls)
        %Find Slope between points
        p2 = Sls(:, i) - Sls(:, i-1);
        p2(3,1) = 0;
        
        %Then find the angle between the slope and ex
        q = find_angle([0;0;1],[1;0;0],p2);
        
        %Rotate 90 deg clockwise
        q = q + pi/2;
        
        %qT3 -> q3T
        q = q + pi; 
        
        RT = rot2(q);
        xT(1:2, i) = RT(:,1);
        yT(1:2, i) = RT(:,2);
    end
    xT(1:2, 1) = xT(1:2, 2);
    yT(1:2, 1) = yT(1:2, 2);
end
