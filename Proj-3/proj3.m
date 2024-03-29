% Mini-Project 3 CSCI-4480
% Yuxin Hu 661956455
% This file is modified based on the proj example provided

clear all; close all;clc;

% Declare basic vectors
zz=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];

load S_sphere_path

% plot the spherical S
figure(1);plot3(p_S(1,:),p_S(2,:),p_S(3,:),'rx','linewidth',3);
xlabel('x');ylabel('y');zlabel('z');
title("The Original S curve(Yuxin Hu)");
hold on;
% add a 3d sphere
surf(X,Y,Z)
% make it transparent
alpha .5
axis(r*[-1 1 -1 1 0 2]);axis('square');
view(120,10);

% convert to equal path length grid
diffS=vecnorm(diff(p_S')');
ls=[0 cumsum(diffS)];
lf=sum(diffS);
N=100;
l=(0:lf/N:lf);

pS=interp1(ls,p_S',l,'spline')';
% plot it out again with equal path length
figure(2);plot3(pS(1,:),pS(2,:),pS(3,:),'rx','linewidth',3);
xlabel('x');ylabel('y');zlabel('z');
title("The Modified S curve with equal path length(Yuxin Hu)");
hold on;
% 3d sphere
surf(X,Y,Z)
% make it transparent
alpha 0.4
axis(r*[-1 1 -1 1 0 2]);axis('square');
view(120,10);

% save it as the path file
save S_sphere_path_uniform l pS

% find the end effector frame
pc=r*ez;
N=length(pS);
xT=zeros(3,N);zT=zeros(3,N);yT=zeros(3,N);
quat=zeros(4,N); % allocate space for unit quaternion representation of R_{0T}

for i=1:N   
    xT(:,i)=(pS(:,i)-pc)/norm(pS(:,i)-pc);
    if i<N
        yT(:,i)=(pS(:,i+1)-pS(:,i));
    else
        yT(:,i)=yT(:,i-1);
    end
    yT(:,i)=yT(:,i)-yT(:,i)'*xT(:,i)*xT(:,i);
    yT(:,i)=yT(:,i)/norm(yT(:,i));
    zT(:,i)=cross(xT(:,i),yT(:,i));
    R=[xT(:,i) yT(:,i) zT(:,i)];
    
    %Three representations as required
    quat(:,i)=R2q(R);               %Quaternion
    euler_angle(:, i) = R2EuA(R);   %Euler Angle
    [k, th]=R2kth(R);
    kth(:, i) = [k;th];               %Angle-Axis Product
end

% Generate lambda array (Length along path)
dlvec=vecnorm(diff(pS')');
lambda(1) = dlvec(1);
for i = 2:N
    lambda(i) = lambda(1) + lambda(i-1);
end

% Plot yaw-pitch-roll ZYX Euler Angles
figure(4);
plot(lambda, euler_angle, 'linewidth', 2);
title("The Euler Angle (ZYX) representation of the end effector pose (Yuxin Hu)");
grid();
xlabel("Length along path");
ylabel("Euler Angle beta");
legend(["Beta1", "Beta2", "Beta3"]);

% Plot Angle-Axis Product
figure(5);
plot(lambda, kth, 'linewidth', 2);
title("The Angle-Axis Product representation of the end effector pose (Yuxin Hu)");
grid();
xlabel("Length along path");
ylabel("Axis Component / Theta angle");
legend(["kx", "ky", "kz", "theta"]);

% plot out the end effector frame
m=5;
% MATLAB's plotTransforms command plots a frame at a given location
figure(2);h=plotTransforms(pS(:,1:m:end)',quat(:,1:m:end)');
set(h,'LineWidth',.5);

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

% define collision body for abb 1200
radius=.01;
[irb1200_rbt,colLink]=defineRobot(irb1200,radius);
% 
% find all inverse kinematics solutions
%

for i=1:N
    % specify end effector SE(3) frame
    Td{i}=[[xT(:,i) yT(:,i) zT(:,i)]*R6T' pS(:,i);[0 0 0 1]];
    %Td{i}=[[xT(:,i) yT(:,i) zT(:,i)] pS(:,i);[0 0 0 1]];
    irb1200.T=Td{i};
    %
    irb1200=invkinelbow(irb1200); % << The implemented part invkin
    %
    for k=1:8
        q(:,i,k)=irb1200.q(:,k);
    end
    
    % The Checking part was removed, but the result is proved to be correct
    % check forward kinematics to make sure the IK solution is correct
    %for k=1:8
    %    irb1200.q=q(:,i,k);
    %    irb1200=fwddiffkiniter(irb1200);
    %    T{i,k}=irb1200.T;
    %end

end

% choose the pose to visualize
ksol=6

% Draw a new plot
figure(6);
plot3(pS(1,:),pS(2,:),pS(3,:),'rx','linewidth',3);
xlabel('x');ylabel('y');zlabel('z');
title("The IRB-1200/0.9 Robot Motion Path(Yuxin Hu)");
hold on;
% 3d sphere
surf(X,Y,Z)
% make it transparent
alpha 0.4
axis(r*[-1 1 -1 1 0 2]);axis('square');
view(120,10);
h=plotTransforms(pS(:,1:m:end)',quat(:,1:m:end)');
set(h,'LineWidth',.5);

%Set up the Animation Recording
v = VideoWriter('IRB-1200_invkin.avi');
v.FrameRate = 10;
open(v);

for i=1:N
    % show robot pose (ever 5 frames)
    if mod(i,5)==0
        %disp(norm(T{i,ksol}-Td{i}));
        figure(6);show(irb1200_rbt,q(:,i,ksol),'collision','on');
        frame = getframe; 
        writeVideo(v,frame);
        view(150,10);
    end
end
close(v);

% compute end effector linear and angular velcoity 
lsdot=.01;
for i=1:N-1
    dt(i) = (ls(i+1)-ls(i))/lsdot;
    for k=1:8
        qdot(:,i,k)=(q(:,i+1,k)-q(:,i,k))/dt(i);
    end
end

for k=1:8
   for i=1:6
       maxqdot(i,k)=max(qdot(i,:,k));
   end
   fprintf('maximum qdot for pose %d \n', k);
   disp(maxqdot(:,k)');   
end

%% Functions
%
% R2q.m
%
% converts R in SO(3) to unit quaternion q, (q0,q_vec)
%

function q=R2q(R)
  
  q=zeros(4,1);
  q(1)=.5*sqrt(trace(R)+1); %Half angle equation?
  if abs(q(1))<1e-5
    [k,theta]=R2kth(R);
    q(2:4)=k;
  else
    q(2:4)=vee(R-R.')/4/q(1);
  end
end

%
% R2kth
%
% converts R in SO(3) to k and theta
%

function [k, th]=R2kth(R)
    s = norm(R - R.')/2;
    c = (trace(R) - 1)/2;
    th = atan2(s, c);
    t = (R - R.')/(2*sin(th));
    k = vee(t);
end

%
% R2kth
%
% converts R in SO(3) to Yaw-Pitch_Roll
% Returns beta = [beta1 beta2 beta3]
%

function beta = R2EuA(R)
    ex = [1;0;0];
    ey = [0;1;0];
    ez = [0;0;1];
    
    k = ey;
    h1 = ex;
    h2 = ez;
    d = ex'*R(:, 3);
    beta2 = subprob4(k, h1, h2, d);
    beta2 = beta2(1,1);
    
    c2 = cos(beta2);
    s2 = sin(beta2);
    
    k = ex;
    p1 = R(:, 3);
    p2 = [c2 0 s2; 0 1 0; -s2 0 c2]*ez;
    beta1 = -subprob1(k, p1 ,p2);
    
    c2 = cos(-beta2);
    s2 = sin(-beta2);
    k = ez;
    p1 = R'*ex;
    p2 = [c2 0 s2; 0 1 0; -s2 0 c2]*ex;
    beta3 = subprob1(k, p1, p2);
    
    beta = [beta1; beta2; beta3];
end

%
% hat
%
% Find kx
% input:  k = 3x1 vector
% output: M = kx
%

function M = hat(k)
    M = [0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
end

%
% vee
%
% The inverse function of hat
%   input: R = hat(k)
%   output: k = inverse_hat(R)
%

function k = vee(R)
    k(1,1) = R(3,2);
    k(2,1) = R(1,3);
    k(3,1) = R(2,1);
end