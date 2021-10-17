%
%
% 
clear all; close all;clc;

zz=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];

load S_sphere_path

% plot the spherical S
figure(1);plot3(p_S(1,:),p_S(2,:),p_S(3,:),'rx','linewidth',3);
xlabel('x');ylabel('y');zlabel('z');
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
N=101;
l=(0:lf/N:lf);

pS=interp1(ls,p_S',l,'spline')';
% plot it out again with equal path length
figure(2);plot3(pS(1,:),pS(2,:),pS(3,:),'rx','linewidth',3);
xlabel('x');ylabel('y');zlabel('z');
hold on;
% 3d sphere
surf(X,Y,Z)
% make it transparent
alpha 0.4
axis(r*[-1 1 -1 1 0 2]);axis('square');
view(120,10);

% check the path length is indeed equal
dlvec=vecnorm(diff(pS')');
figure(3);plot(dlvec,'x')
dl=mean(dlvec);
disp(max(abs(dlvec-dl)));

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
    kth(:, i) = k*th;               %Angle-Axis Product
end

% plot out the end effector frame
m=5;
% MATLAB's plotTransforms command plots a frame at a given location
figure(2);h=plotTransforms(pS(:,1:m:end)',quat(:,1:m:end)');
set(h,'LineWidth',.5);



% ABB IRB 1200 parameters

L1=399.1;
L2=350;
L3=42;
L4=351;
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
R6T=[-ez ey ex];

% define abb 1200 robot using POE convention
irb1200.P=[p01 p12 p23 p34 p45 p56 p6T]/1000;
irb1200.H=[h1 h2 h3 h4 h5 h6];
irb1200.joint_type=[0 0 0 0 0 0];
irb1200.R6T=R6T;

% define collision body for abb 1200
radius=.01;
[irb1200_rbt,colLink]=defineRobot(irb1200,radius);
% 
% find all inverse kinematics solutions
%

for i=1:N
    % specify end effector SE(3) frame
    %Td{i}=[[xT(:,i) yT(:,i) zT(:,i)]*R6T' pS(:,i);[0 0 0 1]];
    Td{i}=[[xT(:,i) yT(:,i) zT(:,i)] pS(:,i);[0 0 0 1]];
    irb1200.T=Td{i};
    %
    irb1200=invkinelbow(irb1200); % << you need to supply this!!!
    %
    for k=1:8
        q(:,i,k)=irb1200.q(:,k);
    end
        % check forward kinematics to make sure the IK solution is correct
    for k=1:8
        irb1200.q=q(:,i,k);
        irb1200=fwddiffkiniter(irb1200);
        T{i,k}=irb1200.T;
    end

end

% choose the pose to visualize
ksol=1

for i=1:N
    % show robot pose (ever 5 frames)
    if mod(i,5)==0
        disp(norm(T{i,ksol}-Td{i}));
        figure(2);show(irb1200_rbt,q(:,i,ksol),'collision','on');
        view(150,10);
    end
end
% compute end effector linear and angular velcoity 

lsdot=.01;
for i=1:N-1
    dt(i) = (ls(i+1)-ls(i))/lsdot;
    for k=1:8
        qdot(:,i,k)=(q(:,i+1,k)-q(:,i,k))/dt(i);
        Ri1=T{i+1,k}(1:3,1:3);
        Ri=T{i,k}(1:3,1:3);
        w(:,i,k)=vee(Ri1*Ri'-eye(3,3))/dt(i);
        pi1=T{i+1,k}(1:3,4);
        pi=T{i,k}(1:3,4);
        v(:,i,k)=(pi1-pi)/dt(i);
    end
end

for k=1:8
   for i=1:6
       maxqdot(i,k)=max(qdot(i,:,k));
   end
   fprintf('maximum qdot for pose %d \n', k);
   disp(maxqdot(:,k)');   
end

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

function k = vee(R)
    k(1,1) = R(3,2);
    k(2,1) = R(1,3);
    k(3,1) = R(2,1);
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
    
    ex'*R(:,3);
    
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
    
    %Verify answer
    c1 = cos(beta1);
    s1 = sin(beta1);
    c2 = cos(beta2);
    s2 = sin(beta2);
    c3 = cos(beta3);
    s3 = sin(beta3);
    %R = RxBeta1*RyBeta2*RzBeta3
    R1 = [1 0 0; 0 c1 -s1; 0 s1 c1]*[c2 0 s2; 0 1 0; -s2 0 c2]*[c3 -s3 0; s3 c3 0; 0 0 1];
    R1 - R;
end
function M = hat(k)
    M = [0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
end

function robot=invkinelbow(robot)

end
