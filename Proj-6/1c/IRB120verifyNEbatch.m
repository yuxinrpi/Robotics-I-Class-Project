%
% initialize IRB120 robot based on the MATLAB parameters
%

%
% define robot kinematics
%
%clear all; close all;
clear Ic
zz=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];

% from MATLAB IRB120

abb=loadrobot('abbIrb120','DataFormat','column','Gravity',[0;0;-9.81]);

% link lengths
Lval(1)=abb.Bodies{3}.Joint.JointToParentTransform(3,4);
Lval(2)=abb.Bodies{4}.Joint.JointToParentTransform(3,4);
Lval(3)=abb.Bodies{5}.Joint.JointToParentTransform(3,4);
Lval(4)=abb.Bodies{6}.Joint.JointToParentTransform(1,4);
Lval(5)=abb.Bodies{7}.Joint.JointToParentTransform(1,4);

%
% define ABB IRB 1200 robot based on product of exponential

% P
p01=0*ex+Lval(1)*ez;
p12=zz;
p23=Lval(2)*ez;
p34=Lval(3)*ez+Lval(4)*ex;
p45=zz;
p56=zz;
p6T=Lval(5)*ex;
% H
h1=ez;h2=ey;h3=ey;h4=ex;h5=ey;h6=ex;

P=[p01 p12 p23 p34 p45 p56 p6T];
IRB120.P=P;
IRB120.H=[h1 h2 h3 h4 h5 h6];
IRB120.joint_type=[0 0 0 0 0 0];

% mass, center of mass, and inertia
m=zeros(n,1);
Pc=zeros(3,n);
for i=1:n
    m(i)=abb.Bodies{i+1}.Mass;
    Pc(:,i)=abb.Bodies{i+1}.CenterOfMass';
    Ic{i}=[abb.Bodies{i+1}.Inertia(1) abb.Bodies{i+1}.Inertia(6) ...
        abb.Bodies{i+1}.Inertia(5); abb.Bodies{i+1}.Inertia(6) ... 
        abb.Bodies{i+1}.Inertia(2) abb.Bodies{i+1}.Inertia(4); ...
        abb.Bodies{i+1}.Inertia(5) abb.Bodies{i+1}.Inertia(4) ...
        abb.Bodies{i+1}.Inertia(3)] + m(i)*hat(Pc(:,i))*hat(Pc(:,i));
end
Pc(:,1) = -P(:,1)+Pc(:,1);
Pc(:,4) = -Lval(4)*ex + Pc(:,4);
Pc(:,6) = P(:,n+1)+Pc(:,6);

%
% center of mass 
%

IRB120.Pc=Pc;
IRB120.m=m;
IRB120.Ic=Ic;

n=length(IRB120.joint_type);

q=(rand(n,1)-.5)*2*pi;
q=[  2.3335    0.4783    -1.9623     3.0970     2.4657    -3.0094]';
IRB120.q=q;
IRB120.qdot=randn(n,1)*.1;
IRB120.qddot=randn(n,1)*.5;
IRB120.nu0=zeros(6,1);
IRB120.nudot0=[zeros(3,1);9.81*ez];

IRB120=NEdyn_batch(IRB120);

% use MATLAB's inverse dynamics command to compute just the gravity
abb.Gravity=9.81*[0;0;-1];
abbG=inverseDynamics(abb,q);

% first remove gravity to focus just on the mass matrix
abb.Gravity=[0;0;0];
abbM=zeros(n,n);
% use MATLAB's inverse dynamics to compute the mass matrix
for i=1:n
    qddot=zeros(n,1);qddot(i)=1;
    abbM(:,i)=inverseDynamics(abb,q,zeros(n,1),qddot);
end

disp([IRB120.G-abbG])
disp([IRB120.M-abbM])

%%%

function khat = hat(k)
  
  khat=[0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
  
end