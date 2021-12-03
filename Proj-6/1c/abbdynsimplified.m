%
% define robot kinematics
%
%clear all; close all;

zz=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];

% symbolic ABB IRB 1200 robot

syms L1 L2 L3 L4 L5 positive
syms q1 q2 q3 q4 q5 q6 real

% define ABB IRB 1200 robot symbolically

% P
p01=0*ex+L1*ez;p12=zz;p23=L2*ez;p34=L3*ez+L4*ex;p45=zz;p56=zz;p6T=L5*ex;
% H
h1=ez;h2=ey;h3=ey;h4=ex;h5=ey;h6=ex;
% q
q=[q1;q2;q3;q4;q5;q6];

robot.P=[p01 p12 p23 p34 p45 p56 p6T];
robot.H=[h1 h2 h3 h4 h5 h6];
robot.joint_type=[0 0 0 0 0 0];
robot.q=q;

%
% Kinetic energy 
%

syms Lc1 Lc2a Lc2b Lc2c Lc3a Lc3b Lc3c Lc4 Lc5 Lc6 positive 
syms m1 m2 m3 m4 m5 m6 positive 
syms Ic1a Ic1b Ic1c Ic2a Ic2b Ic2c Ic3a Ic3b Ic3c positive
syms Ic4a Ic4b Ic4c Ic5a Ic5b Ic5c Ic6a Ic6b Ic6c positive
syms Ic3xz Ic3xy Ic3yz Ic2yz Ic2xz real
syms qd1 qd2 qd3 qd4 qd5 qd6 g real

pc(:,1)=Lc1*ez;pc(:,2)=Lc2b*ey+Lc2c*ez;pc(:,3)=Lc3a*ex+Lc3c*ez;
pc(:,4)=Lc4*ex;pc(:,5)=zz;pc(:,6)=Lc6*ex;
robot.Pc=pc;
robot.m=[m1;m2;m3;m4;m5;m6];

n=6; % 3-link
robot.qdot=[qd1;qd2;qd3;qd4;qd5;qd6]; % thetadot

for i=1:n    
    ii=num2str(i);
    text1=['Ic{i}=diag([Ic',ii,'a Ic',ii,'b Ic',ii,'c]);'];
    eval(text1);
end
robot.Ic=Ic;
robot.g=g;
robot.hg=ez;

robot=LEdyn(robot);

