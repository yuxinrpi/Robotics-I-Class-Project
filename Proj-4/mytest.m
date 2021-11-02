%
% mini-project 4 example script
% 
clear all; close all; clc;

zz=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];

% symbolic ABB IRB 1200 robot

syms L1 L2 L3 L4 L5 positive
syms q1 q2 q3 q4 q5 q6 real

% define ABB IRB 1200 robot symbolically

% P
p01=0*ex+L1*ez;p12=zz;p23=L2*ez;p34=L3*ez+L4*ex;p45=zz;p56=zz;p6T=L5*ex;
%p01=0*ex+L1*ez;p12=zz;p23=L2*ez;p34=L4*ex;p45=zz;p56=zz;p6T=L5*ex;

% H
h1=ez;h2=ey;h3=ey;h4=ex;h5=ey;h6=ex;

% q
q=[q1;q2;q3;q4;q5;q6];

% forward kinematics and Jacobian

irb1200_s.P=[p01 p12 p23 p34 p45 p56 p6T];
irb1200_s.H=[h1 h2 h3 h4 h5 h6];
irb1200_s.joint_type=[0 0 0 0 0 0];
irb1200_s.q=q;

irb1200_s1=fwddiffkiniter(irb1200_s);
J_1 = irb1200_s1.J

irb1200_s2 = mydiff2(irb1200_s);
J_2 = irb1200_s2.J

d = J_2 - J_1