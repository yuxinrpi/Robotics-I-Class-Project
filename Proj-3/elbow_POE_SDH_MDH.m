
%
% set up elbow arm in symbolic form
%
clc; clear; close all;
fwdkin_examples;

disp('T_{0T} from POE method');
simplify(elbow.T)

%SDH

% set up SDH parameters
elbow1.d=[l1;0;0;l4;0;0];
elbow1.a=[0;l2;l3;0;0;0];
elbow1.alpha=sym([-pi/2;0;-pi/2;pi/2;-pi/2;0]);
elbow1.theta=[q1;q2-pi/2;q3;q4;q5;q6];
% calculate T_{06}
elbow1=fwdkinsdh(elbow1);
% additional transformation to match with POE end effector frame
T6T=[[0 0 1;0 -1 0; 1 0 0] l5*ez;[zeros(1,3) 1]];
% This should match with POE's T_{0T}
T0T_SDH=simplify(elbow1.T*T6T);

disp('T_{0T} from SDH method');
disp(T0T_SDH);

% check
disp('difference between POE and SDH forward kinematics');
simplify(elbow.T-T0T_SDH)

%MDH

% set up MDH parameters
elbow2.d=[l1;0;0;l4;0;0];
elbow2.a=[0;0;l2;l3;0;0];
elbow2.alpha=sym([0;-pi/2;0;-pi/2;pi/2;-pi/2]);
elbow2.theta=[q1;q2-pi/2;q3;q4;q5;q6];
% calculate T_{07}
elbow2=fwdkinmdh(elbow2);
% additional transformation to match with POE end effector frame
T7T=[[[0 0 1;0 -1 0; 1 0 0] l5*ez];[zeros(1,3) 1]];
% This should match with POE's T_{0T|
T0T_MDH=simplify(elbow2.T*T7T);

disp('T_{0T} from MDH method');
disp(T0T_MDH);

% check

disp('difference between POE and MDH forward kinematics');
simplify(elbow.T-T0T_MDH)
