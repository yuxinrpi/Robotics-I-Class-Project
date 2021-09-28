% Mini-Project 2
% Yuxin Hu
% Kinematic Control for an RRR robot to follow the S-shape
% Geometric Inverse Kinematics
% 
% Note: This file was heavily modified from the class example

clear all;close all;clc;

%
% load the letter S as a curve
%
load S_letter_path

% Initial Plot of the path
figure(1);plot(Sls(1,:),Sls(2,:),Sls(1,1),Sls(2,1),'o','linewidth',2);
hold on;
plot_normal(Sls); %Mark the normal vector on the plot with quiver
xlabel('x');ylabel('y');
title('S Letter Curve Path with Normal Vectors (Yuxin Hu)');
axis('square');grid;

% Initialize a list of normal vectors
[xT,yT]=setR0T(Sls);

% Unit Vectors
zz=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];

% robot parameters
% De-comment other parameters to try other configurations
l1 = 1.5; l2 = 1.5; l3 = 0.5;
%l1 = 1.5; l2 = 1.5; l3 = 0.1;
%l1 = 2; l2 = 1.5; l3 = 0.25;
%l1 = 1.5; l2 = 1.5; l3 = 1;
robot.P = [zz l1*ex l2*ex l3*ex];
robot.H = [ez ez ez];
robot.joint_type=[0 0 0];

% zero configuration 
% Initialize and plot the robot
robot.q=[0;0;0];
radius = .01;
robot_rb=defineRobot(robot,radius);
figure(10);hold on;show(robot_rb,robot.q,'Collision','on');
view(0,90);
axis([0,4,-2,2,-2,2]);
axis('square');
xlabel('x-axis');ylabel('y-axis');
title('Planar RRR arm in zero configuration (q_1=q_2=q_3=0)')

% Using Inverse Kinematics to obtain two sets of solutions
nl=length(Sls);
qsol1=zeros(3,nl-1);
qsol2=zeros(3,nl-1);
for i=1:nl-1
    robot.T(1:3,1:4)=[xT(:,i) yT(:,i) ez [Sls(:,i);0]];
    qsol=threelink_invkin_geometric(robot);
    qsol1(:,i)=qsol(:,1);
    qsol2(:,i)=qsol(:,2);    
end

% Plot the first set of solution
figure(2);plot(Sls(1,:),Sls(2,:),Sls(1,1),Sls(2,1),'o','linewidth',2);
hold on
axis([-1,3,-2,2,-2,2])
axis('square');
for i=1:3:nl-1
    robot.q=qsol1(:,i);
    show(robot_rb,robot.q,'Collision','on'); 
    pause(.1);
    view(0,90);
end
xlabel('x');
ylabel('y');
title('First set of solution of Inverse Kinematics (Yuxin Hu)');
grid;
hold off;
fprintf('max joint speed: %5.4f, %5.4f, %5.4f\n',max(abs(diff(qsol1')')')');

% Plot the second set of solution
figure(3);plot(Sls(1,:),Sls(2,:),Sls(1,1),Sls(2,1),'o','linewidth',2);
hold on
axis([-1,3,-2,2,-2,2])
axis('square');
for i=1:3:nl-1
    robot.q=qsol2(:,i);
    show(robot_rb,robot.q,'Collision','on'); 
    view(0,90);
end
xlabel('x');
ylabel('y');
title('Second set of solution of Inverse Kinematics (Yuxin Hu)');
grid;
fprintf('max joint speed: %5.4f, %5.4f, %5.4f\n',max(abs(diff(qsol2')')')');

% Verification of Forward and Inverse Kinematics Functions
% Generate three random q in [-pi, pi]
% Process the forward Kinematics then solve the inverse Kinematics
% Verify if the original configuration is in the solutions
qmin = -pi;
qmax = pi;
q = (qmin + rand(1,3)*(qmax-qmin).');

% Forward
robot.q = q;
[T, J]=threelink_forkin(robot);
robot.T = T;
robot.J = J;

% Inverse
qs=threelink_invkin_geometric(robot);

%Output
fprintf('Forward Kinematics Input  q   : %2.3f, %2.3f, %2.3f.\n', q(1), q(2), q(3))
fprintf('Inverse Kinematics Output q(1): %2.3f, %2.3f, %2.3f.\n', qs(1,1), qs(2,1), qs(3,1))
fprintf('Inverse Kinematics Output q(2): %2.3f, %2.3f, %2.3f.\n', qs(1,2), qs(2,2), qs(3,2))

% Since == is not reliable for float numbers, check the norm between vector
d1 = norm(q.' - qs(:,1));
d2 = norm(q.' - qs(:,2));
if d1 < .001 || d2 <.001 % Set a tiny threshold for float computing errors
    fprintf('Forward and Inverse Kinematics Function Test Passed!\n');
else
    fprintf('Forward and Inverse Kinematics Function Test Failed!\n');
end

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
% 3D rotation matrix
% Imported from class example.
%
% input = theta (angle in Rad)
% output = 3x3 SO(3) matrix
%
function R=rot3(theta)
    c=cos(theta);s=sin(theta);
    R=[c -s 0;s c 0;0 0 0];
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
% Plot outward normal vectors in the plot
%
% input = Sls, The path to follow
% output = None
%
function plot_normal(Sls)
    k = pi/2; %Clockwise 90 deg
    R = rot2(k);
    size = 0.5; %The relative size of the arrow
    % The slope is calculated by the difference between vectors
    for i = 2:length(Sls)
        p = R*(Sls(:, i) - Sls(:, i-1));
        p = p/norm(p)*size;
        Sp(1:2, i) = p;
    end
    %Assume the slope for the first point equal to that of the second one
    Sp(:,1) = Sp(:,2);
    
    %Plot
    for i = 1:length(Sls)
        q = quiver(Sls(1,i), Sls(2,i), Sp(1, i), Sp(2, i), 0.3, 'linewidth',2);
    end
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

%
% Forward Kinematics for Three Link RRR robot
%
% Input = robot, with joint angle q
% Output = [T, J], the new Transform and Jacobian Matrix of the config
%
function [T, J]=threelink_forkin(robot)
    q = robot.q;
    
    % Length of each element
    l1 = robot.P(1,2);
    l2 = robot.P(1,3);
    l3 = robot.P(1,4);
    
    % As defined, 
    % xT = l1*c1 + l2*c12 + l3*c123
    % yT = l1*s1 + l2*s12 + l3*s123
    x1 = l1*cos(q(1));
    x2 = x1 + l2*cos(q(1)+q(2));
    x3 = x2 + l3*cos(q(1)+q(2)+q(3));
    y1 = l1*sin(q(1));
    y2 = y1 + l2*sin(q(1)+q(2));
    y3 = y2 + l3*sin(q(1)+q(2)+q(3));
    
    %The task position and angle
    p = [x3;y3];
    qT = q(1) + q(2) + q(3);
    
    %process Jacobian Matrix
    ez = [0;0;1];
    J = zeros(3,3);
    J(:,1) = cross(ez, [x3- 0;y3- 0;0]);
    J(:,2) = cross(ez, [x3-x1;y3-y1;0]);
    J(:,3) = cross(ez, [x3-x2;y3-y2;0]);
    
    %Process Transform Matrix
    T = zeros(4,4);
    T(1:2, 1:2) = rot2(qT);
    T(1:2, 4) = p;
end

%
% Geometrix Inverse Kinematics Solver for Three Link RRR robot
%
% Input = robot, with end effector pose T
% Output = qsol, a 3-by-2 matrix with solution q1 at qsol(:,1),
%                                 and solution q2 at qsol(:,2)
%
function qsol=threelink_invkin_geometric(robot)
    % Note: The robot is modeled as
    % 01 ---- 2 ---- 3 --- T
    % ^q1     ^q2    ^q3
    
    % Fetch data from robot
    R = robot.T(1:3,1:3);
    pT = robot.T(1:3,4);
    l1 = robot.P(:,2);
    l2 = robot.P(:,3);
    l3 = robot.P(:,4);
    
    % find p2 by the effector pose
    p3 = pT - R*l3;
    q = find_angle([0;0;1], [1;0;0], p3);
    
    % Apply law of cosine
    a = l1(1,1);
    b = l2(1,1);
    q1 = acos((a^2 + norm(p3)^2 - b^2)/(2*a*norm(p3)));
    q1_1 =   q1 + q;
    q1_2 = - q1 + q;
    
    % Find other q
    p2_1 = rot3(q1_1)*l1;
    q2_1 = find_angle([0;0;1], p2_1,p3-p2_1);
    q3_1 = find_angle([0;0;1], p3-p2_1,pT-p3);
    
    p2_2 = rot3(q1_2)*l1;
    q2_2 = find_angle([0;0;1], p2_2,p3-p2_2);
    q3_2 = find_angle([0;0;1], p3-p2_2,pT-p3);
    
    qsol = [q1_1 q1_2; q2_1 q2_2; q3_1 q3_2];
    
    % If q falls out of [-pi, pi], add or minus pi
    qsol = (qsol > pi)*(-pi) + (qsol < -pi)*(pi) + qsol;
end