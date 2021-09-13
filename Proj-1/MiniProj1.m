%   Mini Project 1
%   Yuxin Hu
%   RIN: 661956455
%   CSCI-4480


clear all;
close all;

%-------------------Initialization Start-------------------
%
%   To make everthing clear, the position and rotation of each
%       element is represented by pAB and RAB, for example,
%       pRoomRobot and RRoomRobot.
%   Also the direction (e2) in degrees is represented in thA, for
%       example thTable = 45.
%

%Initialize Robot
rRobot = 0.3;
zRobot = 0.1;

p10 = [-0.5; 4];
R10 = [0 1; -1 0];
pRoomRobot = R10*p10; %pRobotRoom in Room Frame = RRobotRoom * (pRobotRoom in Robot Frame)
RRoomRobot = rot2(-m2t(R10)); %RRoomRobot = RRobotRoom^-1
thRobot = rad2deg(m2t(RRoomRobot));

robot = collisionCylinder(rRobot, zRobot);
robot.Pose(1:2,4)=pRoomRobot;
robot.Pose(1:2,1:2)=RRoomRobot;


%Initialize Person
rPerson = 0.2;
zPerson = 0.1;
pPersonRobot = [3; 0];%Should be fine since Person direction align with Room direction
pRoomPerson = pRoomRobot - pPersonRobot;
thPerson = deg2rad(0); %facing north
RRoomPerson = rot2(thPerson);

person = collisionCylinder(rPerson, zPerson);
person.Pose(1:2,4)=pRoomPerson;
person.Pose(1:2,1:2)=RRoomPerson;

%Intialize some variable
lRoom = [5, 5];
zEverything = 0.1;

%Initialize Table
lTable = 0.5; 
pRoomTable = lRoom/2; %center of room
thTable = 45; %NorthEast
RRoomTable = rot2(deg2rad(thTable));

table = collisionBox(lTable, lTable, zEverything);
table.Pose(1:2, 4)=pRoomTable;
table.Pose(1:2, 1:2)=RRoomTable;

%Initialize Shelf
lShelf = [0.8 0.3];
shelf = collisionBox(lShelf(1), lShelf(2), zEverything);
thShelf = -90;%facing east
pPersonShelf = [0;3.5];
pRoomShelf = pRoomPerson + pPersonShelf;
RRoomShelf = rot2(deg2rad(thShelf));
shelf.Pose(1:2, 4)=pRoomShelf;
shelf.Pose(1:2, 1:2)=RRoomShelf;

%Display initial position
showroom(1,5,5);
hold on;
show(robot);
show(person);
show(table);
show(shelf);
title('Part1 - Initial Position of each element');
hold off;
%----------------Initialization Complete---------------

%------------------------Part 2------------------------
%   No high-end technique here. Just some basic geometry
%   (a)Find p_a* and theta_a*( Represented by p_a and th_a)
%       Robot facing the front of the Shelf, and its edge
%       at 0.1m from shelf edge.
%   (b)Find p_b* and theta_b*( Represented by p_b and th_b)
%       Robot facing the front of the Person, and its edge
%       at 0.1m from shelf edge.

th_a = thShelf+180; %Facing the shelf's direction
p_a = pRoomShelf;
p_a(1,1) = p_a(1,1) + lShelf(2)/2 + 0.1 + rRobot;

th_b = thPerson+180;
p_b = pRoomPerson;
p_b(2,1) = p_b(2,1) + rPerson + 0.1 + rRobot;
%--------------------Part 2 Complete--------------------

%------------------------Part 3------------------------
% (a) I give up
% (b) Find a path to a then b.

showroom(2,5,5); hold On;
show(robot);
show(person);
show(table);
show(shelf);

p0 = robot.Pose(1:2, 4);
p1 = [3;3];
moveRobot(robot, 100, p0, p1, thRobot, th_a);


p2 = p_a;
moveRobot(robot, 20, p1, p2, th_a, th_a);


p3 = p2-[0;1]
moveRobot(robot, 20, p2, p3, th_a, th_b);

p4 = p_b;
moveRobot(robot, 20, p3, p4, th_b, th_b);




%-----------------------Functions-----------------------
%
% 2D rotation matrix
% 
% input = theta (angle in RAD)
% output = 2x2 SO(2) matrix
%
function R=rot2(theta)
    c=cos(theta);s=sin(theta);
    R=[c -s;s c];
end

%
% Convert 2D rotation matrix to theta
% 
% input = 2x2 SO(2) matrix
% output = theta (angle in deg)
%
function theta=m2t(mat)
    theta = atan2(mat(2,1),mat(1,1));
end

%
% show room 
%  
% input:    fignum = figure number
%           xdim = horizontal dimension of room
%           ydim = vertical dimension of room
%
function h=showroom(fignum,xdim,ydim)
    h=figure(fignum);
    % adjust the view so you can see from above
    view(0,90)
    % adjust the figure range so you can see the 5x5 room
    axis([0 xdim 0 ydim]);
    grid;
end

%
% plot line from p0 to p1
%
% input:    robot=robot
%           N=number of points
%           p0=starting point
%           p1=ending point
%           th0=starting orientation
%           th1=ending orientation
%

function path=moveRobot(robot, N, p0, p1, th0, th1)
    distance = norm(p1-p0);
    interval = distance/N;
    path = zeros(3, N+1);
    path(1:2,1) = p0;
    count = 0;
    for i=2:N+1
        path(1,i) = p0(1) + (i-1)*(p1(1)-p0(1))/N;
        path(2,i) = p0(2) + (i-1)*(p1(2)-p0(2))/N;
        path(3,i) = th0 + (i-1)*(th1-th0)/N;
        robot.Pose(1:2, 4) = path(1:2,i);
        %show(robot);
        
        %   In order to make a clearer view,
        %   Show only 1/5 quiver arrows 
        count = count + 1;
        if mod(count,5) == 0
            U = -sin(deg2rad(path(3,i)));
            V = cos(deg2rad(path(3,i)));
            q = quiver(path(1,i), path(2,i), U, V, 0.3, 'linewidth',2);
            q.Marker = '.';
            q.ShowArrowHead = 'off';
            count = 0;
        end
    end
    plot(path(1,:), path(2,:), 'LineWidth', 3);

    %show(robot);
end
