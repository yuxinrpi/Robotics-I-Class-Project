%   Mini Project 1
%   Yuxin Hu
%   RIN: 661956455
%   CSCI-4480

clc;
clear all;
close all;

%-------------------------Initialization Start-------------------------
%|   To make everthing clear, the position and rotation of each       |
%|       element is represented by pAB and RAB, for example,          |
%|       pRoomRobot and RRoomRobot.                                   |
%|   Also the direction (e2) in degrees is represented in thA, for    |
%|       example thTable = 45.                                        |
%-------------------------Initialization Start-------------------------

%Initialize Robot
rRobot = 0.3;
zRobot = 0.1;

p10 = [-0.5; 4];
R10 = [0 1; -1 0];
pRoomRobot = R10*p10; % pRobotRoom in Room Frame = RRobotRoom * (pRobotRoom in Robot Frame)
RRoomRobot = rot2(-m2t(R10)); % RRoomRobot = RRobotRoom^-1
thRobot = rad2deg(m2t(RRoomRobot));

robot = collisionCylinder(rRobot, zRobot);
robot.Pose(1:2,4)=pRoomRobot;
robot.Pose(1:2,1:2)=RRoomRobot;


%Initialize Person
rPerson = 0.2;
zPerson = 0.1;
pPersonRobot = [3; 0]; % Should be fine since Person direction align with Room direction
pRoomPerson = pRoomRobot - pPersonRobot;
thPerson = deg2rad(0); % facing north
RRoomPerson = rot2(thPerson);

person = collisionCylinder(rPerson, zPerson);
person.Pose(1:2,4)=pRoomPerson;
person.Pose(1:2,1:2)=RRoomPerson;

%Intialize some variable
lRoom = [5, 5];
zEverything = 0.1;

%Initialize Table
lTable = 0.5; 
pRoomTable = lRoom/2; % center of room
thTable = 45; % NorthEast
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
showroom(1,5,5); % The view boundary is the room
hold on;
show(robot);
show(person);
show(table);
show(shelf);
title('Part 1 - Initial Position of each element');
hold off;
%----------------Initialization Complete---------------



%------------------------Part 2------------------------
%   No high-end technique here. Just some basic geometry
%   (a)Find p_a* and theta_a*( Represented by p_a and th_a )
%       Robot facing the front of the Shelf, and its edge
%       at 0.1m from shelf edge.
%   (b)Find p_b* and theta_b*( Represented by p_b and th_b )
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
% (a) See report

% (b) Find a path to a then b.

% Represent th_a th_b in rads
w_a = deg2rad(th_a);
w_b = deg2rad(th_b);

showroom(2,5,5); hold On;
show(robot);
show(person);
show(table);
show(shelf);

p0 = robot.Pose(1:2, 4);
p1 = [3;3];
moveRobot(robot, 20, p1, w_a);


p2 = p_a;
moveRobot(robot, 20, p2, w_a);

p3 = p_b;
moveRobot(robot, 20, p3, w_b);

title('Part 3(b) - Minimum straight line paths');
hold off;

% (c) Set Speed Limit
%     |ui| <= 2 m/s, |w| < 1 rad/s
limits = [2 1];

% Reset the Robot to its starting point and starting orientation
robot.Pose(1:2,4)=p0;
robot.Pose(1:2,1:2)=rot2(deg2rad(thRobot));

showroom(3,5,5); hold On;
show(robot);
show(person);
show(table);
show(shelf);

% Time step for the simulation
% Here I used 0.01 to ensure a higher accuracy
step = 0.01;

p1 = [3.05;2.85];
path1 = moveRobot(robot, step, p1, w_a, limits);
p2 = p_a;
path2 = moveRobot(robot, step, p2, w_a, limits);
p3 = p_b;
path3 = moveRobot(robot, step, p3, w_b, limits);

title('Part 3(c) - Path with speed limits');
hold off;

%Simulation complete, wrap all records together
path = [path1 path2 path3];

% Plot XYW speed against time
figure; hold on;
t = 0:step:(length(path)-1)*step;
plot(t,path(4,:),'linewidth',2)
plot(t,path(5,:),'linewidth',2)
plot(t,path(6,:))
legend('X-Speed', 'Y-Speed', 'W-Speed');
title(['Part 3(c) - Speed-Time plot: ', num2str(length(path)*step), '(s)']);
hold off
%--------------------Part 3 Complete--------------------



%-----------------------Functions-----------------------
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
% Omni-direction Robot
% Limit the speed while keeping the direction
% input:    U = X speed
%           V = Y speed
%           W = Rotation speed
%           vmax = max speed in any dir
%           wmax = max rotation speed in any dir
% output:   pdot = 2 element array of speed in X and Y dir
%           wdot = rotation speed
%

function [pdot, wdot] = omnivehi(U, V, W, vmax, wmax)
    m = max([abs(U)/vmax, abs(V)/vmax, abs(W)/wmax]);
    Xdot = U/m;
    Ydot = V/m;
    wdot = W/m;
    pdot = [Xdot; Ydot];
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
% Imported from class example.
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
%           N= (1) when limits is not given , equals number of points
%              (2) when limits is given, equals time step (s).
%           p1=ending point
%           w1=ending orientation
%           limits = (OPTIONAL) 4 element array that represents Vmax Vmin Wmax Wmin
%                    If the variable does not exist, ignore the limits
%
% output:   path = a 2D array with dimension (6, x)
%                   where path(1) is the x-position
%                         path(2) is the y-position
%                         path(3) is the w-position
%                         path(4) is the x-speed
%                         path(5) is the y-speed
%                         path(6) is the w-speed
%                   In mode 0(Without speed limit), row 4,5,6 are zeros.

function path=moveRobot(robot, N, p1, w1, limits)

    mode = 0; % Default mode - Without speed limits
    if nargin > 4 % If arg limits is given - With speed limits
        mode = 1; 
        vmax = limits(1);
        wmax = limits(2);
    end
    p0 = robot.Pose(1:2, 4);
    w0 = m2t(robot.Pose(1:2,1:2));
    w = w0;
    
    % --------------------------Part A--------------------------
    % |  This part is for Problem 2.b, where the max speed     |
    % |      is not considered. The robot is able to move and  |
    % |      rotate in any speed.                              |
    % --------------------------Part A--------------------------
    if mode == 0
        path = zeros(6, N+1);      
        path(1:2,1) = p0;
        path(3,1) = w0;
        for i=2:N+1
            path(1,i) = p0(1) + (i-1)*(p1(1)-p0(1))/N;
            path(2,i) = p0(2) + (i-1)*(p1(2)-p0(2))/N;
            path(3,i) = w0 + (i-1)*(w1-w0)/N;
            robot.Pose(1:2, 4) = path(1:2,i);
            robot.Pose(1:2, 1:2) = rot2(path(3,i));
        end
        showquiver(path, 2);

    % --------------------------Part B--------------------------
    % |  This part is for Problem 2.c, where the max speed     |
    % |      is considered. The robot is able to move and      |
    % |      rotate in a restricted speed.                     |
    % --------------------------Part B--------------------------
    else
        p = p0;
        step = N; %time step 0.1s
        tmax = max(max(abs(p1-p0)/vmax),abs(w1-w0)/wmax);
        path = zeros(6, floor(tmax/step)+1);
        path(1:2,1) = p0;
        path(3,1) = w0;
        for i=2:length(path)
            %Assume we can reach the goal at this iteration
            %Then use omni-direction vehicle to limit the speed
            wdot = (w1 - w)/step;
            v = (p1-p)/step;
            [pdot, wdot] = omnivehi(v(1,1), v(2,1), wdot, vmax, wmax);
            p = p + pdot*step;
            w = w + wdot*step;
            
            path(1:2,i) = p;
            path(3,i) = w;
            path(4:5,i) = pdot;
            path(6,i) = wdot;
            robot.Pose(1:2, 4) = p;
            robot.Pose(1:2, 1:2) = rot2(w);
            %show(robot);
        end
    end
    showquiver(path, 10);
    plot(path(1,:), path(2,:), 'LineWidth', 3);
    show(robot);
end

%
%   Plot quiver arrows
%
%   input: path = the path matrix generated by moverobot() function
%          N = The arrow density, plot 1/N of total arrows
%
%   output: None
%

function showquiver(path, N)
    count = 0;
    for i = 1:length(path)
        if mod(count,N) == 0
            a = cos(path(3,i)+pi/2);
            b = sin(path(3,i)+pi/2);
            q = quiver(path(1,i), path(2,i), a, b, 0.3, 'linewidth',2);
            q.Marker = '.';
            q.ShowArrowHead = 'off';
            count = 0;
        end
        count = count + 1;
    end
end
