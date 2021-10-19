%
% fwdkinmdh.m
% 

function robot=fwdkinmdh(robot)

ex = [1;0;0]; ez = [0;0;1];

n=length(robot.theta);

T=eye(4,4);
for i=1:n
    T=T*[rotx(robot.alpha(i))*rotz(robot.theta(i)) ...
        robot.d(i)*rotx(robot.alpha(i))*ez+robot.a(i)*ex;...
        [0 0 0 1]];
end

robot.T=T;

end

function M = hat(k)
    M = [0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
end

function M = rotx(k)
    M = expm(k*hat([1;0;0]));
end
function M = roty(k)
    M = expm(k*hat([0;1;0]));
end
function M = rotz(k)
    M = expm(k*hat([0;0;1]));
end