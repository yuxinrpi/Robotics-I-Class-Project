% # of links
n=6;

%
% parameters from ABB IRB 120 in MATLAB robotics systems toolbox adjusted
% to our coordinate system
% Note: MATLAB's inertia is with respect to each link frame origin, not the
% center of mass, so we needed to use the parallel-axis theorem to tranform
% to the inertia with respect to the center of mass that we use in our
% formulation.
%
m=[3.067    3.909    2.944    1.328    0.546    0.137]';
Pc=[
    0       0.0008    0.0228   -0.0773   0 0.0649;
    0       -0.0021    0.0011    0    0   0;
    0.0516   0.1012    0.0579    0    0   0 ];    

Ic=[0.0142    0.0144    0.0105;
    0.0603    0.0416    0.0260;
    0.0084    0.0167    0.0127;
    0.0028    0.0040    0.0053;
    0.4049e-3    0.8928e-3    0.8155e-3;
    1.0000e-3    1.0000e-3    1.0000e-3];

% link length numerical value
Lval=[0.2900    0.2700    0.0700    0.3020    0.0720];
% numerical link vector matrix
P=eval(subs(robot.P,{L1,L2,L3,L4,L5},{Lval(1),Lval(2),Lval(3),Lval(4),Lval(5)}));

% random choice of joint angles
q=(rand(n,1)-.5)*2*pi;

% use MATLAB's inverse dynamics command to compute just the gravity
abb.Gravity=9.81*[0;0;-1];
tau_rst=inverseDynamics(abb,q);

% use our LEdyn code output to compute gravity
gravload = subs(robot.G,{q1,q2,q3,q4,q5,q6,m1,m2,m3,m4,m5,m6,g,...
    L1,L2,L3,L4,L5,...
    Lc1,Lc2a,Lc2b,Lc2c,Lc3a,Lc3b,Lc3c,Lc4,Lc5,Lc6},...
    {q(1),q(2),q(3),q(4),q(5),q(6),m(1),m(2),m(3),m(4),m(5),m(6),9.81,...
    Lval(1),Lval(2),Lval(3),Lval(4),Lval(5),...
    Pc(3,1),0,Pc(2,2),Pc(3,2),Pc(1,3),0,Pc(3,3),Pc(1,4),0,Pc(1,6)});
tau_LE=eval(gravload);

% the two should be very close (we ignore some small elements in Pc)
disp([tau_rst,tau_LE])
disp(tau_rst-tau_LE)

%
% Now compare the mass matrix
%

% first remove gravity to focus just on the mass matrix
abb.Gravity=[0;0;0];
% use MATLAB's inverse dynamics to compute the mass matrix
for i=1:n
    qddot=zeros(n,1);qddot(i)=1;
    M_rst(:,i)=inverseDynamics(abb,q,zeros(n,1),qddot);
end

% use our LEdyn code ouptut to compute mass matrix
Mmat=subs(robot.M,{q1,q2,q3,q4,q5,q6,m1,m2,m3,m4,m5,m6,g,...
    L1,L2,L3,L4,L5,...
    Lc1,Lc2a,Lc2b,Lc2c,Lc3a,Lc3b,Lc3c,Lc4,Lc5,Lc6},...    
    {q(1),q(2),q(3),q(4),q(5),q(6),m(1),m(2),m(3),m(4),m(5),m(6),0,...
    Lval(1),Lval(2),Lval(3),Lval(4),Lval(5),...
    Pc(3,1),0,Pc(2,2),Pc(3,2),Pc(1,3),0,Pc(3,3),Pc(1,4),0,Pc(1,6)});

% substitute in inertia (approximate with just diagonal elements)
for i=1:n
    ii=num2str(i);
    Ictext=['{Ic',ii,'a,Ic',ii,'b,Ic',ii,'c}'];
    eval(['Mmat=subs(Mmat,',Ictext,',{Ic(i,:)});']);
end

Mmat=subs(Mmat,{Ic2yz,Ic2xz,Ic3xz,Ic3xy,Ic3yz},{0,0,0,0,0});
M_LE=eval(Mmat);

% the two should be very close (we ignore some small elements in Ic)
disp(M_rst);
disp(M_LE);
disp(M_rst-M_LE);