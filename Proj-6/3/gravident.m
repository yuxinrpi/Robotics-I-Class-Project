% 
% gravity term identiication
%
% write gravity term as G(q)=A(q)x, and estimate x based on our approximate
% A(q) from the Lagrange-Euler dynamical equation
%

% load MATLAB's ABB IRB 120
abb=loadrobot('abbIrb120','DataFormat','column','Gravity',[0;0;-9.81]);

% # of random poses
Nq=50;
% # of unknown parameters
nx=12;

% randomly generate joint angles
n=6;
q=(rand(n,Nq)-.5)*2*pi;
tau_g=zeros(6*Nq,1);
A=zeros(6*Nq,nx);

% use MATLAB's inverse dynamics command to compute just the gravity
abb.Gravity=9.81*[0;0;-1];

% use the measured gravity and joint angles to estimate the unknown
% parameters x
for i=1:Nq
    tau_g((i-1)*6+1:i*6)=inverseDynamics(abb,q(:,i));
    A((i-1)*6+1:i*6,:)=gravAq(q(:,i),nx);
end

% least square solution of x
x=pinv(A)*tau_g;

% overall error
disp('overall fit error: ||Ax-tau||');
norm(A*x-tau_g)

% comparing gravity on each joint 
tau=reshape(tau_g,6,Nq);
tauhat=reshape(A*x,6,Nq);

disp('fit error for each axis: ||Ax-tau||');
for i=1:6
%    figure(i+100);
%    plot((1:Nq),tau(i,:),'o',(1:Nq),tauhat(i,:),'x','linewidth',2);
    disp(sprintf('i = %d, G_i error: %g',i,...
        norm(tau(i,:)-tauhat(i,:))));
end

% for testing
% randomly generate joint angles
q=(rand(n,Nq)-.5)*2*pi;
tau_check=zeros(6,Nq);
tau_est=zeros(6,Nq);

for i=1:Nq
    tau_check(1:6,i)=inverseDynamics(abb,q(:,i));
    tau_est(1:6,i)=gravAq(q(:,i),nx)*x;
end

disp('testing error based on ramdom samples for each axis: ||Ax-tau||');
for i=1:6
%    figure(i+200);
%    plot((1:Nq),tau_check(i,:),'o',(1:Nq),tau_est(i,:),'x','linewidth',2);
    disp(sprintf('i = %d, G_i error: %g',i,...
        norm(tau_check(i,:)-tau_est(i,:))));
end

%
% regressor for the gravity load
%

function Aq=gravAq(q,nx)

s2=sin(q(2));
s23=sin(q(2)+q(3));
c23=cos(q(2)+q(3));
s4=sin(q(4));
c4=cos(q(4));
s5=sin(q(5));
c5=cos(q(5));

Aq=[...
    zeros(1,nx);
    s2 s23 c23 s23*c4*s5 c23*c5 zeros(1,nx-5);...
    zeros(1,5) s23 c23 s23*c4*s5 c23*c5 zeros(1,nx-9);...
    zeros(1,9) c23*s4*s5 zeros(1,nx-10);...
    zeros(1,10) s23*s5 c23*c4*c5;...
    zeros(1,nx)];

end

