%
% find the linearized model 
% 

% first identify gravity to obtain the constant vector x
gravident;

% use the outstretched configuration
q=[0;pi/2;-pi/2;0;0;0];
%q=zeros(6,1);
% next find gradG
G_lin=gravgradnum(q,x);

% now find the M matrix
abb=loadrobot('abbIrb120','DataFormat','column','Gravity',[0;0;-9.81]);

% first remove gravity to focus just on the mass matrix
abb.Gravity=[0;0;0];
M_lin=zeros(n,n);
% use MATLAB's inverse dynamics to compute the mass matrix
for i=1:n
    qddot=zeros(n,1);qddot(i)=1;
    M_lin(:,i)=inverseDynamics(abb,q,zeros(n,1),qddot);
end
