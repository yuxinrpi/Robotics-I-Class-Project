%
% Lagrange Euler dynamics of a serial arm (symbolic)
% 
% input: robot structure with following fields
%           q: nx1 joint vector
%           qdot: nx1 joint velocity vector
%           P: link displacements 3x(n+1)
%           H: joint axis 3xn
%           Pc: CM vectors in link frame 3xn
%           Ic: n cells each 3x3 inertia about center of mass
%           m: nx1 link mass vector
%

function robot=LEdyn(robot)

% constant vectors
zz=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];

% compute T_{0i} and (J_i)_i (partial Jacobian in ith frame)
n=size(robot.q,1);
T=eye(4,4);
J=zeros(6,n);
q=robot.q;
Ic=robot.Ic;
m=robot.m;
Pc=robot.Pc;

% iterative forward kinematics
for i=1:n
    h=robot.H(1:3,i);
    if robot.joint_type(i)==0
        R=expm(hat(h)*q(i));p=robot.P(1:3,i);zeros(1,3);
    else
        R=eye(3,3);p=robot.P(1:3,i)+q(i)*h;
    end
    T=T*[R p;zeros(1,3) 1];
    % this is different than what we used before
    % this is J_i represented in ith frame rather than in the base frame
    J=phi(R',p)*J;
    if robot.joint_type(i)==0
        J(:,i)=[h;zeros(3,1)];
    else
        J(:,i)=[zeros(3,1);h];
    end
    Ji{i}=J;
    % ith link potential energy
    % P_i = m g hg^T ( p_0i + R_0i * p_ci)
    Plink{i}=robot.m(i)*robot.g*robot.hg'*(T(1:3,4)+T(1:3,1:3)*robot.Pc(:,i));
end
%
% compute mass matrix
%

for i=1:n
    I{i}=Ic{i}-robot.m(i)*hat(robot.Pc(:,i))*hat(robot.Pc(:,i));
    Mlink{i}=[I{i} robot.m(i)*hat(robot.Pc(:,i)); ...
        -robot.m(i)*hat(robot.Pc(:,i)) robot.m(i)*eye(3,3)];
    Mqlink{i}=Ji{i}'*Mlink{i}*Ji{i};
end

P=0;
M=sym(zeros(n,n));
for i=1:n
    M=M+Mqlink{i};
    P=P+Plink{i};
end

%
% Centrifugal and Coriolis terms
% 

qdot=robot.qdot;
for i=1:n
  for j=1:n
    for k=1:n      
      Mdiff(i,j,k)=diff(M(i,j),q(k));
    end
  end
end

MD=M;
for i=1:n
  for k=1:n
    MD(i,k)=(Mdiff(i,:,k)*qdot);
  end
end

C=(MD-.5*MD.');

%
% Gravity Load
%

for i=1:n
  G(i,1)=diff(P,q(i));
end

% return values
robot.M=simplify(M);
robot.C=C;
robot.PE=P;
robot.G=simplify(G);
robot.Ji=Ji;
robot.Mlink=Mlink;

end

%%%% 

function vhat=hat(v)
    vhat=[0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];
end

function Phi=phi(R,p)

Phi=[R zeros(3,3);-R*hat(p) R];

end