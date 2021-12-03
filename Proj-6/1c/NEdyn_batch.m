%
% Newton Euler dynamics of a serial arm (symbolic) -- Batch method
% 
% input: robot structure with following fields
%           q: nx1 joint vector
%           qdot: nx1 joint velocity vector
%           P: link displacements 3x(n+1)
%           H: joint axis 3xn
%           Pc: CM vectors in link frame 3xn
%           Ic: n cells each 3x3 inertia about center of mass
%           m: nx1 link mass vector
% output: additional robot structure fields
%           M: mass-inertia matrix
%           G: gravity load
%           Jtr: Jacobian transpose (J_T^T in the end effector frame)
%           

function robot=NEdyn_batch(robot)

% constant vectors
zz=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];

% compute T_{0i} and (J_i)_i (partial Jacobian in ith frame)
n=size(robot.q,1);
q=robot.q;
Ic=robot.Ic;
m=robot.m;
Pc=robot.Pc;
Hmat=zeros(6*n,6);
Phimat=eye(6*n,6*n);
Emat=zeros(6*n,6);
Bmat=zeros(6*n,6);
Mmat=zeros(6*n,6*n);

% iterative forward kinematics
for i=1:n
    irange=((i-1)*6+1:6*i);
    h=robot.H(1:3,i);
    if robot.joint_type(i)==0
        R=expm(hat(h)*q(i));p=robot.P(1:3,i);zeros(1,3);
    else
        R=eye(3,3);p=robot.P(1:3,i)+q(i)*h;
    end
    if i==1
       Emat(1:6,:)=phi(R',p);
    else
      Phimat(irange,1:(i-1)*6)=phi(R',p)*Phimat((i-2)*6+1:6*(i-1),1:(i-1)*6);  
    end
    if robot.joint_type(i)==0
        Hmat(irange,i)=[h;zeros(3,1)];
    else
        Hmat(irange,i)=[zeros(3,1);h];
    end
    I{i}=Ic{i}-robot.m(i)*hat(robot.Pc(:,i))*hat(robot.Pc(:,i));
    Mmat(irange,irange)=...
        [I{i} robot.m(i)*hat(robot.Pc(:,i)); ...
        -robot.m(i)*hat(robot.Pc(:,i)) robot.m(i)*eye(3,3)];
end

Bmat((n-1)*6+1:6*n,:)=phi(eye(3,3),robot.P(1:3,n+1)); 

% return values
robot.Hmat=Hmat;
robot.Mmat=Mmat;
robot.Phimat=Phimat;
robot.M=Hmat'*Phimat'*Mmat*Phimat*Hmat;
robot.G=Hmat'*Phimat'*Mmat*Phimat*Emat*robot.nudot0;
robot.Jtr=Hmat'*Phimat'*Bmat;

end

%%%% 

function vhat=hat(v)
    vhat=[0 -v(3) v(2);v(3) 0 -v(1);-v(2) v(1) 0];
end

function Phi=phi(R,p)

Phi=[R zeros(3,3);-R*hat(p) R];

end