
%
% forward kinematics examples
%

zz=zeros(3,1); ex = [1;0;0]; ey = [0;1;0]; ez = [0;0;1];

syms q1 q2 q3 q4 q5 q6 real
q=[q1;q2;q3;q4;q5;q6];

syms l0 l1 l2 l3 l4 l5 b1 b2 b3 b4 real

% elbow

elbow.H=[ez,ey,ey,ex,ey,ex];
elbow.P=[l1*ez,zz,l2*ez,l3*ez+l4*ex,zz,zz,l5*ex];
elbow.joint_type=[0 0 0 0 0 0];
elbow.q=q;
elbow=fwdkiniter(elbow);
%elbow=fwddiffkiniter(elbow);
% cross production function
function khat = hat(k)
  
  khat=[0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
  
end
