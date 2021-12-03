% 
% gradient of gravity load
%
% write gravity term as G(q)=A(q)x, 
% Then grad G(q) = [\partial G/\partial q_1, ... \partial G/\partial q_n]
%               = [\partial A(q) \partial q_1 x, ... \partial A(q) \partial q_n x]
%

% define joint vector
function gradG=gravgradnum(q,x)

n=length(q);
nx=length(x);

gradG=(zeros(n,n));

s2=sin(q(2));
c2=cos(q(2));
s23=sin(q(2)+q(3));
c23=cos(q(2)+q(3));
s4=sin(q(4));
c4=cos(q(4));
s5=sin(q(5));
c5=cos(q(5));

% third argument is \partial q_i
gradA_q=(zeros(6,12,6));

gradA_q(2,:,2)=[c2 c23 -s23 c23*c4*s5 -s23*c5 zeros(1,nx-5)];
gradA_q(2,:,3)=[0 c23 -s23 c23*c4*s5 -s23*c5 zeros(1,nx-5)];
gradA_q(2,:,4)=[0 0 0 -s23*s4*s5 0 zeros(1,nx-5)];
gradA_q(2,:,5)=[0 0 0 s23*c4*c5 -c23*s5 zeros(1,nx-5)];

gradA_q(3,:,2)=[zeros(1,5) c23 -s23 c23*c4*s5 -s23*c5 zeros(1,nx-9)];
gradA_q(3,:,3)=[zeros(1,5) c23 -s23 c23*c4*s5 -s23*c5 zeros(1,nx-9)];
gradA_q(3,:,4)=[zeros(1,5) 0 0 -s23*s4*s5 0 zeros(1,nx-9)];
gradA_q(3,:,5)=[zeros(1,5) 0 0 s23*c4*c5 -c23*s5 zeros(1,nx-9)];

gradA_q(4,:,2)=[zeros(1,9) -s23*s4*s5 zeros(1,nx-10)];
gradA_q(4,:,3)=[zeros(1,9) -s23*s4*s5 zeros(1,nx-10)];
gradA_q(4,:,4)=[zeros(1,9) c23*c4*s5 zeros(1,nx-10)];
gradA_q(4,:,5)=[zeros(1,9) c23*s4*c5 zeros(1,nx-10)];

gradA_q(5,:,2)=[zeros(1,10) c23*s5 -s23*c4*c5];
gradA_q(5,:,3)=[zeros(1,10) c23*s5 -s23*c4*c5];
gradA_q(5,:,4)=[zeros(1,10) 0 -c23*s4*c5];
gradA_q(5,:,5)=[zeros(1,10) s23*c5 -c23*c4*s5];

for i=2:5
    gradG(:,i)=squeeze(gradA_q(:,:,i))*x;
end

% Aq=[...
%     zeros(1,nx);
%     s2 s23 c23 s23*c4*s5 c23*c5 zeros(1,nx-5);...
%     zeros(1,5) s23 c23 s23*c4*s5 c23*c5 zeros(1,nx-9);...
%     zeros(1,9) c23*s4*s5 zeros(1,nx-10);...
%     zeros(1,10) s23*s5 c23*c4*c5;...
%     zeros(1,nx)];

end

