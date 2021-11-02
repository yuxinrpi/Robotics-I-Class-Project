%% Initial Setup
clc;clear;close all;

proj4init;
%% Mode Selection
% Mode 1 -> Generate 50 ramdom points, and test invJac kinematics
% Mode 2 -> Load the Sphere S Letter Path and find q
mode = 1
if mode == 1
%% Mode 1 - 50 Points Testing
    n = 50;
    
    dt = .05; % unit step
    t=0:dt:150; % Time vector
    N=length(t);

    % weighting in damped least square to avoid excessive speed
    epsilon=10;
    % maximum joint speed
    umax=ones(6,1)*1;
    w = 5;
    Kp = 10;
    fprintf('This may take a while,\nPlease Stand By\n')
    
    for i = 1:n
        fprintf('>')
        % Generate a list of p, range = [-0.5, 0.5]
        pTd = 2*rand(3,1)-1;
        % Generate a list of R, range of beta = [-0.5pi, 0.5pi]
        h = normalize(2*rand(1, 3)-1)';
        beta = rand(1)*pi-pi/2;
        RTd = expm(hat(h)*beta);
        q = zeros(6,1);
        precord(:, i) = pTd;
        Rrecord(:,:,i) = RTd;
        for k = 1:N-1
            irb1200.q = q(:, k);
            %
            irb1200=mydiff(irb1200); % 
            %
            R = irb1200.T(1:3, 1:3);
            p = irb1200.T(1:3, 4);
            ER = R*RTd';
            qe = R2q(ER);
            eR1 = 4*(norm(qe(2:4)))^2;
            s1 = 4*qe(1)*qe(2:4);
            % form task space error 
            dX=[w*s1;p-pTd];

            % Jacobian kinematics control (soft constraint on qdot)
            u(:,k)=-Kp*irb1200.J'*inv(irb1200.J*irb1200.J'+epsilon*eye(6))*dX;
            % constraint the maximum speed
            u(:,k)=(u(:,k)>umax).*umax+(u(:,k)<-umax).*(-umax)+...
                (u(:,k)<umax).*(u(:,k)>-umax).*u(:,k);
            % update robot motion using qdot (based on finite difference)
            qq=q(:,k)+(t(k+1)-t(k))*u(:,k);
            % restrict to be in -pi to pi
            q(:,k+1)=(qq>pi).*(-2*pi+qq)+(qq<-pi).*(2*pi+qq)+(qq<pi).*(qq>-pi).*qq;
        end
        froberror(i) = norm(irb1200.T(1:3,1:3)*RTd'-eye(3),'fro');
    end
    fprintf('\nSimulation Complete!\nmax Frobenius Error is %5.4f\n',max(froberror))
    
elseif mode == 2
%% Mode 2 - S Letter Path
    R6T = eye(3);
    q0 = [0;0;0;0;0;0];
    irb1200.q = q0;

    % Time and Step setup
    dt = .1; % unit step
    t=0:dt:1000; % Time vector
    N=length(t);

    q=zeros(6,N);
    % initial arm configuration
    q(:,1)=q0;
    % proportional feedback gain
    Kp=diag(ones(6,1));


    % The threshold for Initial and Final point
    % Nthres - Frame dedicated for initial and final point
    % FPD - The rest frames will be evenly distributed to other points
    % N1 + FPD*(Number of points in Curve) + NF = Total Time steps
    Nthres=10;
    FPD=round(N/(length(pS)-2+2*Nthres));%Frame per dot
    N1 = Nthres*FPD;
    Nf = N - N1 - (length(pS)-2)*FPD;

    % weighting in damped least square to avoid excessive speed
    epsilon=1;
    % maximum joint speed
    umax=ones(6,1)*1;
    w = 1;

    id = 1;
    froberror = [0];
    for k=1:N-1
        oldid = id;
        % desired target
        if k<=N1
            RTd = R_T(:,:,1);
            pTd=pS(:,1);
            id = 1;
        elseif k>= N-Nf
            RTd = R_T(:,:,end);
            pTd=pS(:,end); 
            id = length(pS);
        else 
            id = floor((k-N1)/FPD)+1;
            RTd = R_T(:,:,id);
            pTd=pS(:,id);         
        end
        if oldid ~= id
            froberror(end+1) = norm(irb1200.T(1:3,1:3)*R_T(:,:,oldid)'-eye(3),'fro');
        end

        irb1200.q = q(:, k);
        %
        irb1200=mydiff(irb1200); % 
        %
        R = irb1200.T(1:3, 1:3);
        p = irb1200.T(1:3, 4);
        ER = R*RTd';
        qe = R2q(ER);
        eR1 = 4*(norm(qe(2:4)))^2;
        s1 = 4*qe(1)*qe(2:4);
        % form task space error 
        dX=[w*s1;p-pTd];

        % Jacobian kinematics control (soft constraint on qdot)
        u(:,k)=-Kp*irb1200.J'*inv(irb1200.J*irb1200.J'+epsilon*eye(6))*dX;
        % constraint the maximum speed
        u(:,k)=(u(:,k)>umax).*umax+(u(:,k)<-umax).*(-umax)+...
            (u(:,k)<umax).*(u(:,k)>-umax).*u(:,k);
        % update robot motion using qdot (based on finite difference)
        qq=q(:,k)+(t(k+1)-t(k))*u(:,k);
        % restrict to be in -pi to pi
        q(:,k+1)=(qq>pi).*(-2*pi+qq)+(qq<-pi).*(2*pi+qq)+(qq<pi).*(qq>-pi).*qq;

    end

    figure(6);
    plot3(pS(1,:),pS(2,:),pS(3,:),'rx','linewidth',3);
    xlabel('x');ylabel('y');zlabel('z');
    title("The IRB-1200/0.9 Robot Motion Path(Yuxin Hu)");
    hold on;
    % 3d sphere
    surf(X,Y,Z)
    % make it transparent
    alpha 0.4
    axis(r*[-1 1 -1 1 0 2]);axis('square');
    view(120,10);
    kinterval = 5;
    for k=1:FPD*kinterval:N
        show(irb1200_rbt,q(:,k),'collision','on');
        pause(.01);
    end

    grid;
    fprintf('max joint speed: %5.4f, %5.4f, %5.4f\n',max(abs(diff(q')')')')
    fprintf('max Frobenius Error is %5.4f\n',max(froberror))
end
% 
%
% R2q.m
%
% converts R in SO(3) to unit quaternion q, (q0,q_vec)
%

function q=R2q(R)
  
  q=zeros(4,1);
  q(1)=.5*sqrt(trace(R)+1);
  if abs(q(1))<1e-5
    [k,theta]=R2kth(R);
    q(2:4)=k;
  else
    q(2:4)=vee(R-R')/4/q(1);
  end
end

%
% hat.m (converting a vector into a skew-symmetric cross-product matrix
%
% khat = hat(k)
%

function khat = hat(k)
  
  khat=[0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];

end

%
% rot.m
%
% rotation matrix given axis and angle
%
function R=rot(k,theta)
  
  k=k/norm(k);
  R=eye(3,3)+sin(theta)*hat(k)+(1-cos(theta))*hat(k)*hat(k);

end

%
% vee
%
% The inverse function of hat
%   input: R = hat(k)
%   output: k = inverse_hat(R)
%

function k = vee(R)
    k(1,1) = R(3,2);
    k(2,1) = R(1,3);
    k(3,1) = R(2,1);
end