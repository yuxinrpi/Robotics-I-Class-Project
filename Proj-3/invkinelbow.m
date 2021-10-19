function robot=invkinelbow(robot)
    T = robot.T;
    P = robot.P;
    ex = [1;0;0];
    ey = [0;1;0];
    ez = [0;0;1];
    
    %||P0T - P01 - R0T*P6T|| = ||P23 + R23P34||
    % d = || p2 - R(k,q)p1||
    % Solve for q3 (2 Solutions)
    d = norm(T(1:3, 4) - P(1:3, 1) - T(1:3, 1:3)*P(1:3, 7));
    p2 = P(1:3, 3);
    p1 = -P(1:3, 4);
    k = ey;
    q3 = subprob3(k,p1,p2,d);
    
    q = zeros(6,8);
    q(3, 1:4) = q3(1,1);
    q(3, 5:8) = q3(2,1);
    for i = 1:2
        %P0T - P01 - R0T*P6T = R01*R12(P23 + R23P34)
        %Rz(-q1)*(P16) = Ry(q2)*(P23 + R23P34)
        k1 = -ez;
        k2 = ey;
        p1 = T(1:3, 4) - P(1:3, 1) - T(1:3, 1:3)*P(1:3, 7);
        p2 = P(1:3, 3) + expm(q3(i,1)*hat(ey))*P(1:3, 4);
        %q1, q2 (2/1/0 sets of solutions)
        [q1, q2] = subprob2(k1, k2, p1, p2);
        q(:,4*i-3) = [q1(1,1);q2(1,1);q3(i,1);0;0;0]; % 1 and 5
        q(:,4*i-2) = q(:,4*i-3);
        q(:,4*i-1) = [q1(2,1);q2(2,1);q3(i,1);0;0;0]; % 3 and 7
        q(:,4*i  ) = q(:,4*i-1);  
    end

    for i = 1:4
        j = 2*i-1; %1 3 5 and 7
        k1 = -ex;
        k2 = ey;
        p1 =      expm(-(q(2,j)+q(3,j))*hat(ey));
        p1 = p1 * expm(-q(1,j)*hat(ez));
        p1 = p1 * T(1:3, 1:3)*ex;
        p2 = ex;
        [q4, q5] = subprob2(k1, k2, p1, p2);
        q(4,j  ) = q4(1,1);
        q(5,j  ) = q5(1,1);
        q(4,j+1) = q4(2,1);
        q(5,j+1) = q5(2,1);
    end
    
    
    for i = 1:length(q)
        %Solve for q6
        p2 =      expm(-q(5,i)          * hat(ey));
        p2 = p2 * expm(-q(4,i)          * hat(ex));
        p2 = p2 * expm(-(q(2,i)+q(3,i)) * hat(ey));
        p2 = p2 * expm(-q(1,i)          * hat(ez));
        p2 = p2 * T(1:3, 1:3) * ey;
        p1 = ey;
        k = ex;
        q6 = subprob1(k, p1, p2);
        q(6,i) = q6;
    end
    q;
    robot.q = q;
end

function M = hat(k)
    M = [0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
end
