clear all;close all;

abb=loadrobot('abbIrb120','DataFormat','column');
abb.Gravity=9.81*[0 0 -1];

% initial configuration
qinit=zeros(6,1);
qdotinit=zeros(6,1);
fext=zeros(6,8);

% initial pose to start compliance control
Tdes=[eye(3,3) [0.5;0;0.5];0 0 0 1];
% feedback gains
kp=[5;5;5;5;5;5];kd=[3;3;3;5;5;5];ki=[.1;.5;.2;.1;.1;.1];
% where the wall is
xwall=0.5;
% generalized damper 
Bdes=20;
% desired force 
fdes=2.5;
% spring constant
kspring=20;
% for gravity compensation
x=[-17.0122   -3.0550   -5.6141    0.0007   -0.0846   -3.0546   -5.6091 ...
    0.0796   -0.0812    0.0848    0.0814   -0.0814]';
% run the simulink
simout=sim('robotwithwall1','StartTime','0','StopTime','30',...
    'FixedStep','0.005','ReturnWorkspaceOutputs', 'on');

p0T=simout.X(1:3,4,:);
p0T=squeeze(p0T);

n=length(simout.tout);

figure(10);plot(simout.tout,simout.q,...
    simout.tout,squeeze(simout.qdes),':','LineWidth',2);
xlabel('time (s)')
ylabel('joint angles (rad)')
title('Actual and Commanded Joint Angles');
legend('q_1','q_2','q_3','q_4','q_5','q_6',...
    'q_{d_1}','q_{d_2}','q_{d_3}','q_{d_4}','q_{d_5}','q_{d_6}');

figure(11);plot(simout.tout,p0T,'LineWidth',2);
xlabel('time (s)')
ylabel('position (m)')
title('End Effector Position')
legend('x0T','y0T','z0T');

figure(40);plot(simout.tout,simout.fspring,'LineWidth',2);
grid;
xlabel('time (s)')
ylabel('force (N)')
title('Contact Spring Force')

% figure(20);n=50;k=fix(size(simout.q,1)/n);
% j=1;
% for i=1:k:size(simout.q,1)
%     show(abb,simout.q(i,:)','Visuals','Off');pause(.1);
%     arm_movie(j)=getframe;
%     j=j+1;
% end
% %hold off
% movie(arm_movie);
% 
% v=VideoWriter('freefall',"MPEG-4");
% open(v);
% writeVideo(v,arm_movie);
% close(v);