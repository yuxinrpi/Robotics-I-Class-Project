clc; clear; close all;

% load MATLAB ABB IRB120
abb=loadrobot('abbIrb120','DataFormat','column');
abb.Gravity=9.81*[0 0 -1];

gravityCompensation = 1
fv=[.1;.1;.1;.05;.05;.05];
fc=[2;2;2;.1;.1;.1];

qinit=zeros(6,1);
qdotinit=zeros(6,1);
fext=zeros(6,8);

if gravityCompensation
    %kp=5*[1.5;1.5;1.5;.1;.1;.1];kd=[2.5;2.5;1;5;5;5];ki=[.1;.1;.1;.1;.1;.1];
    kp=[5;5;5;.1;.3;.1];kd=[3;3;3;1;.5;.5];ki=[.1;.5;.5;.05;.15;.05];
else
    kp = [  20;  300;  450; 3.55;     5;  .75];
    ki = [  65;   20;  300;   .5;   0.5;   .1]; 
    kd = [  20;   10;   10;  0.5;   0.5; 0.05];
end
%kp=zeros(size(kp));kd=zeros(size(kd));ki=zeros(size(ki));

% for gravity compensation
x=[-17.0122   -3.0550   -5.6141    0.0007   -0.0846   -3.0546   -5.6091 ...
    0.0796   -0.0812    0.0848    0.0814   -0.0814]';

proj3;
Tf=length(q(:,:,ksol));
t_table=(0:.1:Tf);
for i=1:6
    qdes(:,i) = repelem(q(i,:,ksol),ones(1, Tf)*10);
end
qdes(end+1,:) = qdes(end,:);
q_table=qdes;
simout=sim('robotwithqdes','StartTime','0','StopTime',num2str(Tf),...
    'FixedStep','0.01','ReturnWorkspaceOutputs', 'on');

n=length(simout.tout);
%figure(1);for i=1:300;show(abb,simout.q(i,:)');end

figure(10);plot(simout.tout,simout.q,...
    simout.tout,squeeze(simout.qdes),':','LineWidth',2);
xlabel('time (s)')
ylabel('joint angles (rad)')
title(['Actual and Commanded Joint Angles ']);
legend('q_1','q_2','q_3','q_4','q_5','q_6',...
    'q_{d_1}','q_{d_2}','q_{d_3}','q_{d_4}','q_{d_5}','q_{d_6}');

%%
figure(20);

xlabel('x');ylabel('y');zlabel('z');
title("IRB1200 Tracking S Sphere Curve(Yuxin Hu)");

axis(r*[-1 1 -1 1 0 2]);axis('square');
view(120,10);
grid();
n=1011/10;k=fix(size(simout.q,1)/n);
%show(abb,simout.q(1,:)','Visuals','Off');pause(.1)
hold on
j=1;
for i=1:k:size(simout.q,1)
    %show(abb,simout.q(i,:)','Visuals','Off');pause(.1);
    arm_movie(j)=getframe;
    j=j+1;
end
hold off
movie(arm_movie);

v=VideoWriter("Track.avi");
open(v);
writeVideo(v,arm_movie);
close(v);