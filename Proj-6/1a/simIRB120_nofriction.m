clc;close all;clear;

abb=loadrobot('abbIrb120','DataFormat','column');
abb.Gravity = 9.81*[0;0;-1];


qinit=zeros(6,1);
qdotinit=zeros(6,1);
fext=zeros(6,8);

fv = [.1,.1,.1,.05,.05,.05];
fc = [2,2,2,.1,.1,.1];

%kp=[1.5;1.5;1.5;.1;.1;.1]*10;kd=[2.5;2.5;1;5;5;5];ki=[.1;.1;.1;0;0;0];
kp = [  20;  150;  450; 3.55;     5;  .75];
ki = [  65;  100;  300;   .5;   0.5;   .1]; 
kd = [  20;   20;   10;  0.5;   0.5; 0.05];

Trials = 3
showMotion = 0;
j=1;
while j<=Trials
    %qdes=[0;0;0;0;0;0];
    %qdes = ones(6,1)*pi;
    qdes=(rand(6,1)-.5)*2*pi;
    simout=sim('robotsimulation_nofriction.slx','ReturnWorkspaceOutputs', 'on');
    
    if showMotion
        %Set up the Animation Recording
        v = VideoWriter('motion.avi');
        v.FrameRate = 10;
        open(v);
        
        n=length(simout.tout);
        interval = floor(n/50);
        figure(1);
        for i=1:interval:n
            show(abb,simout.q(i,:)');
            frame = getframe; 
            writeVideo(v,frame);
        end
        close(v);
    end
    
    
    
    figure(10);plot(simout.tout,simout.q,'LineWidth',2);
    legend('q_1','q_2','q_3','q_4','q_5','q_6');
    xlim([0,2])
    title("The Result Angular Displacement - With Friction (Yuxin Hu)");
    xlabel("Time (s)");
    ylabel("Angular Displacement (rad)");
    grid();
    
    for i = 1:6
        ss = simout.q(end,i) - qdes(i);
        if qdes(i) > qinit(i)
            maxq = max(simout.q(:,i));
            OS = (maxq - qinit(i))/(simout.q(end,i) - qinit(i))*100 - 100;
        else
            minq = min(simout.q(:,i));
            OS = (minq - qinit(i))/(simout.q(end,i) - qinit(i))*100 - 100;
        end
        SSpercent = ss/(qdes(i) - qinit(i))*100;
        disp("Joint "+i+" Target "+ qdes(i) +", %OS = "+OS+", SS error = "+ss+", "+SSpercent+"%")
        RobotOS(i) = OS;
        RobotSS(i) = ss;
        RobotSSpercent(i) = SSpercent;
    end
    if max(RobotOS) >= 30 || max(RobotSS) >= 0.1
        continue
    end
    
    avgOS(j,:) = RobotOS;
    avgSSpercent(j,:) = RobotSSpercent;
    avgSS(j,:) = RobotOS;
    j = j + 1;
    disp(" ")
end

avgOS
avgSS
avgSSpercent
%figure(2);plot(simout.tout,simout.tau,'LineWidth',2);
%legend('tau_1','tau_2','tau_3','tau_4','tau_5','tau_6');
%xlim([0,5])