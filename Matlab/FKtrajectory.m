% clear;
clc;
pos = zeros(3,51);
forces = zeros(5,51);
vel = zeros(3,51);

for i=0:50 
    % 时间,每个0.1s计算一次
    t=i/10;

    % 输入位置
    q = [
        0.4*sin(t)
        0.5*sin(2*t)
        0.4 + 0.2*sin(t)
        -0.65 + 0.2*sin(2*t)
        -0.4 + 0.2*sin(t)];
    % 速度
    dq = [
         0.4*cos(t)
         2*0.5*cos(2*t)
         0.2*cos(t)
         2*0.2*cos(2*t)
         0.2*cos(t)];
    % 加速度
    ddq = [
         -0.4*sin(t)
         -2*2*0.5*sin(2*t)
         -0.2*sin(t)
         -2*2*0.2*sin(2*t)
         -0.2*sin(t)];

    % 计算
    testFD;
    
    pos(:,i+1) = ee(1:3,4);
    forces(:,i+1) = abs(actuation_force);
    %forces(:,i+1) = n1(4:5);
    vel(:,i+1) = a3(4:6);
    %vel(:,i+1) = v9(1:3)+cross(v9(4:6),pm_j10(1:3,4));
    %vel(:,i+1) = a3(1:3)+cross(a3(4:6),pm_j3(1:3,4))+cross(v3(4:6),v3(1:3))+C3(v3(4:6))*C3(v3(4:6))*pm_j3(1:3,4);
end

t = 0:0.1:5;
% plot(t,j2pos(:,1),'LineWidth',1.2)
% hold on;
% plot(t,j2pos(:,2),'LineWidth',1.2)
% hold on;
% plot(t,j2pos(:,3),'LineWidth',1.2)
% hold on
% plot(t,pos(1,:))
% hold on;
% plot(t,pos(2,:))
% hold on;
% plot(t,pos(3,:))
% axis([0 5 -1 1])
% plot(t,vel(1,:))
% hold on;
% plot(t,vel(2,:))
% hold on;
% plot(t,vel(3,:))
% legend('x','y','z');
plot(t,acf,'k','LineWidth',1.2)
hold on
plot(t,forces(5,:),'--r','LineWidth',2),title('torque5')
% plot(t,forces(1,:))
% figure
% plot(t,forces(2,:))
% figure
% plot(t,forces(3,:))
% figure
% plot(t,forces(4,:))
% figure
% plot(t,forces(5,:))

% subplot(3,1,1),plot(t,lal(:,1),'k','LineWidth',1.2),hold on
% plot(t,vel(1,:),'--r','LineWidth',2),title('L3\_alphax')
% subplot(3,1,2),plot(t,lal(:,2),'k','LineWidth',1.2),hold on
% plot(t,vel(2,:),'--r','LineWidth',2),title('L3\_alphay')
% subplot(3,1,3),plot(t,lal(:,3),'k','LineWidth',1.2),hold on
% plot(t,vel(3,:),'--r','LineWidth',2),title('L3\_alphaz')
