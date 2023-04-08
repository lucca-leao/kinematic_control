%%
%Planejamento de trajetória poligonal e controle via feedback linearization
%AUTOR: LUCCA GARCIA LEÃO
%DATA: 10/02/2021
close all; 
clear all;
clc;

load('qhistory.mat');
load('goalHist.mat');
load('goalConfig.mat');
qTraj = qHistory;

mdl_ur5

%%
points =  [0.18407 -0.01936 0.6057 -8.7099 41.044 177.57; 0.28 0 0.6 -4.9747 -14.863 -89.879;...
    0.23 0.3 0.6 10.518 -14.314 -85.954; 0 0.4 0.6 19.45 1.3207 -86.252;...
    -0.25 0.4 0.6 18.44 16.288 -86.096; -0.3 0.2 0.6 -1.753 14.454 -85.899;...
    -0.2 0 0.6 -16.508 8.1022 -83.652];

q0 = [26.47 -60.27 -111.8 41.54 79.04 -28.09]*pi/180;

rep1 = repmat(points(2,:),50,1);
rep2 = repmat(points(3,:),50,1);
rep3 = repmat(points(4,:),50,1);
rep4 = repmat(points(5,:),50,1);
rep5 = repmat(points(6,:),50,1);
rep6 = repmat(points(7,:),50,1);

gols = [rep1;rep2;rep3;rep4;rep5;rep6];

t1 = linspace(0,5,50);
t2 = linspace(5,10,50);
t3 = linspace(10,15,50);
t4 = linspace(15,20,50);
t5 = linspace(20,25,50);
t6 = linspace(25,30,50);

t_J = [t1 t2 t3 t4 t5 t6];

[Q1,QD1,QDD1] = jtraj(q0,goalConfig(1,:),t1);
[Q2,QD2,QDD2] = jtraj(goalConfig(1,:),goalConfig(2,:),t1);
[Q3,QD3,QDD3] = jtraj(goalConfig(2,:),goalConfig(3,:),t1);
[Q4,QD4,QDD4] = jtraj(goalConfig(3,:),goalConfig(4,:),t1);
[Q5,QD5,QDD5] = jtraj(goalConfig(4,:),goalConfig(5,:),t1);
[Q6,QD6,QDD6] = jtraj(goalConfig(5,:),goalConfig(6,:),t1);

Q = [Q1;Q2;Q3;Q4;Q5;Q6];
QD = [QD1;QD2;QD3;QD4;QD5;QD6];

%%
sim = remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
sim.simxFinish(-1); % just in case, close all opened connections
clientID = sim.simxStart('127.0.0.1',19999,true,true,5000,5);
n = 6;
sim.simxStartSimulation(clientID, sim.simx_opmode_oneshot);
sim.simxSynchronous(clientID, true);
qdot = zeros(n,1);

erroJ = [];
erroOri = [];
if(clientID > -1)
   disp('Connected to remote API server');        
    [res,objs] = sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking);

    if (res == sim.simx_return_ok)
        fprintf('Number of objects in the scene: %d\n',length(objs));
    else
        fprintf('Remote API function call returned with error code: %d\n',res);
    end

    % Now send some data to CoppeliaSim in a non-blocking fashion:
    sim.simxAddStatusbarMessage(clientID,'Hello CoppeliaSim!',sim.simx_opmode_oneshot);

    [~, h(1)] = sim.simxGetObjectHandle (clientID, 'UR5_joint1', sim.simx_opmode_blocking);
    [~, h(2)] = sim.simxGetObjectHandle (clientID, 'UR5_joint2', sim.simx_opmode_blocking);
    [~, h(3)] = sim.simxGetObjectHandle (clientID, 'UR5_joint3', sim.simx_opmode_blocking);
    [~, h(4)] = sim.simxGetObjectHandle (clientID, 'UR5_joint4', sim.simx_opmode_blocking);
    [~, h(5)] = sim.simxGetObjectHandle (clientID, 'UR5_joint5', sim.simx_opmode_blocking);
    [~, h(6)] = sim.simxGetObjectHandle (clientID, 'UR5_joint6', sim.simx_opmode_blocking);
    [~, camHandle] = sim.simxGetObjectHandle(clientID, 'Vision_sensor', sim.simx_opmode_blocking);
    Kp = 1.0;
    n=6;
    for(i=1:length(Q))
        conf = Q(i,:);
        T_current = ur5.fkine(conf);
        trans = T_current.transl;
        gol = gols(i,:);
        err = sqrt((gol(1) - trans(1))^2 + (gol(2) - trans(2))^2 + (gol(3) - trans(3))^2);
        erroJ = [erroJ err];
        oricurrent = UnitQuaternion(T_current.R);
        origoal = UnitQuaternion.eul(gol(4:6),'deg');
        erro_o = origoal*oricurrent.inv();
        erroOri = [erroOri; erro_o.v];
        for j=1:n
            sim.simxSetJointTargetPosition(clientID,h(j), conf(j), sim.simx_opmode_oneshot);
            sim.simxAddStatusbarMessage(clientID,'Following trajectory!',sim.simx_opmode_oneshot);
        end
        sim.simxSynchronousTrigger(clientID);
    end
end

%%
%t = linspace(0,30,490);
%prepara dados para plotar
%t_J = t;
xl_J = []
yl_J = []
zl_J = []
x_goal = []
y_goal = []
z_goal = []
roll_J = []
pitch_J = []
yaw_J = []
%erroD = []
%goalHistory = goalHistory';
for k = 1:length(Q)
   pose = ur5.fkine(Q(k,:));
   %poseGoal = goalHistory(k,:);
   %transGoal = poseGoal(1:3);
   %x_goal = [x_goal transGoal(1)];
   %y_goal = [y_goal transGoal(2)];
   %z_goal = [z_goal transGoal(3)];
   %oriGoal = poseGoal(4:6);
   trans = pose.t;
   ori = pose.o;
   xl_J = [xl_J trans(1)];
   yl_J = [yl_J trans(2)];
   zl_J = [zl_J trans(3)];
   roll_J = [roll_J ori(1)];
   pitch_J = [pitch_J ori(2)];
   yaw_J = [yaw_J ori(3)];
   %err = sqrt((transGoal(1) - trans(1))^2 + (transGoal(2) - trans(2))^2 + (transGoal(3) - trans(3))^2);
   %erroD = [erroD err];
end

%% load moveL data 
moveL = load('moveL_data.mat');
figure
plot3(xl_J,yl_J,zl_J,'Color',[1 0 0],'LineWidth',2);
hold on
plot3(moveL.xl,moveL.yl,moveL.zl,'Color',[0 0 1],'LineWidth',2);
hold on
scatter3(points(2:7,1),points(2:7,2),points(2:7,3),'filled');
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
legend('J','L');
%% plota estados
figure
subplot(2,3,1);
plot(t_J,xl_J, moveL.t, moveL.xl,"LineWidth",1.5);
xline(5,'-.');
xline(10,'-.');
xline(15,'-.');
xline(20,'-.');
xline(25,'-.');
xline(30,'-.');
xlabel("Tempo [s]");
ylabel("Deslocamento [m]")
title("X");
legend('J','L');

subplot(2,3,2);
plot(t_J,yl_J,moveL.t, moveL.yl,"LineWidth",1.5);
xline(5,'-.');
xline(10,'-.');
xline(15,'-.');
xline(20,'-.');
xline(25,'-.');
xline(30,'-.');
xlabel("Tempo [s]");
ylabel("Deslocamento [m]")
title("Y");
legend('J','L');

subplot(2,3,3);
plot(t_J,zl_J,moveL.t, moveL.zl,"LineWidth",1.5);
xline(5,'-.');
xline(10,'-.');
xline(15,'-.');
xline(20,'-.');
xline(25,'-.');
xline(30,'-.');
xlabel("Tempo [s]");
ylabel("Deslocamento [m]")
title("Z");
legend('J','L');

subplot(2,3,4);
plot(t_J,roll_J*180/pi,moveL.t, moveL.roll*180/pi,"LineWidth",1.5);
xline(5,'-.');
xline(10,'-.');
xline(15,'-.');
xline(20,'-.');
xline(25,'-.');
xline(30,'-.');
xlabel("Tempo [s]");
ylabel("Ângulo [º]");
title("Roll");
legend('J','L');

subplot(2,3,5);
plot(t_J,pitch_J*180/pi,moveL.t, moveL.pitch*180/pi,"LineWidth",1.5);
xline(5,'-.');
xline(10,'-.');
xline(15,'-.');
xline(20,'-.');
xline(25,'-.');
xline(30,'-.');
title("Pitch");
xlabel("Tempo [s]");
ylabel("Ângulo [º]");
legend('J','L');

subplot(2,3,6);
plot(t_J,yaw_J*180/pi,moveL.t, moveL.yaw*180/pi,"LineWidth",1.5);
xline(5,'-.');
xline(10,'-.');
xline(15,'-.');
xline(20,'-.');
xline(25,'-.');
xline(30,'-.');
title("Yaw");
xlabel("Tempo [s]");
ylabel("Ângulo [º]");
legend('J','L');

%%
figure
subplot(2,2,1);
plot(t_J,erroJ, moveL.t, moveL.erroD,"LineWidth",1.5);
xline(5,'-.');
xline(10,'-.');
xline(15,'-.');
xline(20,'-.');
xline(25,'-.');
xline(30,'-.');
title("Erro posição");
xlabel("Tempo [s]");
ylabel("Erro [m]");
legend('J','L');

subplot(2,2,2);
plot(t_J,erroOri(:,1)*180/pi,moveL.t,moveL.erroOri(:,1)*180/pi,"LineWidth",1.5);
xline(5,'-.');
xline(10,'-.');
xline(15,'-.');
xline(20,'-.');
xline(25,'-.');
xline(30,'-.');
title("Erro roll");
xlabel("Tempo [s]");
ylabel("Erro [º]");
title("Erro roll");
legend('J','L');

subplot(2,2,3);
plot(t_J,erroOri(:,2)*180/pi,moveL.t,moveL.erroOri(:,2)*180/pi,"LineWidth",1.5);
xline(5,'-.');
xline(10,'-.');
xline(15,'-.');
xline(20,'-.');
xline(25,'-.');
xline(30,'-.');
xlabel("Tempo [s]");
ylabel("Erro [º]");
title("Erro pitch");
legend('J','L');

subplot(2,2,4);
plot(t_J,erroOri(:,3)*180/pi,moveL.t,moveL.erroOri(:,3)*180/pi,"LineWidth",1.5);
xline(5,'-.');
xline(10,'-.');
xline(15,'-.');
xline(20,'-.');
xline(25,'-.');
xline(30,'-.');
xlabel("Tempo [s]");
ylabel("Erro [º]");
title("Erro yaw");
legend('J','L');

%%
%plota sinal de controle
u1 = QD(:,1);
u2 = QD(:,2);
u3 = QD(:,3);
u4 = QD(:,4);
u5 = QD(:,5);
u6 = QD(:,6);

figure
subplot(2,3,1);
plot(t_J,u1,moveL.t,moveL.u1,"LineWidth",1.5);
xline(5,'-.');
xline(10,'-.');
xline(15,'-.');
xline(20,'-.');
xline(25,'-.');
xline(30,'-.');
xlabel("Tempo [s]");
ylabel("Velocidade angular [rad/s]");
title("u1");
legend('J','L');

subplot(2,3,2);
plot(t_J,u2,moveL.t,moveL.u1,"LineWidth",1.5);
xline(5,'-.');
xline(10,'-.');
xline(15,'-.');
xline(20,'-.');
xline(25,'-.');
xline(30,'-.');
xlabel("Tempo [s]");
ylabel("Velocidade angular [rad/s]");
title("u2");
legend('J','L');

subplot(2,3,3);
plot(t_J,u3,moveL.t,moveL.u1,"LineWidth",1.5);
xline(5,'-.');
xline(10,'-.');
xline(15,'-.');
xline(20,'-.');
xline(25,'-.');
xline(30,'-.');
xlabel("Tempo [s]");
ylabel("Velocidade angular [rad/s]");
title("u3");
legend('J','L');

subplot(2,3,4);
plot(t_J,u4,moveL.t,moveL.u1,"LineWidth",1.5);
xline(5,'-.');
xline(10,'-.');
xline(15,'-.');
xline(20,'-.');
xline(25,'-.');
xline(30,'-.');
xlabel("Tempo [s]");
ylabel("Velocidade angular [rad/s]");
title("u4");
legend('J','L');

subplot(2,3,5);
plot(t_J,u5,moveL.t,moveL.u1,"LineWidth",1.5);
xline(5,'-.');
xline(10,'-.');
xline(15,'-.');
xline(20,'-.');
xline(25,'-.');
xline(30,'-.');
xlabel("Tempo [s]");
ylabel("Velocidade angular [rad/s]");
title("u5");
legend('J','L');

subplot(2,3,6);
plot(t_J,u6,moveL.t,moveL.u1,"LineWidth",1.5);
xline(5,'-.');
xline(10,'-.');
xline(15,'-.');
xline(20,'-.');
xline(25,'-.');
xline(30,'-.');
xlabel("Tempo [s]");
ylabel("Velocidade angular [rad/s]");
title("u6");
legend('J','L');