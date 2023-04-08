%%
%Controle cinemático de um manipulador robótico de 6 juntas rotativas
%AUTOR: LUCCA GARCIA LEÃO
%DATA: 10/02/2021
close all; 
clear all;
clc;

mdl_KR5

r = 0.4;
x_center = 0.0;
z_center = 0.0;
wn = pi/10;

p0 = [0 -1 0 0 0 0];
pf = [0.5 -0.7 -0.5 0 0 0];
t_f = 40;

syms t
%círculo
pds(t) = [x_center+r*cos(wn*t) -1 z_center+r*sin(wn*t)];
pddot(t) = diff(pds);

%linha
%pds(t) = [p0(1)+(pf(1)-p0(1))*t/t_f p0(2)+(pf(2)-p0(2))*t/t_f p0(3)+(pf(3)-p0(3))*t/t_f];
%pddot(t) = diff(pds);

%posição inicial e desejada

x_0 = x_center + r * cos(0);
z_0 = z_center + r * sin(0);

Ro = rpy2r(45,15,100,'deg');
R = rpy2r(0, 0, 180, 'deg');
td = [x_center -1 z_center];
qd = UnitQuaternion(R);
rt = rt2tr(Ro,td);

q0 = KR5.ikunc(rt);
plot(KR5,q0)
pause
hold on
trplot(rt)

%%

T_sample = 0.1;
Kt = 0.8;
Ko = 0.8;

xHistory = rt;
mvHistory = [];
errorHistory = [];
goalHistory = [];
qHistory = q0;

q = q0;

i = 0;

T_current = KR5.fkine(q0);
x_current = [x_center -1 z_center 0 0 1];

cont0 = tic;
t0 = tic;
contf = toc(cont0);

t = [];
erroOri = [];
while(contf < 40)
   T_current = KR5.fkine(q);
   
   trans_T = T_current.t;
   x_current = [trans_T' tr2rpy(T_current.R)];
   
   pd = double(pds(contf)); %Posição desejada numérica
   pddots = [double(pddot(contf)) 0 0 0]; %Ação de feedfoward

   goalHistory = [goalHistory; pd 0 0 0];
   
   Qe = UnitQuaternion(T_current.R);
   
   erro_t = pd(1:3) - x_current(1:3);
   erro_o = qd*Qe.inv;
   J = KR5.jacob0(q);
   
   qdot_t = pddots(1:3) + Kt*erro_t;
   qdot_o = pddots(4:6) + Ko*erro_o.v;
   
   %qdot_t = pddots(1:3);
   %qdot_o = pddots(4:6);
   %qdot_t = Kt*erro_t;
   %qdot_o = Ko*erro_o.v;
   
   qdot = inv(J)*[qdot_t qdot_o]'
   contf = toc(cont0);
   tf = toc(t0);
   t0 = tic;
    
   tf_robo = @(t,qi) [qdot(1);qdot(2);qdot(3);qdot(4);qdot(5);qdot(6)];
   [~, qi] = ode45(tf_robo, 0:tf:tf, zeros(1,6));
   q = q + qi(end,:);
   qHistory = [qHistory; q];
   mvHistory = [mvHistory qdot];
   erroOri = [erroOri ; erro_o.v];
   t = [t contf];
   pause(0.1);
end

%%
%prepara dados para plotar
xl = []
yl = []
zl = []
x_goal = []
y_goal = []
z_goal = []
roll = []
pitch = []
yaw = []
erroD = []
%goalHistory = goalHistory';
for k = 1:length(qHistory)-1
   pose = KR5.fkine(qHistory(k,:));
   poseGoal = goalHistory(k,:);
   transGoal = poseGoal(1:3);
   x_goal = [x_goal transGoal(1)];
   y_goal = [y_goal transGoal(2)];
   z_goal = [z_goal transGoal(3)];
   oriGoal = poseGoal(4:6);
   trans = pose.t;
   ori = pose.o;
   xl = [xl trans(1)];
   yl = [yl trans(2)];
   zl = [zl trans(3)];
   roll = [roll ori(1)];
   pitch = [pitch ori(2)];
   yaw = [yaw ori(3)];
   err = sqrt((transGoal(1) - trans(1))^2 + (transGoal(2) - trans(2))^2 + (transGoal(3) - trans(3))^2);
   erroD = [erroD err];
end

%% Animação
pause
y_dHistory = -1*ones([1,96]);
Qplot = qHistory;
plot3(goalHistory(:,1),goalHistory(:,2),goalHistory(:,3),'Color',[0 0 1],'LineWidth',2);
plot(KR5,Qplot)
hold on
plot3(xl,yl,zl,'Color',[1 0 0],'LineWidth',2);
hold on

%%
%plota estados
figure
subplot(2,3,1);
plot(t,xl,"LineWidth",1.5);
title("x");

subplot(2,3,2);
plot(t,yl,"LineWidth",1.5);
title("y");

subplot(2,3,3);
plot(t,zl,"LineWidth",1.5);
title("z");

subplot(2,3,4);
plot(t,roll,"LineWidth",1.5);
title("roll");

subplot(2,3,5);
plot(t,pitch,"LineWidth",1.5);
title("pitch");

subplot(2,3,6);
plot(t,yaw,"LineWidth",1.5);
title("yaw");

%%
figure
subplot(2,2,1);
plot(t,erroD,"LineWidth",1.5);
title("erro");

subplot(2,2,2);
plot(t,erroOri(:,1),"LineWidth",1.5);
title("Erro roll");

subplot(2,2,3);
plot(t,erroOri(:,2),"LineWidth",1.5);
title("Erro pitch");

subplot(2,2,4);
plot(t,erroOri(:,3),"LineWidth",1.5);
title("Erro yaw");

%%
%plota sinal de controle
u1 = mvHistory(1,:);
u2 = mvHistory(2,:);
u3 = mvHistory(3,:);
u4 = mvHistory(4,:);
u5 = mvHistory(5,:);
u6 = mvHistory(6,:);

figure
subplot(2,3,1);
plot(t,u1,"LineWidth",1.5);
title("u1");

subplot(2,3,2);
plot(t,u2,"LineWidth",1.5);
title("u2");

subplot(2,3,3);
plot(t,u3,"LineWidth",1.5);
title("u3");

subplot(2,3,4);
plot(t,u4,"LineWidth",1.5);
title("u4");

subplot(2,3,5);
plot(t,u5,"LineWidth",1.5);
title("u5");

subplot(2,3,6);
plot(t,u6,"LineWidth",1.5);
title("u6");
