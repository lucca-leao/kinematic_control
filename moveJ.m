%%
%Planejamento de trajetória no espaço das juntas (moveJ)
%AUTOR: LUCCA GARCIA LEÃO
%DATA: 10/02/2021
close all; 
clear all;
clc;


mdl_KR5

points = [-0.25 -1 0.25; 0.0 -0.75 0.5; 0.25 -0.5 0.2; 0.4 -0.7 -0.3; 0.5 -0.4 -0.4];

%%
q0 = [-1.9450   -0.8777    3.5072   -0.5569   -1.8519    0.4537]
goals = [-1.5680   -1.1641    2.7193   -0.0002   -0.7421    1.5676;...
         -1.1062   -1.9128    2.5842   -0.0000   -1.3554    1.1062;...
         -1.0516   -1.5148    3.7394   -0.0000   -2.1126    1.0516;...
         -0.6741   -1.7107    3.7261   -0.0000   -2.2952    0.6741];

t1 = linspace(0,10,50);
t2 = linspace(10,20,50);
t3 = linspace(20,30,50);
t4 = linspace(30,40,50);

t_J = [t1 t2 t3 t4];

[Q1,QD1,QDD1] = jtraj(q0,goals(1,:),t1);
[Q2,QD2,QDD2] = jtraj(goals(1,:),goals(2,:),t1);
[Q3,QD3,QDD3] = jtraj(goals(2,:),goals(3,:),t1);
[Q4,QD4,QDD4] = jtraj(goals(3,:),goals(4,:),t1);

Q = [Q1;Q2;Q3;Q4];
QD = [QD1;QD2;QD3;QD4];

%% Posição inicial
plot(KR5,q0)
hold on
scatter3(points(:,1),points(:,2),points(:,3),'filled')
%% Animação
path = KR5.fkine(Q).transl;
x = path(:,1)';
y = path(:,2)';
z = path(:,3)';
plot3(x,y,z,'Color',[1 0 0],'LineWidth',2);
plot(KR5,Q)

%% plota sinais
qplot(Q)
qplot(QD)