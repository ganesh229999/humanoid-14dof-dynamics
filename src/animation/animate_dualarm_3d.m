% ANIMATE_DUALARM_3D  Real-time 3D animation of the dual-arm mobile-base robot.
%
% Loads trajectory data from Excel, integrates base kinematics using
% semi-implicit Euler, and renders the animated robot with end-effector traces.
%
% ───────────────────────────────────────────────────────────────────────────
% USAGE:
%   Run this script after executing main_rne_mobilebase.m.
%   Ensure 'Testoctlifting.xlsx' (or your filename) is on the MATLAB path.
%
% CONTROLS:
%   slowdown_factor – increase to slow animation; 1 = real-time at dt
% ───────────────────────────────────────────────────────────────────────────
clc; clear; close all;

%% Load data
fname = 'Testoctlifting.xlsx';
qA_hist       = readmatrix(fname,'Sheet','ArmA_q');
qB_hist       = readmatrix(fname,'Sheet','ArmB_q');
xObj_hist     = readmatrix(fname,'Sheet','ObjectPc');
baseAcc_hist  = readmatrix(fname,'Sheet','BaseAcc')';

if size(qA_hist,1)~=7,      qA_hist=qA_hist';      end
if size(qB_hist,1)~=7,      qB_hist=qB_hist';      end
if size(xObj_hist,1)<3,     xObj_hist=xObj_hist';  end
if size(baseAcc_hist,1)<3,  baseAcc_hist=baseAcc_hist'; end

dt = 0.001;
N  = size(qA_hist,2);

%% Integrate base kinematics (semi-implicit Euler)
basePos    = zeros(3,N-1);
baseVel    = zeros(3,N-1);
baseOrient = zeros(3,N-1);

for i = 2:N-1
    a_base = baseAcc_hist(1:3,i-1);
    th = baseOrient(3,i-1);
    Rz = [cos(th),-sin(th),0; sin(th),cos(th),0; 0,0,1];
    a_world = Rz*a_base;
    baseVel(:,i)    = baseVel(:,i-1)    + a_world*dt;
    basePos(:,i)    = basePos(:,i-1)    + baseVel(:,i)*dt;
    baseOrient(3,i) = baseOrient(3,i-1);
end

reach = 1.5;
xl = [min(basePos(1,:))-reach, max(basePos(1,:))+reach];
yl = [min(basePos(2,:))-reach, max(basePos(2,:))+reach];

%% Figure
figure('Color','k','Name','Dual-Arm Humanoid Simulation','NumberTitle','off');
ax=axes; grid(ax,'on'); hold(ax,'on');
ax.Color='k'; ax.GridColor=[0.3 0.3 0.3];
ax.XColor='w'; ax.YColor='w'; ax.ZColor='w';
xlabel('X [m]','Color','w'); ylabel('Y [m]','Color','w'); zlabel('Z [m]','Color','w');
title('14-DOF Dual-Arm Humanoid | Mobile Base Simulation','Color','w','FontSize',13);
axis equal; xlim(xl); ylim(yl); zlim([-0.5,0.5]); view(45,25);

hA   = plot3(nan,nan,nan,'r.-','MarkerSize',14,'LineWidth',2.2);
hB   = plot3(nan,nan,nan,'c.-','MarkerSize',14,'LineWidth',2.2);
hBas = plot3(nan,nan,nan,'ws','MarkerSize',10,'MarkerFaceColor','w');
hObj = plot3(nan,nan,nan,'go','MarkerSize',9,'MarkerFaceColor','g');
trA=nan(3,N); trB=nan(3,N); trO=nan(3,N);

slowdown_factor = 4;
for i = 1:N-1
    bx=basePos(1,i); by=basePos(2,i); th=baseOrient(3,i);
    Rz=[cos(th),-sin(th),0; sin(th),cos(th),0; 0,0,1];

    % Arm A FK
    rA=get_robot_params('A',qA_hist(:,i));
    pA=zeros(3,8); for j=1:8, pA(:,j)=rA.T{j+1}(1:3,4); end
    pA=Rz*pA+basePos(1:3,i);

    % Arm B FK
    rB=get_robot_params('B',qB_hist(:,i));
    pB=zeros(3,8); for j=1:8, pB(:,j)=rB.T{j+1}(1:3,4); end
    pB=Rz*pB+basePos(1:3,i);

    objW=Rz*xObj_hist(1:3,i)+basePos(1:3,i);
    trA(:,i)=pA(:,end); trB(:,i)=pB(:,end); trO(:,i)=objW;

    set(hA,'XData',pA(1,:),'YData',pA(2,:),'ZData',pA(3,:));
    set(hB,'XData',pB(1,:),'YData',pB(2,:),'ZData',pB(3,:));
    set(hBas,'XData',bx,'YData',by,'ZData',0);
    set(hObj,'XData',objW(1),'YData',objW(2),'ZData',objW(3));
    drawnow; pause(dt*slowdown_factor);
end

% Final traces
plot3(trA(1,:),trA(2,:),trA(3,:),'r--','LineWidth',1.5);
plot3(trB(1,:),trB(2,:),trB(3,:),'c--','LineWidth',1.5);
plot3(trO(1,:),trO(2,:),trO(3,:),'g--','LineWidth',1.5);
legend({'Arm A','Arm B','Base','Object','Trace A','Trace B','Object Trace'},...
       'TextColor','w','Color','k','Location','best');
