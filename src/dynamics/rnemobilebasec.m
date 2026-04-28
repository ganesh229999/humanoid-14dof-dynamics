function [tau,tauB,M,MB,C,CB,G,GB,a_base_all] = rnemobilebasec(...
         th_A,th_B,thdot_A,thdot_B,thddot_A,thddot_B,baseState,objParams)
% RNEMOBILEBASEC  Dual-arm RNE dynamics on a mobile (floating) base platform.
%
% Computes joint torques for BOTH arms simultaneously while accounting for
% the dynamic coupling between the arms and the moving base platform.
% Base acceleration is solved from Newton-Euler coupling via Khalil-Dombre.
%
%   τ_arm = M·q̈ + C(q,q̇)·q̇ + G(q) + τ_ext   (for each arm)
%   Mbase·ä_base = −[Jrc_A·q̈_A + Jrc_B·q̈_B + β_rc]
%
% INPUTS:
%   th_A/B      – N×7  joint angles [rad]
%   thdot_A/B   – N×7  joint velocities [rad/s]
%   thddot_A/B  – N×7  joint accelerations [rad/s²]
%   baseState   – struct: .T_w0 (4×4), .v0 (3×1), .w0 (3×1)
%   objParams   – struct: .mass, .I, .g_obj
%
% OUTPUTS:
%   tau/tauB    – N×7  joint torques, Arm A / Arm B  [N·m]
%   M/MB        – 7×7×N inertia matrices
%   C/CB        – N×7  Coriolis/centrifugal torques
%   G/GB        – N×7  gravity torques
%   a_base_all  – 3×N  base acceleration history [ax; ay; α]
%
% REFERENCE: Khalil & Dombre (2002), Ch. 9.

if nargin<8, error('objParams must be provided.'); end
numLinks=8;
if size(th_A,1)==1, N_samples=1; else, N_samples=size(th_A,1)-1; end

tau  =zeros(N_samples,7); M_mat =zeros(7,7,N_samples);
G_vec=zeros(N_samples,7); C_vec =zeros(N_samples,7);
tauB =zeros(N_samples,7); M_matB=zeros(7,7,N_samples);
G_vecB=zeros(N_samples,7); C_vecB=zeros(N_samples,7);
a_base_all=zeros(3,N_samples);

%% External forces
ext_force_flag=input('External forces? (1=Yes, 0=No): ');
if ext_force_flag==1
    method=input('(1) Manual  (2) Excel: ');
    extEntries=struct('joint',{},'wrench',{});
    if method==1
        n=input('Number of entries: ');
        for idx=1:n
            extEntries(idx).joint=input(sprintf('Joint (1-8) #%d: ',idx));
            extEntries(idx).wrench=[input('fx=');input('fy=');input('fz=');
                                    input('mx=');input('my=');input('mz=')];
        end
    else
        ext=readmatrix(input('Excel file: ','s'));
        for idx=1:size(ext,1)
            extEntries(idx).joint=ext(idx,2);
            extEntries(idx).wrench=ext(idx,3:8)';
        end
    end
else
    extEntries=[]; disp('No external forces.');
end

%% Main loop
for i=1:N_samples
    th_A_i=th_A(i,:)'; thdot_A_i=thdot_A(i,:)'; thddot_A_i=thddot_A(i,:)';
    th_B_i=th_B(i,:)'; thdot_B_i=thdot_B(i,:)'; thddot_B_i=thddot_B(i,:)';

    v0=baseState.v0; w0=baseState.w0;
    robA=get_robot_paramsmobile('A',th_A_i,baseState.T_w0,v0,w0);
    robB=get_robot_paramsmobile('B',th_B_i,baseState.T_w0,v0,w0);

    a_base=compute_base_accel(th_A_i,thdot_A_i,thddot_A_i,...
                              th_B_i,thdot_B_i,thddot_B_i,baseState,extEntries);

    tauA_tot=compute_tau_mobile(thdot_A_i,thddot_A_i,robA,numLinks,objParams,a_base);
    tauB_tot=compute_tau_mobile(thdot_B_i,thddot_B_i,robB,numLinks,objParams,a_base);

    tau_gA=compute_tau_mobile(zeros(7,1),zeros(7,1),robA,numLinks,objParams,a_base);
    tau_gB=compute_tau_mobile(zeros(7,1),zeros(7,1),robB,numLinks,objParams,a_base);

    obj0=objParams; obj0.mass=0; obj0.I=zeros(3);
    MA_i=zeros(7,7); MB_i=zeros(7,7);
    for k=1:7
        e=zeros(7,1); e(k)=1;
        MA_i(:,k)=compute_tau_mobile(zeros(7,1),e,robA,numLinks,obj0,a_base)-tau_gA;
        MB_i(:,k)=compute_tau_mobile(zeros(7,1),e,robB,numLinks,obj0,a_base)-tau_gB;
    end
    MA_i=0.5*(MA_i+MA_i'); MB_i=0.5*(MB_i+MB_i');
    CA_i=tauA_tot-(MA_i*thddot_A_i+tau_gA);
    CB_i=tauB_tot-(MB_i*thddot_B_i+tau_gB);

    tau(i,:)=tauA_tot'; M_mat(:,:,i)=MA_i; G_vec(i,:)=tau_gA'; C_vec(i,:)=CA_i';
    tauB(i,:)=tauB_tot'; M_matB(:,:,i)=MB_i; G_vecB(i,:)=tau_gB'; C_vecB(i,:)=CB_i';
    a_base_all(:,i)=a_base;
end
M=M_mat; C=C_vec; G=G_vec;
MB=M_matB; CB=C_vecB; GB=G_vecB;
end

function tau_s=compute_tau_mobile(thdot_i,thddot_i,robot,numLinks,objParams,a_base)
thdot_f=[0;thdot_i]; thddot_f=[0;thddot_i];
A=robot.A; Ri=robot.Ri; Pc=robot.Pc; I=robot.I; m=robot.m; g0=robot.g0;
Z=[0;0;1];
w=cell(1,numLinks+1); aa=cell(1,numLinks+1); al=cell(1,numLinks+1); ac=cell(1,numLinks);
al_base=[a_base(1);a_base(2);0]-[g0;0;0]+robot.v0;
aa_base=[0;0;a_base(3)]+robot.w0;
w{1}=robot.w0; aa{1}=aa_base; al{1}=al_base;
for j=1:numLinks
    R=Ri{j}';
    w{j+1}=R*(w{j}+Z*thdot_f(j));
    aa{j+1}=R*(aa{j}+cross(w{j},Z*thdot_f(j))+Z*thddot_f(j));
    p=A{j}(1:3,4);
    al{j+1}=R*(al{j}+cross(aa{j},p)+cross(w{j},cross(w{j},p)));
    ac{j}=al{j+1}+cross(aa{j+1},Pc(:,j))+cross(w{j+1},cross(w{j+1},Pc(:,j)));
end
Fl=cell(1,numLinks); Nl=cell(1,numLinks);
for j=1:numLinks, Fl{j}=m(j)*ac{j}; Nl{j}=I{j}*aa{j+1}+cross(w{j+1},I{j}*w{j+1}); end
f=cell(1,numLinks+1); n=cell(1,numLinks+1);
for k=1:numLinks+1, f{k}=zeros(3,1); n{k}=zeros(3,1); end
f{numLinks+1}=zeros(3,1); n{numLinks+1}=zeros(3,1);  % EE wrench zeroed for mobile
for j=numLinks:-1:1
    if j<numLinks
        pn=A{j+1}(1:3,4); fn=Ri{j+1}*f{j+1}; nn=Ri{j+1}*n{j+1};
    else
        pn=zeros(3,1); fn=f{numLinks+1}; nn=n{numLinks+1};
    end
    f{j}=fn+Fl{j};
    n{j}=Nl{j}+nn+cross(Pc(:,j),Fl{j})+cross(pn,fn);
end
tau_s=zeros(7,1);
for j=1:7, tau_s(j)=n{j}'*Z; end
end
