% dual_param_ex3.m
% parameters for Controllability - Observability tutorial
% MKS units
% Author: Brian Tremaine, TremaineConsultingGroup.com

% When output y is x1+x3 the system is not fully observable
% by having two separate outputs, x1, and x3, the system 
% states are fully observable
% states: x1= cart1 pos
%         x2= cart1 vel
%         x3= cart2 pos
%         x4= cart2 vel
clear

% first define continuous time system 
% large mass: M,  small mass: m

M = 10.0;
Kf2= 10.0;  % force constant
Kv2= 0.5;   % viscous damping
Ks2= 0.1;   % spring constant

m = 1.0;
Kf1= 2.0;   % force constant
Kv1= 0.1;   % viscous damping
Ks1= 0.1;   % spring constant

%% states of plant: x1=pos1, x2=vel1, x3=pos2, x4=vel2
A = [  0      1     0      0 ;
      -Ks1/m -Kv1/m 0      0;
       0      0     0      1 ;
       0      0    -Ks2/M -Kv2/M];
B = [0     0;
     Kf1/m 0
     0     0;
     0     Kf2/M];
C = [1 0 0 0;
     0 0 1 0];
D = [0 0; 0 0];

%% Augment plant with integral of x1==x4 and integral of x3==x5
Aaug = A;
Baug = B;
Caug = C;
Daug = D;


%% Convert to discrete time
Ts = 10E-3;
SYS= ss(Aaug,Baug,Caug,Daug);
%% matricies from sim model
SYSd = c2d(SYS,Ts);
Ad = SYSd.A;
Bd = SYSd.B;
Cd = SYSd.C;
Dd = SYSd.D;


    























%% compute CO and OB
CO = ctrb(Ad,Bd);
OB = obsv(Ad,Cd);
rankCO= rank(CO);
rankOB= rank(OB);
disp(["rank CO", rankCO]);
disp(["rank OB", rankOB]);


%% reference feedforward gains
[Nx, Nu, N1, n1, n2, n3, n4, n5, n6] = ffwd_ref(Ad,Bd,Cd);

%% lqr gains  could use any method ...
Q = 1;      % states
R = 1;      % control
[K,S, CLP]= dlqr(Ad,Bd,Q,R);

Kint= [0.1; 0.1]; % should augment int to plant and use LQR for both
                  % this value was trial and error
%Kint= [0.6; 0.1;];

%% closed loop w/ no feed-forward
SYS1 = ss(Ad-Bd*K,Bd,Cd,Dd);
% step response
dstep(Ad,Bd,Cd,Dd)

pzmap(SYS1); axis([0.95 1.05 -0.1 0.1])

%%
function [Nx, Nu, N1, n1, n2, n3, n4, n5, n6] = ffwd_ref(a1,b1,c1)
% calculate feedforward reference gains
% Refer Franklin/Powell/Workman 2nd Ed pg 276
% Digital Control of Dynamic Systems
% check:
[n1,n2] = size(a1) ; assert(n1==n2)                   % (n,n)
[n3,n4] = size(b1) ; assert(n1==n3)                   % (n,m)
[n5,n6] = size(c1) ; assert(n5==n4); assert(n6==n1);  % (r,n)
% define:
% xr = Nx * r,  (n,r)
% uss= Nu * r,  (m,r)
n = n1;
m = n4;
r = n5;
N1 = ([a1-eye(size(a1)) b1; c1 zeros(r,m)]) \ [zeros(n1,2); eye(2,2)]

%% Nx has dim nxr, Nu has dim mxr
Nx = N1(1:n,1:r);
Nu = N1(n+1:n+m,1:r);
end
% -------------------------------------
