%% Optimal Derivative Feedback Control Design Using Online Value Iteration
clear all 
clc 
close all

T_smpl_sim=0.001;         %simulation sampling time
step_data_reduction=45;   % Data reduction  best is 10
simulation_time=15;       % simulation time in seconds

%% Magnetic Levitation system dynamics:
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % UCF
CtoA = 3.4/32767;

% Actuator constants
a = 1.65 / CtoA;
b = 6.2  / 100;
c = 2.69 * CtoA;
d = 4.2/ 100;
n = 4;

% Magnet weight and mass
mg = .46;  %% wight of each disks
m = mg/9.81;

% Magnet damping
c1 = 0.01;
c2 = 0.01;

% Linearlization point
y10 =  0.02;
y20 = -0.02;
yc = .14; 
y120 = yc - y10 + y20;

% Linearization input
u10= a*(y10+b)^n*(c/(y120+d)^n+mg);
u20= a*(-y20+b)^n*(-c/(y120+d)^n+mg);

% Linearized spring constants
k1 = n*u10/a/(y10+b)^(n+1);
k12 = n*c/(y120+d)^(n+1);
k2 = n*u20/a/(-y20+b)^(n+1);
ku11 = 1/a/(y10+b)^n;
ku22 = 1/a/(-y20+b)^n;

% Linearized state-space matrices
A = [         0,     1,          0,     0;
    -(k1+k12)/m, -c1/m,      k12/m,     0;
              0,     0,          0,     1;
          k12/m,     0, (k2-k12)/m, -c2/m];

B = [0 0; ku11/m 0; 0 0; 0 ku22/m];
C = [1 0 0 0;0 0 1 0];
[n,m]=size(B);

x0=randi([-50 50],[n,1])/1000;
% Quadratic cost function parameters
Q=eye(n);
R=eye(m);

%% Optimal derivative feedback controller for comparison
[P_opt,K_x,~] = icare(inv(A),inv(A)*B,Q,R);
K_opt=-K_x;%Take negative to convert to derivative feedback controller

%% stable derivative feedback controller gain for training
poles=[-0.1;
       -0.2;
       -0.3;
       -0.4];
K_pp = -place(inv(A), inv(A)*B, poles); % Using pole placement method to find the gains.

K=K_pp;     %Training and initialize gain 


%%% Repeated data collection 
data_collection_iteration=1;
trash_hold=0.001;
while true
    %% exp make theta not be full rank freq  amplitude is not should be so difference 
    stopTime=2;%simulation stop time of each iteration of learning 
    t_exp=0:T_smpl_sim:stopTime;
    exp_signal=zeros(m,length(t_exp));
    
    lower_bound=-100;
    upper_bound=100;
    amplitude=[0.03,0.02];
    freq=100;
    
    for j=1:m
        for i=1:freq
              exp_signal(j,:)=exp_signal(j,:)+sin(randi([lower_bound,upper_bound],1)*t_exp);
        end
    end
    
    for index=1:m
    exp_signal(index,:)=amplitude(index)*exp_signal(index,:)./max(exp_signal(index,:)')';
    end
    
    u_exp=timeseries(exp_signal,t_exp);% exploratory signal
    u_exp.Data=permute(u_exp.Data,[3 1 2]);

%% Collecting data with simulink model 

  
    stopTime=1;%simulation stop time of each iteration of learning
    mdl='Simulink_model_convergance';
    load_system(mdl);
    cs = getActiveConfigSet(mdl);
    mdl_cs = cs.copy;
    set_param(mdl_cs,'StartTime','0','StopTime',num2str(stopTime),'SolverType','Fixed-Step',...
             'SolverName','FixedStepAuto','FixedStep',num2str(T_smpl_sim));
    
    x=[];%system states
    t=[];%time
    u=[];%input
    err_P=[];% value matrix error (trained vs optimal)
    err_K=[];% controller error (trained vs optimal)
    
    % numerically determine the integrals I_{xx} and I_{xu} eq(58 and 59)
    % save the values inside the integral and then numerically calculate the
    % integral
    % initial conditions of kronecker products
    
    xdot_u0=0;%I_{xx}, eq(58) in paper (actually derivative of I_{xx})
    xx_dot0=0;%I_{xu}, eq(59) in paper (actually derivative of I_{xu})
    i=1;
    P=zeros(n);%initial value matrix estimate
    t0=0;
    
    % simulate the system
    out = sim(mdl,mdl_cs);
    
    simSmpl=size(out.x,1);
    t_sim=out.x(1:simSmpl,1);%iteration time
    x_sim=out.x(1:simSmpl,2:n+1);%iteration states
    u_sim=out.u(1:simSmpl,2);%iteration input
    x_dot=out.x_dot(1:simSmpl,2:n);%iteration state derivatives
    xx_dot=out.xx_dot(1:simSmpl,2:n*n+1);%iteration kronecked product I_{xx}
    xdot_u=out.xdot_u(1:simSmpl,2:n*m+1);%iteration kronecked product I_{xu}
    % system states, time, and inputs for the total simulated time
    x=[x;x_sim(1:simSmpl-1,:)]; % exclude last sample to avoid overlapping of saved data
    t=[t;t0+t_sim(1:simSmpl-1,:)];
    u=[u;u_sim(1:simSmpl-1,:)];
    % downsampling to T=0.05 sec
    % relation between this T and system eigenvalues found in vrabie 2009
    % Automatica page 5. "Adaptive optimal control for continuous-time
    % linear systems based on policy iteration "
    x_iter=x_sim(1:step_data_reduction:size(x_sim,1),:);
    xx_dot=xx_dot(1:step_data_reduction:size(xx_dot,1),:);
    xdot_u=xdot_u(1:step_data_reduction:size(xdot_u,1),:);
    
    smp_itr=size(x_iter,1);
    x0_bar=[x_basis(x_iter(1:smp_itr-1,:)) -2*x_iter(1:smp_itr-1,:)];
    x1_bar=[x_basis(x_iter(2:smp_itr,:)) -2*x_iter(2:smp_itr,:)];
    X_diff=x1_bar-x0_bar;%eq(64 and 65)
    Ixx=xx_dot(2:smp_itr,:)-xx_dot(1:smp_itr-1,:);%eq(67)
    Ixu=xdot_u(2:smp_itr,:)-xdot_u(1:smp_itr-1,:);%eq(68)
        
    while 1
        Q_bar=Q+K'*R*K;
        Y=-Ixx*Q_bar(:);%eq(66)
        Theta=[X_diff -2*Ixx*kron(K'*R,eye(n))-2*Ixu*kron(R,eye(n))];%X_i in eq(62)
        is_full_rank = rank(Theta) == min(size(Theta));
        if ~is_full_rank
            display("Theta is not full rank")
        end
        p=pinv(Theta)*Y;%eq(69)
        P=P_mat(p(1:n*n));%extract value matrix P from p
        %extract controller gain K from p
        K=p(n*n+n+1:n*n+n+n*m)';%need to convert from vector to matrix in case of mimo system
        K=reshape(K,n,m)';
        err_P=[err_P norm(P-P_opt,'fro')];
        err_K=[err_K norm(K-K_opt,'fro')];
        if i>=2
            if abs(err_P(i)-err_P(i-1))<0.0001 || i>50
                break
            end
        end
        i=i+1;
    end
    x0=randi([-50 50],[n,1])/1000;
    % Once training is complete simulate the system with the trained controller
    for i=1:1
        % simulate the system
        exp_signal=zeros(size(t_exp));
        u_exp=timeseries(exp_signal,t_exp);
        set_param(mdl_cs,'StopTime',num2str(simulation_time))
        out = sim(mdl,mdl_cs);
        
        simSmpl=size(out.x,1);
        t_sim=out.x(1:simSmpl,1);
        x_sim=out.x(1:simSmpl,2:n+1);
        u_sim=out.u(1:simSmpl,2:end);
        x_dot=out.x_dot(1:simSmpl,2:n);
        xx_dot=out.xx_dot(1:simSmpl,2:n*n+1);
        xdot_u=out.xdot_u(1:simSmpl,2:n*m+1);
        x=[x_sim(1:simSmpl-1,:)];
        t=[t0+t_sim(1:simSmpl-1,:)];
        u=[u_sim(1:simSmpl-1,:)];
        
    end
    fprintf('Iteration: %d\n', data_collection_iteration);
    data_collection_iteration=data_collection_iteration+1;
    if err_P(end)<=trash_hold
        break
    end
end
err_P(end)
err_K(end)
%% Plot the results
% fontname = 'Times New Roman'; 
% set(0,'DefaultAxesFontName',fontname,'DefaultTextFontName',fontname);
make_it_tight = true;
subplot = @(m,n,p) subtightplot (m, n, p, [0.06 0.05], [0.1 0.015], [0.045 0.015]);
if ~make_it_tight,  clear subplot;  end

subplot(2,2,1)
plot(1:numel(err_P),err_P,'-o','LineWidth',1.2,'MarkerSize', 4)
set(gca,'FontSize',10)
legend('$$||{P}^*-{P}_{i}||_F$$','Interpreter','latex','FontSize', 12)
xlabel('Iteration No.')
xlim([1 numel(err_P)])
subplot(2,2,2)
plot(1:numel(err_K),err_K,'-d','LineWidth',1.2,'MarkerSize', 4)
set(gca,'FontSize',10)
legend('$$||{K}^*-{K}_{i}||_F$$','Interpreter','latex','FontSize', 12)
xlim([1 numel(err_K)])
xlabel('Iteration No.')
subplot(2,2,3)
hold on
plot(t,u,'LineWidth',1.2)
set(gca,'FontSize',10)
xlabel('Time (sec)')
legend('$$u_1$$','$$u_2$$','Interpreter','latex','FontSize', 12)
hold off
subplot(2,2,4)
plot(t,x,'LineWidth',1.2)
set(gca,'FontSize',10)
legend('$$x_1$$','$$x_2$$','$$x_3$$','$$x_4$$','Interpreter','latex','FontSize', 12)
xlabel('Time (sec)')

function x_bar=x_basis(x)
%calculate basis functions
% x=[---states-->]
%   [||||||||||||]
%   [||||time||||]  
%   [||||||||||||]
n=size(x,2);%number of states of x
t=size(x,1);%time samples of x
x_bar=zeros(t,n*n);
for i=1:t
    x_bar(i,:)=kron(x(i,:),x(i,:));
end
end

function P=P_mat(p)
%Create matrix P from its vector form
n=sqrt(size(p,1));
k=1;
P=reshape(p,[n,n]);
P=(P+P')/2;
end