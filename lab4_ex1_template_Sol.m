clear;
clc;
close all;

%% SET SIMULATIOM TYPE
enableClosedLoop = true;
enableClosedLoopWithObserver = true; % Set to true/false as needed for the exercise

if enableClosedLoopWithObserver
    desc = "closed loop with observer";
elseif enableClosedLoop
    desc = "closed loop";
else
    desc = "open loop";
end

%% SET SYSTEM PARAMETERS
alfa = 1.4;
beta = 1.8;
gamma = 1.05;
m = 13.5;
J = 0.37;

A = [0,0,0,1,0,0;
    0,0,0,0,1,0;
    0,0,0,0,0,1;
    0,0,0,-alfa/m,0,0;
    0,0,0,0,-beta/m,0;
    0,0,0,0,0,-gamma/J];

xf = [-1;2;3;0;0;0];
testdesc = "Test";

% B Matrix definition 
% Relates inputs (Force/Torque) to acceleration (u_dot, v_dot, r_dot)
B = zeros(6,3);
B(4,1) = 1/m;
B(5,2) = 1/m;
B(6,3) = 1/J;

usize = min(size(B));
T = 25;
dt = 0.01;

% C Matrix definition 
% We measure Position (x, y) and Yaw (psi) only.
C = zeros(3,6);
C(1,1) = 1; 
C(2,2) = 1; 
C(3,3) = 1;

syms tau real
tic
eA = expm(A*(T-tau));
dG = eA*B*B'*eA';
G = vpa(int(dG,tau,0,T));
rankG = rank(G);
disp(['rank(G) = ' num2str(rankG)]);
if rankG == 6
    iG = inv(G);
else
    iG = pinv(G);
    warning("System has rank " + rankG + "! It is not fully controllable.")
end
u(tau) = B'*eA'*iG*xf;
toc

time = [0:dt:T];
time = time';
LL = length(time);
u_time = u(time);
uvalue = [eval(u_time{1}), eval(u_time{2}), eval(u_time{3})];

x0 = [0 0 0 0 0 0]'; % we start from the origin
x = zeros(6,LL); % this is only needed to log the results and plot them
xHat = zeros(6,LL); % this is only needed to log the results and plot them

% Initialize Observer State with error as requested in exercise
% Error = [1, -1, 1.5, 0.5, -0.5, -1.5]
initial_error = [1; -1; 1.5; 0.5; -0.5; -1.5];
xHat(:,1) = x0 + initial_error; 
x(:,1) = x0;

%% SET CONTROLLER PARAMETERS
% Set Kc (controlability)
Kc = zeros(3,6);
% Set parameters
Tau = 1;
xi = 1;
wn = 1;
kc1 = -m * wn^2;
kc2 = -m * wn^2;
kc3 = -J * wn^2;
kc4 = alfa - 2*xi*wn*m;
kc5 = beta - 2*xi*wn*m;
kc6 = gamma - 2*xi*wn*J;

Kc(1:3,1:3) = diag([kc1, kc2, kc3]);
Kc(1:3,4:6) = diag([kc4, kc5, kc6]);

%% SET OBSERVER PARAMETERS
% Set Ko (observability) 
Tau = 1;
xi = 1;
Ko = zeros(6,3);
wn = 10; % Observer must be faster than controller
kx1 = 2*xi*wn - alfa/m;
kx2 = 2*xi*wn - beta/m;
kx3 = 2*xi*wn - gamma/J;
kx4 = wn^2 - kx1*alfa/m;
kx5 = wn^2 - kx2*beta/m;
kx6 = wn^2 - kx3*gamma/J;

% Constructing Ko matrix based on the decoupled structure
Ko(1:3,1:3) = diag([kx1, kx2, kx3]);
Ko(4:6,1:3) = diag([kx4, kx5, kx6]);

%% SIMULATION
u_stdev = 0.1;
y_stdev = 0.0; % You can increase noise to test robustness
x_ol = x * 0; % ol means Open Loop
x_ol(:,1) = x0;

y = zeros(min(size(C)),LL);
yHat = y * 0;
y_noise = y * 0;

% Initial measurement
y(:,1) = C * x(:,1);
yHat(:,1) = C * xHat(:,1);

uref = zeros(3, LL); % Initialize uref for storage

for k = 1:LL-1
    
    %% Control
    u_k = uvalue(k,:)'; 
    
    % Open Loop Dynamics (Reference Trajectory)
    xdot_ol_k = A*x_ol(:,k) + B * u_k;
    x_ol(:,k+1) = x_ol(:,k) + xdot_ol_k * dt;
    
    if ~enableClosedLoop
        % Open Loop: Apply pre-calculated u_k directly
        xdot_k = A*x(:,k) + B*u_k;
        x(:,k+1) = x(:,k) + xdot_k * dt;
        uref(:,k) = u_k;
    else
        % Closed Loop Logic
        if enableClosedLoopWithObserver
            % Feedback using Estimated State (xHat)
            % u = u_feedforward - Kc * (x_estimated - x_reference)
            uref(:,k) = u_k + Kc * (xHat(:,k) - x_ol(:,k));
        else
            % Feedback using True State (Full State Feedback)
            % u = u_feedforward - Kc * (x_true - x_reference)
            uref(:,k) = u_k + Kc * (x(:,k) - x_ol(:,k));
        end
        
        % Apply control to the real system
        xdot_k = A*x(:,k) + B*uref(:,k);
        x(:,k+1) = x(:,k) + xdot_k * dt;
    end
    
    %% Measurement update
    y(:,k+1) = C * x(:,k+1);
    
    %% Observer
    % Add noise to measurements
    y_noise(1:2,k) = y(1:2,k) + y_stdev*randn(2,1);
    y_noise(3,k) = y(3,k) + y_stdev*randn;
    
    u_obs_k = uref(:,k); % The observer knows the input applied
    
    % Luenberger Observer Equation:
    % x_dot_hat = A*x_hat + B*u + Ko*(y_measured - C*x_hat)
    xdotHat = A*xHat(:,k) + B*u_obs_k + Ko*(y_noise(:,k) - C*xHat(:,k));
    
    xHat(:,k+1) = xHat(:,k) + xdotHat * dt;
    yHat(:,k+1) = C * xHat(:,k+1);
end

%% PLOTS
%% Plot states
figure('Units', 'normalized', 'OuterPosition', [0 0 1 1]); % Set figure to full screen
sgtitle("State " + desc) 
% Subplot 1: X Position
subplot(2,3,1);
plot(time, x(1,:),'linewidth',3);
hold on;
plot(time, x_ol(1,:), '--', 'linewidth', 2); % Plot reference trajectory
plot(time, ones(size(time)) * xf(1), 'k:', 'linewidth',1); % Target
xlabel('Time[s]');
ylabel('X[m]');
legend('State','Reference Traj', 'Target');
grid on;
title('X');

% Subplot 2: Y Position
subplot(2,3,2);
plot(time, x(2,:), 'linewidth',3);
hold on;
plot(time, x_ol(2,:), '--', 'linewidth', 2);
plot(time, ones(size(time)) * xf(2), 'k:', 'linewidth',1);
xlabel('Time[s]');
ylabel('Y[m]');
legend('State','Reference Traj', 'Target');
grid on;
title('Y');

% Subplot 3: Theta
subplot(2,3,3);
plot(time, x(3,:), 'linewidth',3);
hold on;
plot(time, x_ol(3,:), '--', 'linewidth', 2);
plot(time, ones(size(time)) * xf(3), 'k:', 'linewidth',1);
xlabel('Time[s]');
ylabel('Yaw[rad]');
legend('State','Reference Traj', 'Target');
grid on;
title('Yaw');

% Subplot 4: v_x
subplot(2,3,4);
plot(time, x(4,:), 'linewidth',3);
hold on;
plot(time, x_ol(4,:), '--', 'linewidth', 2);
plot(time, ones(size(time)) * xf(4), 'k:', 'linewidth',1);
xlabel('Time[s]');
ylabel('v_x[m/s]');
legend('State','Reference Traj', 'Target');
grid on;
title('v_x');

% Subplot 5: v_y
subplot(2,3,5);
plot(time, x(5,:), 'linewidth',3);
hold on;
plot(time, x_ol(5,:), '--', 'linewidth', 2);
plot(time, ones(size(time)) * xf(5), 'k:', 'linewidth',1);
xlabel('Time[s]');
ylabel('v_y[m/s]');
legend('State','Reference Traj', 'Target');
grid on;
title('v_y');

% Subplot 6: Yaw rate
subplot(2,3,6);
plot(time, x(6,:),'linewidth',3);
hold on;
plot(time, x_ol(6,:), '--', 'linewidth', 2);
plot(time, ones(size(time)) * xf(6), 'k:', 'linewidth',1);
xlabel('Time[s]');
ylabel('Yaw rate[rad/s]');
legend('State','Reference Traj', 'Target');
grid on;
title('Yaw rate');

%% State stimation error
if enableClosedLoopWithObserver
    figure('Units', 'normalized', 'OuterPosition', [0 0 1 1]); % Set figure to full screen
    sgtitle("State estimation error " + desc) 
    % Subplot 1: X Position
    subplot(2,3,1);
    plot(time, x(1,:) - xHat(1,:),'linewidth',3);
    xlabel('Time[s]');
    ylabel('X[m]');
    grid on;
    title('X Error');

    % Subplot 2: Y Position
    subplot(2,3,2);
    plot(time, x(2,:) - xHat(2,:), 'linewidth',3);
    xlabel('Time[s]');
    ylabel('Y[m]');
    grid on;
    title('Y Error');

    % Subplot 3: Theta
    subplot(2,3,3);
    plot(time, x(3,:) - xHat(3,:), 'linewidth',3);
    xlabel('Time[s]');
    ylabel('Yaw[rad]');
    grid on;
    title('Yaw Error');

    % Subplot 4: v_x
    subplot(2,3,4);
    plot(time, x(4,:) - xHat(4,:), 'linewidth',3);
    xlabel('Time[s]');
    ylabel('v_x[m/s]');
    grid on;
    title('v_x Error');

    % Subplot 5: v_y
    subplot(2,3,5);
    plot(time, x(5,:) - xHat(5,:), 'linewidth',3);
    xlabel('Time[s]');
    ylabel('v_y[m/s]');
    grid on;
    title('v_y Error');

    % Subplot 6: Yaw rate
    subplot(2,3,6);
    plot(time, x(6,:) - xHat(6,:), 'linewidth',3);
    xlabel('Time[s]');
    ylabel('Yaw rate[rad/s]');
    grid on;
    title('Yaw rate Error');
end

figure;
plot(time,uref(1,:), 'linewidth',3)
hold on
plot(time,uref(2,:), 'linewidth',3)
plot(time,uref(3,:), 'linewidth',3)
legend('u_1 (Fx)','u_2 (Fy)','u_3 (Nz)')
xlabel('Time [s]')
ylabel('Inputs')
title(['Input ',newline,desc])
grid on;

% Assuming you have time vector t corresponding to your data
figure;
scatter(x(1,:), x(2,:), [], time, 'filled'); % Plot with color mapping to time
colormap(jet); % Set colormap
cb = colorbar; % Add colorbar
cb.Label.String = 'time [s]'; % Add label to colorbar
xlabel('x [m]');
ylabel('y [m]');
title(['Trajectory ', desc]); % Corrected title
grid on;

disp("Final state error (Target - Actual):")
disp(xf - x(:,end))

if enableClosedLoopWithObserver
    disp("Final estimation error (Actual - Estimated):")
    disp(x(:,end) - xHat(:,end))
end