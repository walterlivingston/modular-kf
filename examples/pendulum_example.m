clear; close all; clc;

addpath(genpath('../'));

%% Variables
% Constants
g = -9.81;          % Acceleration Due to Gravity [m/s/s]

% System Parameters
Jp = 2.5;           % Mass Moment of Inertia @ Pin [Nm rad/s/s]
m = 1.6;            % Mass [kg]
l = 1;              % Length [m]
J = Jp + m*l^2;     % Mass Moment of Inertia @ CG [Nm rad/s/s]
b = 1.25;           % Rotational Damping Constant [Nm/rad/s]

% Noise Parameters
sigmaE = deg2rad(0.1);
sigmaT = deg2rad(0.01);
sigmaF = 0.1;

% Time Parameters
Fs = 20;            % Sampling Frequency [Hz]
[time, dt, N] = setupTimeParams(0, 10, Fs);

%% Simulation
initialAngle = deg2rad(30);
y = zeros(2,N);     % Measurement Vector [deg : deg/s]
truth = zeros(2,N); % Truth Vector [deg]
[y(1,1), truth(1,1)] = deal(initialAngle);   % Initial Angle [deg]
[y(1,1), y(2,1), truth(1,1), truth(2,1)] = ...
    simPendulum(0, y(1,1), ...
        PendulumType    = "normal", ...
        SystemSigma     = sigmaF, ...
        EncoderSigma    = sigmaE, ...
        EncoderBias     = 0, ...
        TachometerSigma = sigmaT, ...
        TachometerBias  = 0);
for i = 2:N
  [y(1,i), y(2,i), truth(1,i), truth(2,i)] = simPendulum(0);
end

%% Kalman Filter
stateSigmas = sigmaF;
measSigmas = sigmaE;

sBlock = pendulumStateBlock(2, stateSigmas, J, m, l, b);
mBlock = pendulumMeasBlock(measSigmas);
covBlock = basicCovBlock();

KF = kf(sBlock, mBlock, covBlock, ...
                   x_i = [initialAngle 0]', ...
                   mode = 'linear');

EKF = kf(sBlock, mBlock, covBlock, ...
         x_i = [initialAngle 0]', ...
         mode = 'extended');

[xKF, xEKF] = deal(zeros(2,N));
[xKF(1,1), xEKF(1,1)] = deal(initialAngle);
for k = 2:N
    KF.process(dt);
    EKF.process(dt);

    KF.update(y(:,k));
    EKF.update(y(:,k));

    xKF(:,k) = KF.x;
    xEKF(:,k) = EKF.x;
end

%% Results
secs = seconds(time);

figure();
tiledlayout(2,1);
nexttile();
hold('on');
plot(secs, rad2deg(truth(1,:)));
plot(secs, rad2deg(xKF(1,:)), '--');
plot(secs, rad2deg(xEKF(1,:)), '.-');
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Pendulum Angle');
legend('Truth', 'KF', 'EKF');

nexttile();
hold('on');
plot(secs, rad2deg(truth(2,:)));
plot(secs, rad2deg(xKF(2,:)));
plot(secs, rad2deg(xEKF(2,:)));
xlabel('Time (s)');
ylabel('Angular Rate (deg)');
title('Pendulum Angular Rate');
legend('Truth', 'KF', 'EKF');


