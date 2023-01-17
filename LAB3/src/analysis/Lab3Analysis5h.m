% clc, clear
format longG

vFontSize = 14
vLineWidth = 4
vFontWeight = 'bold'

%list all rosbag
time5h_bag = rosbag("./Data/5h.bag")

mag_topic = select(time5h_bag,"Topic","/imu")
mag_message = readMessages(mag_topic,"DataFormat","struct");



% Orientation

% yaw = cellfun(@(m) double(m.Ypr.X),mag_message);

% pitch = cellfun(@(m) double(m.Ypr.Y),mag_message);

% roll = cellfun(@(m) double(m.Ypr.Z),mag_message);

% [yaw, pitch, roll] = quat2angle([orientationX orientationY orientationZ orientationW])

% % AngularVelocity
% angularvelocityX = cellfun(@(m) double(m.IMU.AngularVelocity.X),mag_message);

% angularvelocityY = cellfun(@(m) double(m.IMU.AngularVelocity.Y),mag_message);

% angularvelocityZ = cellfun(@(m) double(m.IMU.AngularVelocity.Z),mag_message);

% % LinearAcceleration
% linearaccelerationX = cellfun(@(m) double(m.IMU.LinearAcceleration.X),mag_message);

% linearaccelerationY = cellfun(@(m) double(m.IMU.LinearAcceleration.Y),mag_message);

% linearaccelerationZ = cellfun(@(m) double(m.IMU.LinearAcceleration.Z),mag_message);
theta = cellfun(@(m) double(m.IMU.LinearAcceleration.Z),mag_message);


t0 = 1/40;

maxNumM = 100;
L = size(theta, 1);
maxM = 2.^floor(log2(L/2));
m = logspace(log10(1), log10(maxM), maxNumM).';
m = ceil(m); % m must be an integer.
m = unique(m); % Remove duplicates.

tau = m*t0;

avar = zeros(numel(m), 1);
for i = 1:numel(m)
    mi = m(i);
    avar(i,:) = sum( ...
        (theta(1+2*mi:L) - 2*theta(1+mi:L-mi) + theta(1:L-2*mi)).^2, 1);
end
avar = avar ./ (2*tau.^2 .* (L - 2*m));
adev = sqrt(avar);

slope = -0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the angle random walk coefficient from the line.
logN = slope*log(1) + b;
N = 10^logN

% Plot the results.
tauN = 1;
lineN = N ./ sqrt(tau);

% Find the index where the slope of the log-scaled Allan deviation is equal
% to the slope specified.
slope = 0.5;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the rate random walk coefficient from the line.
logK = slope*log10(3) + b;
K = 10^logK

% Plot the results.
tauK = 3;
lineK = K .* sqrt(tau/3);


% to the slope specified.
slope = 0;
logtau = log10(tau);
logadev = log10(adev);
dlogadev = diff(logadev) ./ diff(logtau);
[~, i] = min(abs(dlogadev - slope));

% Find the y-intercept of the line.
b = logadev(i) - slope*logtau(i);

% Determine the bias instability coefficient from the line.
scfB = sqrt(2*log(2)/pi);
logB = b - log10(scfB);
B = 10^logB

% Plot the results.
tauB = tau(i);
lineB = B * scfB * ones(size(tau));

tauParams = [tauN, tauK, tauB];
params = [N, K, scfB*B];
figure
loglog(tau, adev, tau, [lineN, lineK, lineB], '--', ...
    tauParams, params, 'o')
title('Allan Deviation with Noise Parameters')
xlabel('\tau')
ylabel('\sigma(\tau)')
legend('$\sigma (rad/s)$', '$\sigma_N ((rad/s)/\sqrt{Hz})$', ...
    '$\sigma_K ((rad/s)\sqrt{Hz})$', '$\sigma_B (rad/s)$', 'Interpreter', 'latex')
text(tauParams, params, {'N', 'K', '0.664B'})
grid on
axis equal

