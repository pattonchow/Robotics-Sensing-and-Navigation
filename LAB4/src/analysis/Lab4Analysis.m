format long g

clc, clear

vFontSize = 14;
vLineWidth = 4;
vFontWeight = 'bold';

%list all rosbag

circle_moving_bag = rosbag("./Data/ruggles.bag");

% circle_moving_bag = rosbag("./Data/backbay_tour.bag");
%get topic
gps_topic = select(circle_moving_bag,"Topic","/gps");
imu_topic = select(circle_moving_bag,"Topic","/imu");

%get message
gps_message = readMessages(gps_topic,"DataFormat","struct");
imu_message = readMessages(imu_topic,"DataFormat","struct");

%Time from IMU
sec = cellfun(@(m) double(m.Header.Stamp.Sec),imu_message);
nsec = cellfun(@(m) double(m.Header.Stamp.Nsec),imu_message);
time = sec + nsec/10^9;
time = time - time(1);

%Time from GPS
sec = cellfun(@(m) double(m.Header.Stamp.Sec),gps_message);
nsec = cellfun(@(m) double(m.Header.Stamp.Nsec),gps_message);
timeG = sec + nsec/10^9;
timeG = timeG - timeG(1);

%gps message process
Latitude = cellfun(@(m) double(m.Latitude),gps_message);
Longitude = cellfun(@(m) double(m.Longitude),gps_message);
Altitude = cellfun(@(m) double(m.Altitude),gps_message);
zone = cellfun(@(m) string(m.Zone),gps_message);
letter = cellfun(@(m) string(m.Letter),gps_message);

% Magnetic message process
magneticfield(:,1) = cellfun(@(m) double(m.MagField.MagneticField.X),imu_message);
magneticfield(:,2) = cellfun(@(m) double(m.MagField.MagneticField.Y),imu_message);
magneticfield(:,3) = cellfun(@(m) double(m.MagField.MagneticField.Z),imu_message);

% gyro rate
angularRate(:,1) = cellfun(@(m) double(m.IMU.AngularVelocity.X),imu_message);
angularRate(:,2) = cellfun(@(m) double(m.IMU.AngularVelocity.Y),imu_message);
angularRate(:,3) = cellfun(@(m) double(m.IMU.AngularVelocity.Z),imu_message);

% linearAcceleration
linearAcc(:,1) = cellfun(@(m) double(m.IMU.LinearAcceleration.X),imu_message);
linearAcc(:,2) = cellfun(@(m) double(m.IMU.LinearAcceleration.Y),imu_message);
linearAcc(:,3) = cellfun(@(m) double(m.IMU.LinearAcceleration.Z),imu_message);

% Yaw Pitch Roll
% orientationX = cellfun(@(m) double(m.IMU.Orientation.X),imu_message);
% orientationY = cellfun(@(m) double(m.IMU.Orientation.Y),imu_message);
% orientationZ = cellfun(@(m) double(m.IMU.Orientation.Z),imu_message);
% orientationW = cellfun(@(m) double(m.IMU.Orientation.W),imu_message);
% quat = [orientationX orientationY orientationZ orientationW]
% [yaw, pitch, roll] = quat2angle(quat)

yaw = cellfun(@(m) double(m.IMU.Orientation.X),imu_message);
% yaw = yaw - yaw(1)
pitch = cellfun(@(m) double(m.IMU.Orientation.Y),imu_message);
roll = cellfun(@(m) double(m.IMU.Orientation.Z),imu_message);

%deg2utm
utmZone = (zone+letter);
utmZone = utmZone(1);
[ellipsoid,estr] = utmgeoid(utmZone);
utmstruct = defaultm('utm');
utmstruct.zone = utmZone;
utmstruct.geoid = ellipsoid;
utmstruct = defaultm(utmstruct);
[utmE,utmN] = mfwdtran(utmstruct,Latitude,Longitude);

T = [0:1:4727]';
% T = [0:1:26811]';


%% ================================ Part 1 ================================
% Magnetometer Calibration using function magcal

x(:,1) = magneticfield(:,1);
x(:,2) = magneticfield(:,2);
x(:,3) = magneticfield(:,3);

[A,b,expMFS]  = magcal(x);
xCorrected = (x-b)*A;
yawMag = atan2d(xCorrected(:,1),xCorrected(:,2));


%% Plot calibration result method 1
figure(1)
plot(magneticfield(:,1),magneticfield(:,2),'LineWidth',4)
hold on
plot(xCorrected(:,1),xCorrected(:,2),'LineWidth',4)
grid on
hold off

axis equal;
title('Magnetometer Calibration');

%% Plot calibration result method 2 
% Note: HelperDrawEllipsoid.m is required
de = HelperDrawEllipsoid;
de.plotCalibrated(A,b,expMFS,x,xCorrected,'Auto');


%% Angle rate comparison

intAngularRateZ = cumtrapz(time,angularRate(:,3));
intAngularRateZ = rad2deg(intAngularRateZ) + yawMag(1);
intAngularRateZ = wrapTo180(intAngularRateZ);

figure(1)
plot(T,intAngularRateZ,'LineWidth',2)
hold on 
plot(T, yawMag,'-.','LineWidth',2)
hold off

xlabel('time');
ylabel('deg');
legend('Yaw from gyro integration ','Yaw from magnetometer');
title('Yaw comparsion');


%% filter the magnetometer estimate using a low pass filter
fs  = 40;  % Hz

magneticfieldF(:,1) = lowpass(xCorrected(:,1),0.1,fs);
magneticfieldF(:,2) = lowpass(xCorrected(:,2),0.1,fs);
magneticfieldF(:,3) = lowpass(xCorrected(:,3),0.1,fs);

figure(1)

% plot(T,magneticfieldF(:,3))
% hold on 
% plot(T, magneticfield(:,3))
% hold off

% plot3(magneticfield(:,1),magneticfield(:,2),magneticfield(:,3),'LineWidth',4)
% hold on
plot3(xCorrected(:,1),xCorrected(:,2),xCorrected(:,3),'-.','LineWidth',1.5)
hold on
plot3(magneticfieldF(:,1),magneticfieldF(:,2),magneticfieldF(:,3),'LineWidth',4.5)
hold off

% legend('After filter','Before filter');
title('Lowpass filter for magnetometer');


%% gyro estimate using a high pass filter
% NOT FINISHED
fs = 40
fpass = 0.001

angularRateF(:,1) = highpass(angularRate(:,1),fpass,fs);
angularRateF(:,2) = highpass(angularRate(:,2),fpass,fs);
angularRateF(:,3) = highpass(angularRate(:,3),fpass,fs);

intAngularRateFZ = cumtrapz(time,angularRateF(:,3));
intAngularRateFZ = rad2deg(intAngularRateFZ);
intAngularRateFZ = wrapTo180(intAngularRateFZ);

angularRate = angularRate - mean(angularRate);
angularRateF = angularRateF - mean(angularRateF);

figure(3)

plot(T, angularRateF(:,3),'-.','LineWidth',1);
hold on 
plot(T,angularRate(:,3),'LineWidth',1);

hold off

legend('After filter','Before filter');
title('Highpass filter for gyro');

% figure(11)
% plot(T, intAngularRateFZ,'-.','LineWidth',1)
% hold on
% plot(T, yawMag,'LineWidth',1)
% hold off


%% Complementary filter 

Fs  = 40;  % Hz
fuse = complementaryFilter('SampleRate', Fs);
q = fuse(linearAcc,angularRateF, magneticfieldF);
fuseYpr = eulerd( q, 'ZYX', 'frame');
yawFuse = - unwrap(fuseYpr(:,1));
yawFuse = yawFuse - yawFuse(1) + intAngularRateZ(1) - 25;
yawFuse = wrapTo180(yawFuse);


figure(1)
plot(T,yawFuse,'LineWidth',2)
hold on 
plot(T,intAngularRateZ,'-.','LineWidth',2)
hold off

xlabel('time');
ylabel('deg');
legend('Yaw fused by gyro and mag','Yaw from IMU');
title('Yaw comparsion');

% %% ================================ Part 2 ================================
fs = 40;
fpass = 0.001;

linearAccF(:,1)  = lowpass(linearAcc(:,1) ,fpass,fs);

% figure
% plot(time,linearAcc(:,1));
% hold on
% plot(time,linearAccF(:,1),'LineWidth',2);

forwardAcc = linearAccF(:,1);
% forwardAcc = forwardAcc - 0.12042;
forwardAcc = forwardAcc - mean(forwardAcc);
% forwardAcc = forwardAcc - mean(forwardAcc)
forwardV = cumtrapz(time,forwardAcc);

figure
plot(time,linearAccF(:,1),'LineWidth',1.5);
hold on 
plot(time,forwardAcc,'LineWidth',1.5);
hold off

xlabel('time');
ylabel('velocity');
legend('Velocity from IMU','Velocity integrated from forward acceleration');
title('Estimation or forward velocity (After adjustment)');

%% GPS ploy
plot3(utmE,utmN,timeG)

%% comparsion of velocity from GPS and IMU

vtimeG = timeG;
vtimeG(length(vtimeG)) = [];

XYDis(1,:) = utmE;
XYDis(2,:) = utmN;
dXYDis = diff(XYDis'); 
dXYDis = hypot(dXYDis(:,1),dXYDis(:,2))
dvtimeG = diff(timeG);
v = dXYDis ./ dvtimeG;
vtimeG(isnan(v))=[];
v(isnan(v))=[];
vtimeG(find(v >5))=[];
v(find(v >5)) =[];


figure
plot(vtimeG,v,'LineWidth',2)
hold on
plot(time,forwardV,'-.','LineWidth',2);
hold off

%% ================================ Part 3 ================================
% Data reading 

clc, clear

vFontSize = 14;
vLineWidth = 4;
vFontWeight = 'bold';

%list all rosbag
moving_bag = rosbag("./Data/backbay_tour.bag");
% circle_moving_bag = rosbag("./Data/circle6.bag");

% circle_moving_bag = rosbag("./Data/static.bag");
%get topic
gps_topic = select(moving_bag,"Topic","/gps");
imu_topic = select(moving_bag,"Topic","/imu");

%get message
gps_message = readMessages(gps_topic,"DataFormat","struct");
imu_message = readMessages(imu_topic,"DataFormat","struct");

%Time from IMU
sec = cellfun(@(m) double(m.Header.Stamp.Sec),imu_message);
nsec = cellfun(@(m) double(m.Header.Stamp.Nsec),imu_message);
time = sec + nsec/10^9;
time = time - time(1);

%Time from GPS
sec = cellfun(@(m) double(m.Header.Stamp.Sec),gps_message);
nsec = cellfun(@(m) double(m.Header.Stamp.Nsec),gps_message);
timeG = sec + nsec/10^9;
timeG = timeG - timeG(1);


%gps message process
Latitude = cellfun(@(m) double(m.Latitude),gps_message);
Longitude = cellfun(@(m) double(m.Longitude),gps_message);
Altitude = cellfun(@(m) double(m.Altitude),gps_message);
% quality = cellfun(@(m) double(m.Quality),gps_message);
zone = cellfun(@(m) string(m.Zone),gps_message);
letter = cellfun(@(m) string(m.Letter),gps_message);

% Magnetic message process
magneticfield(:,1) = cellfun(@(m) double(m.MagField.MagneticField.X),imu_message);
magneticfield(:,2) = cellfun(@(m) double(m.MagField.MagneticField.Y),imu_message);
magneticfield(:,3) = cellfun(@(m) double(m.MagField.MagneticField.Z),imu_message);

% gyro rate
angularRate(:,1) = cellfun(@(m) double(m.IMU.AngularVelocity.X),imu_message);
angularRate(:,2) = cellfun(@(m) double(m.IMU.AngularVelocity.Y),imu_message);
angularRate(:,3) = cellfun(@(m) double(m.IMU.AngularVelocity.Z),imu_message);

% linearAcceleration
linearAcc(:,1) = cellfun(@(m) double(m.IMU.LinearAcceleration.X),imu_message);
linearAcc(:,2) = cellfun(@(m) double(m.IMU.LinearAcceleration.Y),imu_message);
linearAcc(:,3) = cellfun(@(m) double(m.IMU.LinearAcceleration.Z),imu_message);

% Yaw Pitch Roll
yaw = cellfun(@(m) double(m.IMU.Orientation.X),imu_message);
% yaw = yaw - yaw(1)
pitch = cellfun(@(m) double(m.IMU.Orientation.Y),imu_message);
roll = cellfun(@(m) double(m.IMU.Orientation.Z),imu_message);

%deg2utm
utmZone = (zone+letter);
utmZone = utmZone(1);
[ellipsoid,estr] = utmgeoid(utmZone);
utmstruct = defaultm('utm');
utmstruct.zone = utmZone;
utmstruct.geoid = ellipsoid;
utmstruct = defaultm(utmstruct);
[utmE,utmN] = mfwdtran(utmstruct,Latitude,Longitude);
T = [0:1:30410]';

%% stop 

fs = 40;
hfpass = 0.0003;

linearAccF(:,1)  = highpass(linearAcc(:,1) ,hfpass,fs);
linearAccF(:,2)  = highpass(linearAcc(:,2) ,hfpass,fs);

tem = linearAccF(:,1);
tem(find(abs(tem)<0.1)) = 0;
linearAccF(:,1) = tem;

tem = linearAccF(:,2);
tem(find(abs(tem)<0.1)) = 0;
linearAccF(:,2) = tem;

plot(tem)


%% position of the center-of-mass

lfpass = 25;
linearAccF(:,1)  = lowpass(linearAcc(:,1) ,lfpass,fs);

tem = linearAccF(:,1);
plot(tem)
tem(find(abs(tem)<0.3)) = 0;

linearAccF(:,1) = tem;

% Angular rate about the CM
w = rad2deg(angularRate(:,3));
w = angularRate(:,3);

% Plot to see the data
figure
plot(utmE,utmN,'Linewidth',4) 
grid on
xlabel('utmE','FontSize',vFontSize,'FontWeight', vFontWeight)
ylabel('utmN','FontSize',vFontSize,'FontWeight', vFontWeight)
legend({'rawData'},'Location','southwest','FontSize',vFontSize)

figure
plot(time,w,'LineWidth',2);
grid on

vtimeG = timeG;
vtimeG(length(vtimeG)) = [];

% Method 1 to get X and X'

XYDis(1,:) = utmE;
XYDis(2,:) = utmN;
% X wrt. car frame by abs position
X = hypot(utmE,utmN);
dXYDis = diff(XYDis'); 
dXYDis = hypot(dXYDis(:,1),dXYDis(:,2));
dvtimeG = diff(timeG);
% X dot wrt. car frame by diff X
Xdot = dXYDis ./ dvtimeG;
vtimeG(isnan(Xdot))=[];
Xdot(isnan(Xdot))=[];
vtimeG(find(Xdot >10))=[];
Xdot(find(Xdot >10)) =[];

% X2dot - the acceleration measured by the inertial sensocr 
X2dotImu = linearAccF(:,1);
X2dotImu = X2dotImu - mean(X2dotImu);
% plot(time,X2dotImu)

% Xdot by int X2dot
XdotImu = cumtrapz(time,X2dotImu);
X2dotImu(find(abs(XdotImu<1))) = 0;
% XdotImu = cumtrapz(time,X2dotImu);

figure
plot(vtimeG,Xdot,'LineWidth',2)
grid on
hold on 
plot(time,XdotImu,'LineWidth',2)
hold off

xlabel('time');
ylabel('velocity');
legend('Velocity from GPS','Velocity integrated from forward acceleration');
title('Velocity from GPS vs. Velocity from IMU');

%% Part3

% 3.1

Y2dotImu = w.*XdotImu;
Ytime = time;
% Ytime(find(abs(Y2dotImu)>50))=[];
% Y2dotImu(find(abs(Y2dotImu)>50))=[];

figure
plot(Ytime,Y2dotImu,'-.','LineWidth',2.5);
hold on 
plot(time,linearAccF(:,2),'LineWidth',1)
grid on
hold off

xlabel('time');
ylabel('velocity');
legend("Ydot by w*x","Ydot from IMU")
title('Velocity by w*Xdot vs. Velocity from IMU');

%% 3.2
% Magnetometer Calibration using function magcal

x(:,1) = magneticfield(:,1);
x(:,2) = magneticfield(:,2);
x(:,3) = magneticfield(:,3);

fs  = 40;  % Hz

[A,b,expMFS]  = magcal(x);
xCorrected = (x-b)*A;

magneticfieldF(:,1) = lowpass(xCorrected(:,1),0.1,fs);
magneticfieldF(:,2) = lowpass(xCorrected(:,2),0.1,fs);
magneticfieldF(:,3) = lowpass(xCorrected(:,3),0.1,fs);

yawMag = atan2d(magneticfieldF(:,1),magneticfieldF(:,2));

figure(1)
plot3(xCorrected(:,1),xCorrected(:,2),time,'LineWidth',4)

%% 
yaw=cumtrapz(time, w);

uncali_Xdot = cumtrapz(time,linearAcc(:,1));
uncali_Ydot = cumtrapz(time,linearAcc(:,2));

temp1 = linearAccF(:,1);
Xdot = cumtrapz(time,temp1-mean(temp1));
Ydot = cumtrapz(time,temp1-mean(temp1));

tmp=zeros(size(Xdot,1),1);

XYdot = [Xdot Ydot tmp];
wXdot=w.*Xdot;
ydot2=linearAcc(:,2);


delta_easting=diff(utmE);
delta_northing=diff(utmN);
delta_time=diff(timeG);

gps_vel_easting = delta_easting./delta_time;
gps_vel_northing = delta_northing./delta_time;

x(:,1) = magneticfield(:,1);
x(:,2) = magneticfield(:,2);
x(:,3) = magneticfield(:,3);

[A,b,expMFS]  = magcal(x);
xCorrected = (x-b)*A;
yawMag = atan2d(xCorrected(:,1),xCorrected(:,2));

%ddot {x} = dot {v} +¦Ø¡Áv = ddot {X} + dot {¦Ø} ¡Á r+¦Ø¡Á dot {X} +¦Ø¡Á(¦Ø¡Ár)

tmp=zeros(size(linearAcc(:,1),1),1);
Xddot=[linearAcc(:,1) linearAcc(:,2)];
Yddot=Y2dotImu;

dt=1/40;
imu_x=0;
imu_y=0;
mag_x=0;
mag_y=0;

% yaw = deg2rad(yawMag);

for i = 2:size(Xdot)
    if yaw(i) > -1.5707963268
        imu_x(i) = imu_x(i-1) + cos(-yaw(i)) * Xdot(i-1) * dt;
        imu_y(i) = imu_y(i-1) + sin(-yaw(i)) * Xdot(i-1) * dt;
    end
    if (yaw(i) < -1.5707963268)
        if yaw(i) > -3.14
            imu_x(i) = imu_x(i-1) + cos(-yaw(i)) * Xdot(i-1) * dt;
            imu_y(i) = imu_y(i-1) + sin(-yaw(i)) * Xdot(i-1) * dt;
        end
        if yaw(i) < -3.14
            imu_x(i) = imu_x(i-1) + cos(yaw(i)) * Xdot(i-1) * dt;
            imu_y(i) = imu_y(i-1) + sin(yaw(i)) * Xdot(i-1) * dt;
        end

    end
end

% figure
% plot(time,yaw)
% hold on
% plot(time,yawMag)


theta_vec = -0.6*pi;
r2gps = [cos(theta_vec), sin(theta_vec);
-sin(theta_vec),cos(theta_vec)];

imu_xy = r2gps * [imu_x; imu_y];
imu_xy = imu_xy';

theta_vec = -0.6*pi;
magR2gps = [cos(theta_vec), sin(theta_vec);
-sin(theta_vec),cos(theta_vec)];

mag_xy = magR2gps * [mag_x; mag_y];
mag_xy = mag_xy';

figure
idx=1;
idx1=size(imu_xy,1);
plot(imu_xy(idx:idx1,1)-imu_xy(idx,1),imu_xy(idx:idx1,2)-imu_xy(idx,2),'-.','LineWidth',2.5)
hold on;

scaling_ratio=size(time)/size(timeG);
vector=[0:1:size(gps_vel_easting)-1];
idx2=size(utmE,1);
plot(utmE(1:idx2)-utmE(1), utmN(1:idx2)-utmN(1),'-','LineWidth',2);

legend("Estimation from Gyro","Ground truth from GPS")

figure

xc=0.25;
est_utm_x = cumtrapz(time,cos(w) * xc);
est_utm_y = cumtrapz(time,-sin(w) * xc);
est_utm_x = est_utm_x + imu_x';
est_utm_y = est_utm_y + imu_y';
est_utm_xy = 0.85 * r2gps * [est_utm_x'; est_utm_y'];
est_utm_xy=est_utm_xy';

idx=1;

idx1=floor(size(imu_xy,1));
plot(imu_xy(1:idx1,1)-imu_xy(1,1),imu_xy(1:idx1,2)--imu_xy(1,2),'-.','LineWidth',2.5)
hold on

idx2=floor(size(est_utm_xy,1));

plot(est_utm_xy(1:idx2,1)-est_utm_xy(1,1),est_utm_xy(1:idx2,2)-est_utm_xy(1,2),'LineWidth',2.5);

%plot(imu_x, imu_y);

hold on


idx3=floor(size(utmE,1));

plot(utmE(1:idx3)-utmE(1), utmN(1:idx3)-utmN(1),'LineWidth',2.5);

xlabel('easting (m)');
ylabel('northing (m)');
legend('estimate trajectory from gyro','Xc','GPS signal');
axis equal;
title('Trajectory Comparison');

xc=mean((ydot2-wXdot(:,1)));
