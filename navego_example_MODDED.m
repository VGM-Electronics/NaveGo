%           Analog Devices. ADIS16400/ADIS16405 datasheet. High Precision 
% Tri-Axis Gyroscope, Accelerometer, Magnetometer. Rev. B. 
% http://www.analog.com/media/en/technical-documentation/data-sheets/ADIS16400_16405.pdf
% 
%			Garmin International, Inc. GPS 18x TECHNICAL SPECIFICATIONS.
% Revision D. October 2011. 
% http://static.garmin.com/pumac/GPS_18x_Tech_Specs.pdf
% 
% NOTE: NaveGo supposes that IMU is aligned with respect to body-frame as X-forward, Y-right, and Z-down.

clc;
close all;
clear;
matlabrc;
addpath('Data');
addpath('Library');

versionstr = 'NaveGo, release v1.0';

fprintf('\n%s.\n', versionstr)
fprintf('\nNaveGo: starting simulation ... \n')

%% 1) CODE EXECUTION PARAMETERS

% Comment any of the following parameters in order to NOT execute a particular portion of code

% GPS_DATA            = 'ON';                                                 % Simulate GPS data
% IMU_DATA            = 'ON';                                                 % Simulate ADIS16405 IMU data

IMU_INS             = 'ON';                                                 % Execute INS/GPS integration for ADIS16405 IMU

PLOT                = 'ON';                                                 % Plot results.

% If a particular parameter is commented above, it is set by default to 'OFF'.

if (~exist('GPS_DATA','var')),  GPS_DATA  = 'OFF'; end
if (~exist('IMU_DATA','var')), IMU_DATA = 'OFF'; end
if (~exist('IMU_INS','var')),  IMU_INS  = 'OFF'; end
if (~exist('PLOT','var')),      PLOT      = 'OFF'; end

%% 2) CONVERSION CONSTANTS

G                   = 9.814;                                                % Gravity constant, m/s^2
G2MSS               = G;                                                    % g to m/s^2
MSS2G               = (1/G);                                                % m/s^2 to g

D2R                 = (pi/180);                                             % degrees to radians
R2D                 = (180/pi);                                             % radians to degrees

KT2MS               = 0.514444;                                             % knot to m/s
MS2KMH              = 3.6;                                                  % m/s to km/h

%% 3) LOAD REFERENCE DATA

fprintf('NaveGo: loading reference dataset from a trajectory generator... \n')

load ref.mat

% ref.mat contains the reference data structure from which inertial 
% sensors and GPS wil be simulated. It must contain the following fields:

%         t: Nx1 time vector (seconds).
%       lat: Nx1 latitude (radians).
%       lon: Nx1 longitude (radians).
%         h: Nx1 altitude (m).
%       vel: Nx3 NED velocities (m/s).
%      roll: Nx1 roll angles (radians).
%     pitch: Nx1 pitch angles (radians).
%       yaw: Nx1 yaw angle vector (radians).
%        kn: 1x1 number of elements of ref time vector.
%     DCMnb: Nx9 Direct Cosine Matrix nav-to-body. Each row contains 
%            the elements of one DCM matrix ordered by columns as 
%            [a11 a21 a31 a12 a22 a32 a13 a23 a33].
%      freq: sampling frequency (Hz).

%% 4) ADIS16405 IMU error profile

% IMU data structure:
%         t: Ix1 time vector (seconds).
%        fb: Ix3 accelerations vector in body frame XYZ (m/s^2).
%        wb: Ix3 turn rates vector in body frame XYZ (radians/s).
%       arw: 1x3 angle random walks (rad/s/root-Hz).
%      arrw: 1x3 angle rate random walks (rad/s^2/root-Hz).
%       vrw: 1x3 velocity random walks (m/s^2/root-Hz).
%      vrrw: 1x3 velocity rate random walks (m/s^3/root-Hz).
%    gb_std: 1x3 gyros standard deviations (radians/s).
%    ab_std: 1x3 accrs standard deviations (m/s^2).
%    gb_fix: 1x3 gyros static biases or turn-on biases (radians/s).
%    ab_fix: 1x3 accrs static biases or turn-on biases (m/s^2).
%  gb_drift: 1x3 gyros dynamic biases or bias instabilities (radians/s).
%  ab_drift: 1x3 accrs dynamic biases or bias instabilities (m/s^2).
%   gb_corr: 1x3 gyros correlation times (seconds).
%   ab_corr: 1x3 accrs correlation times (seconds).
%    gb_psd: 1x3 gyros dynamic biases PSD (rad/s/root-Hz).
%    ab_psd: 1x3 accrs dynamic biases PSD (m/s^2/root-Hz);
%      freq: 1x1 sampling frequency (Hz).
% ini_align: 1x3 initial attitude at t(1), [roll pitch yaw] (rad).
% ini_align_err: 1x3 initial attitude errors at t(1), [roll pitch yaw] (rad).

ADIS16405.arw       = 2   .* ones(1,3);                                     % Angle random walks [X Y Z] (deg/root-hour)
ADIS16405.arrw      = zeros(1,3);                                           % Angle rate random walks [X Y Z] (deg/root-hour/s)
ADIS16405.vrw       = 0.2 .* ones(1,3);                                     % Velocity random walks [X Y Z] (m/s/root-hour)
ADIS16405.vrrw      = zeros(1,3);                                           % Velocity rate random walks [X Y Z] (deg/root-hour/s)
ADIS16405.gb_fix    = 3   .* ones(1,3);                                     % Gyro static biases [X Y Z] (deg/s)
ADIS16405.ab_fix    = 50  .* ones(1,3);                                     % Acc static biases [X Y Z] (mg)
ADIS16405.gb_drift  = 0.007 .* ones(1,3);                                   % Gyro dynamic biases [X Y Z] (deg/s)
ADIS16405.ab_drift  = 0.2 .* ones(1,3);                                     % Acc dynamic biases [X Y Z] (mg)
ADIS16405.gb_corr	= 100 .* ones(1,3);                                     % Gyro correlation times [X Y Z] (seconds)
ADIS16405.ab_corr   = 100 .* ones(1,3);                                     % Acc correlation times [X Y Z] (seconds)
ADIS16405.freq      = ref.freq;                                             % IMU operation frequency [X Y Z] (Hz)
% ADIS16405.m_psd     = 0.066 .* ones(1,3);                                   % Magnetometer noise density [X Y Z] (mgauss/root-Hz)

% ref dataset will be used to simulate IMU sensors.
ADIS16405.t         = ref.t;                                                % IMU time vector
dt                  = mean(diff(ADIS16405.t));                              % IMU mean period

IMU                = imu_si_errors(ADIS16405, dt);                          % Transform IMU manufacturer error units to SI units.

IMU.ini_align_err	= [3 3 10] .* D2R;                                      % Initial attitude align errors for matrix P in Kalman filter, [roll pitch yaw] (radians)  
IMU.ini_align      = [ref.roll(1) ref.pitch(1) ref.yaw(1)];                 % Initial attitude align at t(1) (radians).

%% 5) Garmin 5-18 Hz GPS error profile

% GPS data structure:
%         t: Mx1 time vector (seconds).
%       lat: Mx1 latitude (radians).
%       lon: Mx1 longitude (radians).
%         h: Mx1 altitude (m).
%       vel: Mx3 NED velocities (m/s).
%       std: 1x3 position standard deviations, [lat lon h] (rad, rad, m).
%      stdm: 1x3 position standard deviations, [lat lon h] (m, m, m).
%      stdv: 1x3 velocity standard deviations, [Vn Ve Vd] (m/s).
%      larm: 3x1 lever arm (x-right, y-fwd, z-down) (m).
%      freq: 1x1 sampling frequency (Hz).

gps.stdm            = [5, 5, 10];                                           % GPS positions standard deviations [lat lon h] (meters)
gps.stdv        	= 0.1 * KT2MS .* ones(1,3);                             % GPS velocities standard deviations [Vn Ve Vd] (meters/s)
gps.larm            = zeros(3,1);                                           % GPS lever arm from IMU to GPS, X-fwd, Y-right, Z-down (meters)
gps.freq            = 5;                                                    % GPS operation frequency (Hz)

%% 6) SIMULATE GPS

rng('shuffle')                                                              % Reset pseudo-random seed

if strcmp(GPS_DATA, 'ON')                                                   % If simulation of GPS data is required ...
    
    fprintf('NaveGo: simulating GPS data... \n')
    
    gps             = gps_err_profile(ref.lat(1), ref.h(1), gps);           % Transform GPS manufacturer error units to SI units.
    
    [gps]           = gps_gen(ref, gps);                                    % Generate GPS dataset from reference dataset.

    save Data/gps.mat gps
    
else
    
    fprintf('NaveGo: loading GPS data... \n') 
    
    load gps.mat
end

%% 7) SIMULATE IMU

rng('shuffle')                                                              % Reset pseudo-random seed

if strcmp(IMU_DATA, 'ON')                                                   % If simulation of IMU data is required ...
    
    fprintf('NaveGo: simulating IMU ACCR data... \n')
    
    fb                  = acc_gen (ref, IMU);                               % Generate acc in the body frame
    IMU.fb              = fb;
    
    fprintf('NaveGo: simulating IMU GYRO data... \n')
    
    wb                  = gyro_gen (ref, IMU);                              % Generate gyro in the body frame
    IMU.wb              = wb;
    
    save Data/IMU.mat IMU
    
    clear wb fb;
    
else
    fprintf('NaveGo: loading IMU data... \n')
    
    load IMU.mat
end

%% 8) INS/GPS integration using IMU

if strcmp(IMU_INS, 'ON')
    
    fprintf('NaveGo: INS/GPS integration for IMU... \n')
    
    % Sincronize GPS data with IMU data.
    
    % Guarantee that gps.t(1) < IMU.t(1) < gps.t(2)
    if (IMU.t(1) < gps.t(1))
        
        igx         = find(IMU.t > gps.t(1), 1, 'first' );
        
        IMU.t       = IMU.t  (igx:end, :);
        IMU.fb      = IMU.fb (igx:end, :);
        IMU.wb      = IMU.wb (igx:end, :);        
    end
    
    % Guarantee that IMU.t(end-1) < gps.t(end) < IMU.t(end)
    gps1 = gps;
    
    if (IMU.t(end) <= gps.t(end))
        
        fgx  = find(gps.t < IMU.t(end), 1, 'last' );
        
        gps1.t      = gps.t  (1:fgx, :);
        gps1.lat    = gps.lat(1:fgx, :);
        gps1.lon    = gps.lon(1:fgx, :);
        gps1.h      = gps.h  (1:fgx, :);
        gps1.vel    = gps.vel(1:fgx, :);
    end
    
    % Execute INS/GPS integration
    % ---------------------------------------------------------------------
    [IMU_e]        = ins_gps(IMU, gps1, 'quaternion', 'double');
    % ---------------------------------------------------------------------
    
    save Data/IMU_e.mat IMU_e
    
else
    
    fprintf('NaveGo: loading INS/GPS integration for IMU... \n')
    
    load IMU_e.mat
end

%% 9) Interpolate INS/GPS dataset 

% INS/GPS estimates and GPS data are interpolated according to the
% reference dataset.

[IMU_ref, ref_1]    = navego_interpolation (IMU_e, ref);
[gps_ref, ref_g]    = navego_interpolation (gps, ref);

%% 10) Print navigation time

to                	 = (ref.t(end) - ref.t(1));

fprintf('\nNaveGo: navigation time is %.2f minutes or %.2f seconds. \n', (to/60), to)

%% 11) Print RMSE from IMU

print_rmse (IMU_ref, gps_ref, ref_1, ref_g, 'INS/GPS IMU');

%% 12) PLOT

if (strcmp(PLOT,'ON'))
    
    sig3_rr         = abs(IMU_e.Pp(:, 1:22:end).^(0.5)) .* 3;               % Only take diagonal elements from Pp
    
    % TRAJECTORY
    figure;
    plot3(ref.lon.*R2D, ref.lat.*R2D, ref.h)
    hold on
    plot3(ref.lon(1).*R2D, ref.lat(1).*R2D, ref.h(1), 'or', 'MarkerSize', 10, 'LineWidth', 2)
    axis tight
    title('TRAJECTORY')
    xlabel('Longitude [deg.]')
    ylabel('Latitude [deg.]')
    zlabel('Altitude [m]')
    grid
    
    % ATTITUDE
    figure;
    subplot(311)
    plot(ref.t, R2D.*ref.roll, '--k', IMU_e.t, R2D.*IMU_e.roll,'-b');
    xlabel('Time [s]')
	ylabel('[deg]')
    legend('REF', 'IMU');
    title('ROLL');
    
    subplot(312)
    plot(ref.t, R2D.*ref.pitch, '--k', IMU_e.t, R2D.*IMU_e.pitch,'-b');
    xlabel('Time [s]')
	ylabel('[deg]')
    legend('REF', 'IMU');
    title('PITCH');
    
    subplot(313)
    plot(ref.t, R2D.* ref.yaw, '--k', IMU_e.t, R2D.*IMU_e.yaw,'-b');
    xlabel('Time [s]')
	ylabel('[deg]')
    legend('REF', 'IMU');
    title('YAW');
    
    % ATTITUDE ERRORS
    figure;
    subplot(311)
    plot(IMU_e.t, (IMU_ref.roll - ref_1.roll).*R2D, '-b');
    hold on
    plot (gps.t, R2D.*sig3_rr(:,1), '--k', gps.t, -R2D.*sig3_rr(:,1), '--k' )
    xlabel('Time [s]')
    ylabel('[deg]')    
    legend('IMU', '3\sigma');
    title('ROLL ERROR');
    
    subplot(312)
    plot(IMU_e.t, (IMU_ref.pitch - ref_1.pitch).*R2D, '-b');
    hold on
    plot (gps.t, R2D.*sig3_rr(:,2), '--k', gps.t, -R2D.*sig3_rr(:,2), '--k' )
    xlabel('Time [s]')
    ylabel('[deg]')  
    legend('IMU', '3\sigma');
    title('PITCH ERROR');
    
    subplot(313)
    plot(IMU_e.t, (IMU_ref.yaw - ref_1.yaw).*R2D, '-b');
    hold on
    plot (gps.t, R2D.*sig3_rr(:,3), '--k', gps.t, -R2D.*sig3_rr(:,3), '--k' )
    xlabel('Time [s]')
    ylabel('[deg]')  
    legend('IMU', '3\sigma');
    title('YAW ERROR');
    
    % VELOCITIES
    figure;
    subplot(311)
    plot(ref.t, ref.vel(:,1), '--k', gps.t, gps.vel(:,1),'-c', IMU_e.t, IMU_e.vel(:,1),'-b');
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('REF', 'GPS', 'IMU');
    title('NORTH VELOCITY');
    
    subplot(312)
    plot(ref.t, ref.vel(:,2), '--k', gps.t, gps.vel(:,2),'-c', IMU_e.t, IMU_e.vel(:,2),'-b');
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('REF', 'GPS', 'IMU');
    title('EAST VELOCITY');
    
    subplot(313)
    plot(ref.t, ref.vel(:,3), '--k', gps.t, gps.vel(:,3),'-c', IMU_e.t, IMU_e.vel(:,3),'-b');
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('REF', 'GPS', 'IMU');
    title('DOWN VELOCITY');
    
    % VELOCITIES ERRORS
    figure;
    subplot(311)
    plot(gps_ref.t, (gps_ref.vel(:,1) - ref_g.vel(:,1)), '-c');
    hold on
    plot(IMU_ref.t, (IMU_ref.vel(:,1) - ref_1.vel(:,1)), '-b');
    plot (gps.t, sig3_rr(:,4), '--k', gps.t, -sig3_rr(:,4), '--k' )
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('GPS', 'IMU', '3\sigma');
    title('VELOCITY NORTH ERROR');
    
    subplot(312)
    plot(gps_ref.t, (gps_ref.vel(:,2) - ref_g.vel(:,2)), '-c');
    hold on
    plot(IMU_ref.t, (IMU_ref.vel(:,2) - ref_1.vel(:,2)), '-b');
    plot (gps.t, sig3_rr(:,5), '--k', gps.t, -sig3_rr(:,5), '--k' )
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('GPS', 'IMU', '3\sigma');
    title('VELOCITY EAST ERROR');
    
    subplot(313)
    plot(gps_ref.t, (gps_ref.vel(:,3) - ref_g.vel(:,3)), '-c');
    hold on
    plot(IMU_ref.t, (IMU_ref.vel(:,3) - ref_1.vel(:,3)), '-b');
    plot (gps.t, sig3_rr(:,6), '--k', gps.t, -sig3_rr(:,6), '--k' )
    xlabel('Time [s]')
    ylabel('[m/s]')
    legend('GPS', 'IMU', '3\sigma');
    title('VELOCITY DOWN ERROR');
    
    % POSITION
    figure;
    subplot(311)
    plot(ref.t, ref.lat .*R2D, '--k', gps.t, gps.lat.*R2D, '-c', IMU_e.t, IMU_e.lat.*R2D, '-b');
    xlabel('Time [s]')
    ylabel('[deg]')
    legend('REF', 'GPS', 'IMU');
    title('LATITUDE');
    
    subplot(312)
    plot(ref.t, ref.lon .*R2D, '--k', gps.t, gps.lon.*R2D, '-c', IMU_e.t, IMU_e.lon.*R2D, '-b');
    xlabel('Time [s]')
    ylabel('[deg]')
    legend('REF', 'GPS', 'IMU');
    title('LONGITUDE');
    
    subplot(313)
    plot(ref.t, ref.h, '--k', gps.t, gps.h, '-c', IMU_e.t, IMU_e.h, '-b');
    xlabel('Time [s]')
    ylabel('[m]')
    legend('REF', 'GPS', 'IMU');
    title('ALTITUDE');
    
    % POSITION ERRORS    
    [RN,RE]  = radius(IMU_ref.lat, 'double');
    LAT2M_1 = RN + IMU_ref.h;
    LON2M_1 = (RE + IMU_ref.h).*cos(IMU_ref.lat);
    
    [RN,RE]  = radius(gps.lat, 'double');
    LAT2M_G = RN + gps.h;
    LON2M_G = (RE + gps.h).*cos(gps.lat);
    
    [RN,RE]  = radius(gps_ref.lat, 'double');
    LAT2M_GR = RN + gps_ref.h;
    LON2M_GR = (RE + gps_ref.h).*cos(gps_ref.lat);
    
    figure;
    subplot(311)
    plot(gps_ref.t,  LAT2M_GR.*(gps_ref.lat - ref_g.lat), '-c')
    hold on
    plot(IMU_ref.t, LAT2M_1.*(IMU_ref.lat - ref_1.lat), '-b')
    plot (gps.t, LAT2M_G.*sig3_rr(:,7), '--k', gps.t, -LAT2M_G.*sig3_rr(:,7), '--k' )
    xlabel('Time [s]')
    ylabel('[m]')
    legend('GPS', 'IMU', '3\sigma');
    title('LATITUDE ERROR');
    
    subplot(312)
    plot(gps_ref.t, LON2M_GR.*(gps_ref.lon - ref_g.lon), '-c')
    hold on
    plot(IMU_ref.t, LON2M_1.*(IMU_ref.lon - ref_1.lon), '-b')
    plot(gps.t, LON2M_G.*sig3_rr(:,8), '--k', gps.t, -LON2M_G.*sig3_rr(:,8), '--k' )
    xlabel('Time [s]')
    ylabel('[m]')
    legend('GPS', 'IMU', '3\sigma');
    title('LONGITUDE ERROR');
    
    subplot(313)
    plot(gps_ref.t, (gps_ref.h - ref_g.h), '-c')
    hold on
    plot(IMU_ref.t, (IMU_ref.h - ref_1.h), '-b')
    plot(gps.t, sig3_rr(:,9), '--k', gps.t, -sig3_rr(:,9), '--k' )
    xlabel('Time [s]')
    ylabel('[m]')
    legend('GPS', 'IMU', '3\sigma');
    title('ALTITUDE ERROR');    
end