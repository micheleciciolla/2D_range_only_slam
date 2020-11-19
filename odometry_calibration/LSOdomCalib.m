close all
clear
clc

addpath '../'
addpath 'tools/g2o_wrapper'
addpath 'tools/visualization'
source "tools/utilities/geometry_helpers_2d.m"
addpath 'datasets'
addpath './getIDs_and_index.m'
addpath './get_formatted_index.m'
addpath './initial_guess_eval.m'
#import 2d geometry utils
% source "../tools/utilities/geometry_helpers_2d.m"
source "odometry_calibration/ls_calibrate_odometry.m"

h = figure(1);

more off;
#load the calibration matrix
disp('loading the dataset');
% loading both datasets
% Z=load("./ls_calib.dat");

Z = get_Z();

#compute the ground truth trajectory
TrueTrajectory=compute_odometry_trajectory(Z(:,1:3));
disp('ground truth');
hold on;
plot(TrueTrajectory(:,1),TrueTrajectory(:,2), 'r-', 'linewidth', 2);
pause(1);

#compute the uncalibrated odometry
OdomTrajectory=compute_odometry_trajectory(Z(:,4:6));
disp('odometry');
hold on;
plot(OdomTrajectory(:,1),OdomTrajectory(:,2), 'g-', 'linewidth', 2);
pause(1);

disp('computing calibration parameters');
#compute the calibration parameters
X=ls_calibrate_odometry(Z);
disp(X);
pause(1);

disp('computing calibrated odometry');
COdom=apply_odometry_correction(X,Z(:,4:6));
CalTrajectory=compute_odometry_trajectory(COdom);
hold on;
plot(CalTrajectory(:,1),CalTrajectory(:,2), 'b-', 'linewidth', 2);
