clc;
clear all;
close all;

%% Lets define the variable here
%% For Generator
U_peak= 60 ;
f_peak= 60 ;
t_ramp = 0.5;
Ls 		= 1365e-6
Rs 		= 0.416
Psi_f 	= 0.166
J_total 	= 340e-6
p 			= 2
T_nom = 10
T_max = 20
n_nom = 1800
rad_per_sec2rpm = 60/(2*pi)
rpm2rad_per_sec = 2*pi/60
U_max = 60
f_max = 60