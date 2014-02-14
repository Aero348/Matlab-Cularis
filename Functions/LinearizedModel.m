% This code will be used to simulate the linearized model for turning 
% steady flight. It has been written for the Winter 2014 section of Aero 
% 348, and will use data that has been taken from a Multiplex Cularis. 
% Data such as C_M, C_L, C_D has been calculated via %Digital Datcom. 
% Written by Christopher Frasson and Dillon Hawley
% Last updated 14 February, 2013

% DO NOT ASSUME: phi0 = w0 = alpha0 = theta0 = 0

%% Linearized Model for Turning Study Flight

function LinearizedModel()
g = 9.80665 % gravitational acceleraion [m/s^2]

m      = % plane mass [kg]
q0     = % [rad/s]   
w0     = %
r0     = % [rad/s]
v0     = %
D0     = %
alpha0 = %[rad]
beta0  = %
L0     = %
T0     = %

m(q0*w0-r0*v0) = -m*g*sin(theta0)-D0*cos(alpha0)*cos(beta0) + L0*sin(

end