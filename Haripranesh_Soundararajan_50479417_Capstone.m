
clc;
close all;
clear all;

%% Define system parameters
m1 = 500;
m2 = 120;
k1 = 50000;
k2 = 300000;
b1 = 150;
b2 = 8000;

%% Transfer function 
nump = [m1 + m2, b2, k2];
denp = [m1 * m2, m1 * (b1 + b2) + m2 * b1, m1 * (k1 + k2) + m2 * k1 + b1 * b2, b1 * k2 + b2 * k1, k1 * k2];
G1 = tf(nump, denp);
num1 = [-m1 * b2, -m1 * k2, 0, 0];
den1 = [m1 * m2, m1 * (b1 + b2) + m2 * b1, m1 * (k1 + k2) + m2 * k1 + b1 * b2, b1 * k2 + b2 * k1, k1 * k2];
G2 = tf(num1, den1);
numf = num1;
denf = nump;
F = tf(numf, denf);

%% PID controller 

t = 0:0.05:5;
Kd = 416050;
Kp = 1664200;
Ki = 1248150;
C = pid(Kp, Ki, Kd);
sys_cl = F * feedback(G1, C);

%% Ploting the response 
figure;
step(0.1 * sys_cl, t);
title('Response to a 0.1-m Step with High-Gain PID');
