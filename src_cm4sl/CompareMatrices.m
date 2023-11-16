clear; clc;
load lti_model;

vxRef = 80/3.6;
Ts = 0.05;
[Ad, Bd, Bd1] = GetMatrices(vxRef, Ts); % find matrices

