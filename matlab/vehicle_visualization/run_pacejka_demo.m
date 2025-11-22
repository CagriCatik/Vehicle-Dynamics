% --------------------------------------------------------------------
%  File: run_pacejka_demo.m
% --------------------------------------------------------------------
clear; close all; clc;

% 1) Load the class
tire = PacejkaTire();    % or pass a custom params struct

% 2) Pick a slip state
demo.kappa  = 0.10;
demo.alpha  = deg2rad(6);
demo.gamma  = 0;
demo.Fz     = 4000;

% 3) Compute forces and moment
Fx0  = tire.calcFx0(demo.kappa,  demo.Fz);
Fy0  = tire.calcFy0(demo.alpha, demo.Fz, demo.gamma);
Fx   = tire.calcFx(demo.kappa, demo.alpha, demo.Fz, demo.gamma);
Fy   = tire.calcFy(demo.alpha, demo.kappa, demo.Fz, demo.gamma);
Mz   = tire.calcMz(demo.alpha, demo.kappa, demo.Fz, demo.gamma);

fprintf('Fx0=%.1f N, Fy0=%.1f N, Fx=%.1f N, Fy=%.1f N, Mz=%.1f N*m\n', ...
        Fx0, Fy0, Fx, Fy, Mz);

% 4) Plot
fig = tire.plotPureCurves();      % method that wraps all the tiled layout logic
tire.exportFigures('output');     % helper that calls exportgraphics