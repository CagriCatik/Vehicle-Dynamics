% Pacejka-style tire model with black background

% 1) Load-sensitive pure-slip Magic Formula for Fx(kappa) and Fy(alpha,gamma)
% 2) Simple combined-slip weighting: Fx = Fx0*Gx(alpha), Fy = Fy0*Gy(kappa)
% 3) Camber effect (shifts) and aligning moment Mz = -t * Fy
% 4) Figures use a black theme and export with black background

clear; close all; clc;

%% Black theme defaults
set(0, 'DefaultFigureColor', [0 0 0]);
set(0, 'DefaultAxesColor', [0 0 0], ...
       'DefaultAxesXColor', [1 1 1], ...
       'DefaultAxesYColor', [1 1 1], ...
       'DefaultAxesZColor', [1 1 1], ...
       'DefaultAxesGridColor', [0.28 0.28 0.28], ...
       'DefaultAxesMinorGridColor', [0.18 0.18 0.18]);
set(0, 'DefaultTextColor', [1 1 1]);
set(0, 'DefaultLineLineWidth', 1.5);

%% Vehicle and operating point (passenger car scale)
veh.m = 1500;           % kg
veh.g = 9.81;           % m/s^2
veh.static_wdist = 0.6; % front axle fraction
axle.Fz_front = (veh.static_wdist*veh.m*veh.g)/2;  % per-tire load [N]
axle.Fz_rear  = ((1-veh.static_wdist)*veh.m*veh.g)/2;
Fz_nom = 4000;          % nominal per-tire load used to define parameters [N]

%% Tire parameters (Magic Formula style, illustrative)
% Longitudinal pure-slip
par.mu_x0 = 1.00;                   % peak friction at Fz_nom
par.Cx    = 1.65;                   % shape factor
par.Ex    = 0.97;                   % curvature
par.Kx0   = 110e3;                  % small-slip stiffness at Fz_nom [N]
par.Kx_exp = 0.8;                   % load sensitivity exponent for Kx
par.mu_x_load_slope = -0.05;        % mu_x(Fz) = mu_x0*(1 + slope*(Fz/Fz_nom - 1))

% Lateral pure-slip
par.mu_y0 = 0.95;
par.Cy    = 1.30;
par.Ey    = 1.00;
par.Ca0   = 80e3;                   % cornering stiffness at Fz_nom [N/rad]
par.Ca_exp = 0.9;                   % load sensitivity exponent for Ca
par.mu_y_load_slope = -0.06;

% Camber influence (shifts)
par.Sh_y_gamma = 0.015;             % horizontal shift per rad of camber [rad/rad]
par.Sv_y_gamma = 0.0;               % vertical shift gain per rad of camber times Fz [N/rad]

% Combined-slip weighting
par.Cgx  = 1.1;                     % Fx reduction vs alpha
par.Cgy  = 1.1;                     % Fy reduction vs kappa
par.Bgx0 = 6.0;                     % B for Gx(alpha) at Fz_nom
par.Bgy0 = 8.0;                     % B for Gy(kappa) at Fz_nom
par.Bg_exp = 0.6;                   % decrease B with load (heavier = smaller B)

% Pneumatic trail model for Mz = -t * Fy
par.t0   = 0.15;                    % nominal trail at small slip and Fz_nom [m]
par.t_exp = 0.6;                    % load sensitivity exponent for trail
par.Bt0  = 5.0;                     % trail drop-off sensitivity vs alpha
par.Ct   = 1.1;                     % trail shape factor
par.Gt_kappa = 1.0;                 % trail reduction vs kappa weight
par.Btk0 = 5.0;                     % trail reduction vs kappa sensitivity

%% Domains
dom.kappa = linspace(-0.25, 0.25, 801);               % longitudinal slip ratio
dom.alpha = deg2rad(linspace(-15, 15, 801));          % slip angle [rad]
dom.gamma_list = deg2rad([0, -2, -4]);                % camber angles [rad]
dom.loads = [3000, 4000, 5000];                       % per-tire Fz [N]
dom.Fz_for_surfaces = Fz_nom;                         % surface plots at nominal Fz

%% Load sensitivity helpers
mu_x = @(Fz) par.mu_x0 .* (1 + par.mu_x_load_slope*((Fz./Fz_nom) - 1));
mu_y = @(Fz) par.mu_y0 .* (1 + par.mu_y_load_slope*((Fz./Fz_nom) - 1));
Kx_small = @(Fz) par.Kx0 .* (Fz./Fz_nom) .^ par.Kx_exp;      % [N]
Ca_small = @(Fz) par.Ca0 .* (Fz./Fz_nom) .^ par.Ca_exp;      % [N/rad]

Bgx = @(Fz) max(0.1, par.Bgx0 .* (Fz_nom./Fz) .^ par.Bg_exp);
Bgy = @(Fz) max(0.1, par.Bgy0 .* (Fz_nom./Fz) .^ par.Bg_exp);

trail0 = @(Fz) par.t0 .* (Fz./Fz_nom) .^ par.t_exp;          % [m]
Bt = @(~) par.Bt0;
Btk = @(~) par.Btk0;

%% Magic Formula with optional shifts
% y(x) = D * sin( C * atan( B*(x + Sh) - E*(B*(x + Sh) - atan(B*(x + Sh))) ) ) + Sv
mf = @(x,B,C,D,E,Sh,Sv) D .* sin( C .* atan( B.*(x + Sh) - E.*(B.*(x + Sh) - atan(B.*(x + Sh))) ) ) + Sv;

%% Pure-slip forces
make_Fx0 = @(kappa,Fz) ...
    mf(kappa, ...
       Kx_small(Fz) ./ (par.Cx .* (mu_x(Fz).*Fz)), ...  % Bx
       par.Cx, ...
       mu_x(Fz).*Fz, ...                                % Dx
       par.Ex, ...
       0.0, 0.0);

make_Fy0 = @(alpha,Fz,gamma) ...
    mf(alpha, ...
       Ca_small(Fz) ./ (par.Cy .* (mu_y(Fz).*Fz)), ...  % By
       par.Cy, ...
       mu_y(Fz).*Fz, ...                                % Dy
       par.Ey, ...
       par.Sh_y_gamma .* gamma, ...
       par.Sv_y_gamma .* gamma .* (Fz./Fz_nom) .* Fz);

%% Combined slip weights and forces
Gx = @(alpha,Fz) cos( par.Cgx .* atan( Bgx(Fz) .* alpha ) );
Gy = @(kappa,Fz) cos( par.Cgy .* atan( Bgy(Fz) .* kappa ) );

make_Fx = @(kappa,alpha,Fz,gamma) make_Fx0(kappa,Fz) .* Gx(alpha,Fz);
make_Fy = @(alpha,kappa,Fz,gamma) make_Fy0(alpha,Fz,gamma) .* Gy(kappa,Fz);

%% Pneumatic trail and aligning moment
trail_pure = @(alpha,Fz) trail0(Fz) .* cos( par.Ct .* atan( Bt(Fz) .* alpha ) );
trail_comb = @(alpha,kappa,Fz) trail_pure(alpha,Fz) .* cos( par.Ct .* atan( par.Gt_kappa .* Btk(Fz) .* kappa ) );
make_Mz = @(alpha,kappa,Fz,gamma) - trail_comb(alpha,kappa,Fz) .* make_Fy(alpha,kappa,Fz,gamma);

%% Figure 1: Pure curves
fig1 = figure('Color',[0 0 0], 'InvertHardcopy','off');
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');

ax1 = nexttile; hold(ax1,'on'); grid(ax1,'on'); box(ax1,'on');
for Fz = dom.loads
    Fx0 = make_Fx0(dom.kappa, Fz);
    plot(ax1, dom.kappa, Fx0, 'DisplayName', sprintf('Fz = %.0f N', Fz));
end
xlabel(ax1,'Kappa (-)'); ylabel(ax1,'Fx0 (N)'); title(ax1,'Pure longitudinal Fx0(kappa)');
style_dark(ax1);
lg = legend(ax1,'Location','SouthEast'); style_legend_dark(lg);

ax2 = nexttile; hold(ax2,'on'); grid(ax2,'on'); box(ax2,'on');
for Fz = dom.loads
    Fy0 = make_Fy0(dom.alpha, Fz, 0.0);
    plot(ax2, rad2deg(dom.alpha), Fy0, 'DisplayName', sprintf('Fz = %.0f N', Fz));
end
xlabel(ax2,'Alpha (deg)'); ylabel(ax2,'Fy0 (N)'); title(ax2,'Pure lateral Fy0(alpha), gamma = 0 deg');
style_dark(ax2);
lg = legend(ax2,'Location','SouthEast'); style_legend_dark(lg);

ax3 = nexttile; hold(ax3,'on'); grid(ax3,'on'); box(ax3,'on');
for gamma = dom.gamma_list
    Fy0g = make_Fy0(dom.alpha, Fz_nom, gamma);
    plot(ax3, rad2deg(dom.alpha), Fy0g, 'DisplayName', sprintf('gamma = %+g deg', rad2deg(gamma)));
end
xlabel(ax3,'Alpha (deg)'); ylabel(ax3,'Fy0 (N)'); title(ax3,'Fy0(alpha) vs camber at Fz = 4000 N');
style_dark(ax3);
lg = legend(ax3,'Location','SouthEast'); style_legend_dark(lg);

ax4 = nexttile; hold(ax4,'on'); grid(ax4,'on'); box(ax4,'on');
Mz0 = make_Mz(dom.alpha, 0.0, Fz_nom, 0.0);
plot(ax4, rad2deg(dom.alpha), Mz0);
xlabel(ax4,'Alpha (deg)'); ylabel(ax4,'Mz (N*m)'); title(ax4,'Aligning moment Mz(alpha), Fz = 4000 N, kappa = 0, gamma = 0');
style_dark(ax4);

%% Figure 2: Combined-slip slices
fig2 = figure('Color',[0 0 0], 'InvertHardcopy','off');
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');

ax5 = nexttile; hold(ax5,'on'); grid(ax5,'on'); box(ax5,'on');
alpha_slices_deg = [-10, 0, 10];
Fz_plot = Fz_nom;
for a_deg = alpha_slices_deg
    a = deg2rad(a_deg);
    Fx_comb = make_Fx(dom.kappa, a, Fz_plot, 0.0);
    plot(ax5, dom.kappa, Fx_comb, 'DisplayName', sprintf('alpha = %+g deg', a_deg));
end
xlabel(ax5,'Kappa (-)'); ylabel(ax5,'Fx (N)'); title(ax5,sprintf('Fx(kappa) at Fz = %.0f N', Fz_plot));
style_dark(ax5);
lg = legend(ax5,'Location','SouthEast'); style_legend_dark(lg);

ax6 = nexttile; hold(ax6,'on'); grid(ax6,'on'); box(ax6,'on');
kappa_slices = [-0.12, 0, 0.12];
for k = kappa_slices
    Fy_comb = make_Fy(dom.alpha, k, Fz_plot, 0.0);
    plot(ax6, rad2deg(dom.alpha), Fy_comb, 'DisplayName', sprintf('kappa = %+0.3f', k));
end
xlabel(ax6,'Alpha (deg)'); ylabel(ax6,'Fy (N)'); title(ax6,sprintf('Fy(alpha) at Fz = %.0f N', Fz_plot));
style_dark(ax6);
lg = legend(ax6,'Location','SouthEast'); style_legend_dark(lg);

ax7 = nexttile; hold(ax7,'on'); grid(ax7,'on'); box(ax7,'on');
for k = kappa_slices
    Mz_comb = make_Mz(dom.alpha, k, Fz_plot, 0.0);
    plot(ax7, rad2deg(dom.alpha), Mz_comb, 'DisplayName', sprintf('kappa = %+0.3f', k));
end
xlabel(ax7,'Alpha (deg)'); ylabel(ax7,'Mz (N*m)'); title(ax7,sprintf('Mz(alpha) at Fz = %.0f N', Fz_plot));
style_dark(ax7);
lg = legend(ax7,'Location','SouthEast'); style_legend_dark(lg);

ax8 = nexttile; hold(ax8,'on'); grid(ax8,'on'); box(ax8,'on');
[Kmesh, Amesh] = meshgrid(linspace(-0.2,0.2,141), deg2rad(linspace(-12,12,141)));
Fxm = make_Fx(Kmesh, Amesh, Fz_plot, 0.0);
Fym = make_Fy(Amesh, Kmesh, Fz_plot, 0.0);
util = sqrt( (Fxm./(mu_x(Fz_plot).*Fz_plot)).^2 + (Fym./(mu_y(Fz_plot).*Fz_plot)).^2 );
contourf(ax8, Kmesh, rad2deg(Amesh), util, 0.2:0.1:1.6, 'LineStyle','none');
cb = colorbar(ax8); cb.Color = [1 1 1];
clim(ax8,[0 1.6]);
contour(ax8, Kmesh, rad2deg(Amesh), util, [1 1], 'w', 'LineWidth', 1.5);
xlabel(ax8,'Kappa (-)'); ylabel(ax8,'Alpha (deg)'); title(ax8,'Friction ellipse utilization (1.0 contour)');
style_dark(ax8);

%% Figure 3: 3D surfaces at nominal load
fig3 = figure('Color',[0 0 0], 'InvertHardcopy','off');
tiledlayout(1,2,'Padding','compact','TileSpacing','compact');

ax9 = nexttile; hold(ax9,'on'); grid(ax9,'on'); box(ax9,'on');
[Kmesh2, Amesh2] = meshgrid(linspace(-0.2,0.2,101), deg2rad(linspace(-12,12,101)));
Fx_surf = make_Fx(Kmesh2, Amesh2, Fz_nom, 0.0);
surf(ax9, Kmesh2, rad2deg(Amesh2), Fx_surf, 'EdgeColor','none');
view(ax9, 135, 25);
xlabel(ax9,'Kappa (-)'); ylabel(ax9,'Alpha (deg)'); zlabel(ax9,'Fx (N)');
title(ax9,'Fx(kappa, alpha), Fz = 4000 N');
cb = colorbar(ax9); cb.Color = [1 1 1];
style_dark(ax9);

ax10 = nexttile; hold(ax10,'on'); grid(ax10,'on'); box(ax10,'on');
Fy_surf = make_Fy(Amesh2, Kmesh2, Fz_nom, 0.0);
surf(ax10, Kmesh2, rad2deg(Amesh2), Fy_surf, 'EdgeColor','none');
view(ax10, 135, 25);
xlabel(ax10,'Kappa (-)'); ylabel(ax10,'Alpha (deg)'); zlabel(ax10,'Fy (N)');
title(ax10,'Fy(alpha, kappa), Fz = 4000 N');
cb = colorbar(ax10); cb.Color = [1 1 1];
style_dark(ax10);

drawnow;

%% Export with black background
outDir = fullfile(pwd,'output');
if ~exist(outDir,'dir'), mkdir(outDir); end
exportgraphics(fig1, fullfile(outDir,'pacejka_pure_black.png'), 'Resolution', 300, 'BackgroundColor','current');
exportgraphics(fig2, fullfile(outDir,'pacejka_combined_black.png'), 'Resolution', 300, 'BackgroundColor','current');
exportgraphics(fig3, fullfile(outDir,'pacejka_surfaces_black.png'), 'Resolution', 300, 'BackgroundColor','current');

%% Demo printout
demo.kappa = 0.10;
demo.alpha = deg2rad(6);
Fx0_demo = make_Fx0(demo.kappa, Fz_nom);
Fy0_demo = make_Fy0(demo.alpha, Fz_nom, 0.0);
Fx_demo  = make_Fx(demo.kappa, demo.alpha, Fz_nom, 0.0);
Fy_demo  = make_Fy(demo.alpha, demo.kappa, Fz_nom, 0.0);
Mz_demo  = make_Mz(demo.alpha, demo.kappa, Fz_nom, 0.0);
fprintf('Fz=%.0f N, kappa=%.3f, alpha=%+.1f deg -> Fx0=%.1f N, Fy0=%.1f N, Fx=%.1f N, Fy=%.1f N, Mz=%.1f N*m\n', ...
    Fz_nom, demo.kappa, rad2deg(demo.alpha), Fx0_demo, Fy0_demo, Fx_demo, Fy_demo, Mz_demo);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Helpers

function style_dark(ax)
% Apply black theme to a single axes
set(ax, 'Color', [0 0 0], ...
        'XColor', [1 1 1], 'YColor', [1 1 1], 'ZColor', [1 1 1], ...
        'GridColor', [0.28 0.28 0.28], 'MinorGridColor', [0.18 0.18 0.18], ...
        'Box', 'on');
ax.Title.Color = [1 1 1];
ax.XLabel.Color = [1 1 1];
ax.YLabel.Color = [1 1 1];
if ~isempty(ax.ZLabel) && isprop(ax.ZLabel,'Color')
    ax.ZLabel.Color = [1 1 1];
end
end

function style_legend_dark(lg)
% Style legend for dark background
set(lg, 'Color', 'none', 'EdgeColor', [1 1 1]);
if isprop(lg,'TextColor')
    lg.TextColor = [1 1 1];
end
end
