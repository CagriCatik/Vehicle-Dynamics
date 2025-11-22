# Pacejka-style tire model with black background
#
# 1) Load-sensitive pure-slip Magic Formula for Fx(kappa) and Fy(alpha,gamma)
# 2) Simple combined-slip weighting: Fx = Fx0*Gx(alpha), Fy = Fy0*Gy(kappa)
# 3) Camber effect (shifts) and aligning moment Mz = -t * Fy
# 4) Figures use a black theme and export with black background

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import os

# Black theme defaults
plt.rcParams["figure.facecolor"] = (0.0, 0.0, 0.0)
plt.rcParams["axes.facecolor"] = (0.0, 0.0, 0.0)
plt.rcParams["axes.edgecolor"] = (1.0, 1.0, 1.0)
plt.rcParams["axes.labelcolor"] = (1.0, 1.0, 1.0)
plt.rcParams["axes.grid"] = True
plt.rcParams["grid.color"] = (0.28, 0.28, 0.28)
plt.rcParams["xtick.color"] = (1.0, 1.0, 1.0)
plt.rcParams["ytick.color"] = (1.0, 1.0, 1.0)
plt.rcParams["text.color"] = (1.0, 1.0, 1.0)
plt.rcParams["lines.linewidth"] = 1.5

deg2rad = np.deg2rad
rad2deg = np.rad2deg

# Vehicle and operating point (passenger car scale)
veh = {}
veh["m"] = 1500.0           # kg
veh["g"] = 9.81             # m/s^2
veh["static_wdist"] = 0.6   # front axle fraction

axle = {}
axle["Fz_front"] = (veh["static_wdist"] * veh["m"] * veh["g"]) / 2.0  # per-tire load [N]
axle["Fz_rear"] = ((1.0 - veh["static_wdist"]) * veh["m"] * veh["g"]) / 2.0
Fz_nom = 4000.0             # nominal per-tire load used to define parameters [N]

# Tire parameters (Magic Formula style, illustrative)
par = {}

# Longitudinal pure-slip
par["mu_x0"] = 1.00                    # peak friction at Fz_nom
par["Cx"] = 1.65                       # shape factor
par["Ex"] = 0.97                       # curvature
par["Kx0"] = 110e3                     # small-slip stiffness at Fz_nom [N]
par["Kx_exp"] = 0.8                    # load sensitivity exponent for Kx
par["mu_x_load_slope"] = -0.05         # mu_x(Fz) = mu_x0*(1 + slope*(Fz/Fz_nom - 1))

# Lateral pure-slip
par["mu_y0"] = 0.95
par["Cy"] = 1.30
par["Ey"] = 1.00
par["Ca0"] = 80e3                      # cornering stiffness at Fz_nom [N/rad]
par["Ca_exp"] = 0.9                    # load sensitivity exponent for Ca
par["mu_y_load_slope"] = -0.06

# Camber influence (shifts)
par["Sh_y_gamma"] = 0.015              # horizontal shift per rad of camber [rad/rad]
par["Sv_y_gamma"] = 0.0                # vertical shift gain per rad of camber times Fz [N/rad]

# Combined-slip weighting
par["Cgx"] = 1.1                       # Fx reduction vs alpha
par["Cgy"] = 1.1                       # Fy reduction vs kappa
par["Bgx0"] = 6.0                      # B for Gx(alpha) at Fz_nom
par["Bgy0"] = 8.0                      # B for Gy(kappa) at Fz_nom
par["Bg_exp"] = 0.6                    # decrease B with load (heavier = smaller B)

# Pneumatic trail model for Mz = -t * Fy
par["t0"] = 0.15                       # nominal trail at small slip and Fz_nom [m]
par["t_exp"] = 0.6                     # load sensitivity exponent for trail
par["Bt0"] = 5.0                       # trail drop-off sensitivity vs alpha
par["Ct"] = 1.1                        # trail shape factor
par["Gt_kappa"] = 1.0                  # trail reduction vs kappa weight
par["Btk0"] = 5.0                      # trail reduction vs kappa sensitivity

# Domains
dom = {}
dom["kappa"] = np.linspace(-0.25, 0.25, 801)               # longitudinal slip ratio
dom["alpha"] = deg2rad(np.linspace(-15.0, 15.0, 801))      # slip angle [rad]
dom["gamma_list"] = deg2rad(np.array([0.0, -2.0, -4.0]))   # camber angles [rad]
dom["loads"] = np.array([3000.0, 4000.0, 5000.0])          # per-tire Fz [N]
dom["Fz_for_surfaces"] = Fz_nom                            # surface plots at nominal Fz

# Load sensitivity helpers
mu_x = lambda Fz: par["mu_x0"] * (1.0 + par["mu_x_load_slope"] * ((np.asarray(Fz) / Fz_nom) - 1.0))
mu_y = lambda Fz: par["mu_y0"] * (1.0 + par["mu_y_load_slope"] * ((np.asarray(Fz) / Fz_nom) - 1.0))
Kx_small = lambda Fz: par["Kx0"] * (np.asarray(Fz) / Fz_nom) ** par["Kx_exp"]      # [N]
Ca_small = lambda Fz: par["Ca0"] * (np.asarray(Fz) / Fz_nom) ** par["Ca_exp"]      # [N/rad]

Bgx = lambda Fz: np.maximum(0.1, par["Bgx0"] * (Fz_nom / np.asarray(Fz)) ** par["Bg_exp"])
Bgy = lambda Fz: np.maximum(0.1, par["Bgy0"] * (Fz_nom / np.asarray(Fz)) ** par["Bg_exp"])

trail0 = lambda Fz: par["t0"] * (np.asarray(Fz) / Fz_nom) ** par["t_exp"]          # [m]
Bt = lambda Fz: par["Bt0"]
Btk = lambda Fz: par["Btk0"]

# Magic Formula with optional shifts
# y(x) = D * sin( C * atan( B*(x + Sh) - E*(B*(x + Sh) - atan(B*(x + Sh))) ) ) + Sv
def mf(x, B, C, D, E, Sh, Sv):
    x = np.asarray(x)
    B = np.asarray(B)
    C = np.asarray(C)
    D = np.asarray(D)
    E = np.asarray(E)
    Sh = np.asarray(Sh)
    Sv = np.asarray(Sv)
    x_shifted = x + Sh
    Bx = B * x_shifted
    inner = Bx - E * (Bx - np.arctan(Bx))
    return D * np.sin(C * np.arctan(inner)) + Sv

# Pure-slip forces
def make_Fx0(kappa, Fz):
    Fz_arr = np.asarray(Fz)
    Bx = Kx_small(Fz_arr) / (par["Cx"] * (mu_x(Fz_arr) * Fz_arr))
    Dx = mu_x(Fz_arr) * Fz_arr
    return mf(kappa, Bx, par["Cx"], Dx, par["Ex"], 0.0, 0.0)

def make_Fy0(alpha, Fz, gamma):
    Fz_arr = np.asarray(Fz)
    gamma_arr = np.asarray(gamma)
    By = Ca_small(Fz_arr) / (par["Cy"] * (mu_y(Fz_arr) * Fz_arr))
    Dy = mu_y(Fz_arr) * Fz_arr
    Sh = par["Sh_y_gamma"] * gamma_arr
    Sv = par["Sv_y_gamma"] * gamma_arr * (Fz_arr / Fz_nom) * Fz_arr
    return mf(alpha, By, par["Cy"], Dy, par["Ey"], Sh, Sv)

# Combined slip weights and forces
Gx = lambda alpha, Fz: np.cos(par["Cgx"] * np.arctan(Bgx(Fz) * np.asarray(alpha)))
Gy = lambda kappa, Fz: np.cos(par["Cgy"] * np.arctan(Bgy(Fz) * np.asarray(kappa)))

def make_Fx(kappa, alpha, Fz, gamma):
    return make_Fx0(kappa, Fz) * Gx(alpha, Fz)

def make_Fy(alpha, kappa, Fz, gamma):
    return make_Fy0(alpha, Fz, gamma) * Gy(kappa, Fz)

# Pneumatic trail and aligning moment
def trail_pure(alpha, Fz):
    Fz_arr = np.asarray(Fz)
    return trail0(Fz_arr) * np.cos(par["Ct"] * np.arctan(Bt(Fz_arr) * np.asarray(alpha)))

def trail_comb(alpha, kappa, Fz):
    Fz_arr = np.asarray(Fz)
    return trail_pure(alpha, Fz_arr) * np.cos(par["Ct"] * np.arctan(par["Gt_kappa"] * Btk(Fz_arr) * np.asarray(kappa)))

def make_Mz(alpha, kappa, Fz, gamma):
    Fy_val = make_Fy(alpha, kappa, Fz, gamma)
    return -trail_comb(alpha, kappa, Fz) * Fy_val

# Helper functions for styling
def style_dark(ax):
    ax.set_facecolor((0.0, 0.0, 0.0))
    ax.tick_params(colors=(1.0, 1.0, 1.0))
    for spine in ax.spines.values():
        spine.set_edgecolor((1.0, 1.0, 1.0))
    ax.grid(True, color=(0.28, 0.28, 0.28))
    ax_title = ax.title
    ax_xlabel = ax.xaxis.label
    ax_ylabel = ax.yaxis.label
    ax_title.set_color((1.0, 1.0, 1.0))
    ax_xlabel.set_color((1.0, 1.0, 1.0))
    ax_ylabel.set_color((1.0, 1.0, 1.0))
    if hasattr(ax, "zaxis"):
        ax.zaxis.label.set_color((1.0, 1.0, 1.0))
        ax.zaxis.set_tick_params(colors=(1.0, 1.0, 1.0))

def style_legend_dark(lg):
    if lg is None:
        return
    frame = lg.get_frame()
    frame.set_facecolor("none")
    frame.set_edgecolor((1.0, 1.0, 1.0))
    for text in lg.get_texts():
        text.set_color((1.0, 1.0, 1.0))

# Figure 1: Pure curves
fig1, axs1 = plt.subplots(2, 2, figsize=(10, 8), constrained_layout=True)
fig1.patch.set_facecolor((0.0, 0.0, 0.0))

ax1 = axs1[0, 0]
ax1.grid(True)
for Fz in dom["loads"]:
    Fx0 = make_Fx0(dom["kappa"], Fz)
    ax1.plot(dom["kappa"], Fx0, label=f"Fz = {Fz:.0f} N")
ax1.set_xlabel("Kappa (-)")
ax1.set_ylabel("Fx0 (N)")
ax1.set_title("Pure longitudinal Fx0(kappa)")
style_dark(ax1)
lg = ax1.legend(loc="lower right")
style_legend_dark(lg)

ax2 = axs1[0, 1]
ax2.grid(True)
for Fz in dom["loads"]:
    Fy0 = make_Fy0(dom["alpha"], Fz, 0.0)
    ax2.plot(rad2deg(dom["alpha"]), Fy0, label=f"Fz = {Fz:.0f} N")
ax2.set_xlabel("Alpha (deg)")
ax2.set_ylabel("Fy0 (N)")
ax2.set_title("Pure lateral Fy0(alpha), gamma = 0 deg")
style_dark(ax2)
lg = ax2.legend(loc="lower right")
style_legend_dark(lg)

ax3 = axs1[1, 0]
ax3.grid(True)
for gamma in dom["gamma_list"]:
    Fy0g = make_Fy0(dom["alpha"], Fz_nom, gamma)
    ax3.plot(rad2deg(dom["alpha"]), Fy0g, label=f"gamma = {rad2deg(gamma):+g} deg")
ax3.set_xlabel("Alpha (deg)")
ax3.set_ylabel("Fy0 (N)")
ax3.set_title("Fy0(alpha) vs camber at Fz = 4000 N")
style_dark(ax3)
lg = ax3.legend(loc="lower right")
style_legend_dark(lg)

ax4 = axs1[1, 1]
ax4.grid(True)
Mz0 = make_Mz(dom["alpha"], 0.0, Fz_nom, 0.0)
ax4.plot(rad2deg(dom["alpha"]), Mz0)
ax4.set_xlabel("Alpha (deg)")
ax4.set_ylabel("Mz (N*m)")
ax4.set_title("Aligning moment Mz(alpha), Fz = 4000 N, kappa = 0, gamma = 0")
style_dark(ax4)

# Figure 2: Combined-slip slices
fig2, axs2 = plt.subplots(2, 2, figsize=(10, 8), constrained_layout=True)
fig2.patch.set_facecolor((0.0, 0.0, 0.0))

ax5 = axs2[0, 0]
ax5.grid(True)
alpha_slices_deg = [-10.0, 0.0, 10.0]
Fz_plot = Fz_nom
for a_deg in alpha_slices_deg:
    a = deg2rad(a_deg)
    Fx_comb = make_Fx(dom["kappa"], a, Fz_plot, 0.0)
    ax5.plot(dom["kappa"], Fx_comb, label=f"alpha = {a_deg:+g} deg")
ax5.set_xlabel("Kappa (-)")
ax5.set_ylabel("Fx (N)")
ax5.set_title(f"Fx(kappa) at Fz = {Fz_plot:.0f} N")
style_dark(ax5)
lg = ax5.legend(loc="lower right")
style_legend_dark(lg)

ax6 = axs2[0, 1]
ax6.grid(True)
kappa_slices = [-0.12, 0.0, 0.12]
for k in kappa_slices:
    Fy_comb = make_Fy(dom["alpha"], k, Fz_plot, 0.0)
    ax6.plot(rad2deg(dom["alpha"]), Fy_comb, label=f"kappa = {k:+0.3f}")
ax6.set_xlabel("Alpha (deg)")
ax6.set_ylabel("Fy (N)")
ax6.set_title(f"Fy(alpha) at Fz = {Fz_plot:.0f} N")
style_dark(ax6)
lg = ax6.legend(loc="lower right")
style_legend_dark(lg)

ax7 = axs2[1, 0]
ax7.grid(True)
for k in kappa_slices:
    Mz_comb = make_Mz(dom["alpha"], k, Fz_plot, 0.0)
    ax7.plot(rad2deg(dom["alpha"]), Mz_comb, label=f"kappa = {k:+0.3f}")
ax7.set_xlabel("Alpha (deg)")
ax7.set_ylabel("Mz (N*m)")
ax7.set_title(f"Mz(alpha) at Fz = {Fz_plot:.0f} N")
style_dark(ax7)
lg = ax7.legend(loc="lower right")
style_legend_dark(lg)

ax8 = axs2[1, 1]
ax8.grid(True)
kappa_grid = np.linspace(-0.2, 0.2, 141)
alpha_grid = deg2rad(np.linspace(-12.0, 12.0, 141))
Kmesh, Amesh = np.meshgrid(kappa_grid, alpha_grid)
Fxm = make_Fx(Kmesh, Amesh, Fz_plot, 0.0)
Fym = make_Fy(Amesh, Kmesh, Fz_plot, 0.0)
util = np.sqrt((Fxm / (mu_x(Fz_plot) * Fz_plot)) ** 2 + (Fym / (mu_y(Fz_plot) * Fz_plot)) ** 2)
levels = np.arange(0.2, 1.6 + 0.1, 0.1)
cf = ax8.contourf(Kmesh, rad2deg(Amesh), util, levels=levels)
cb = fig2.colorbar(cf, ax=ax8)
cb.ax.yaxis.set_tick_params(color=(1.0, 1.0, 1.0))
plt.setp(plt.getp(cb.ax.axes, "yticklabels"), color=(1.0, 1.0, 1.0))
cf.set_clim(0.0, 1.6)
ax8.contour(Kmesh, rad2deg(Amesh), util, levels=[1.0], colors="w", linewidths=1.5)
ax8.set_xlabel("Kappa (-)")
ax8.set_ylabel("Alpha (deg)")
ax8.set_title("Friction ellipse utilization (1.0 contour)")
style_dark(ax8)

# Figure 3: 3D surfaces at nominal load
fig3, axs3 = plt.subplots(1, 2, figsize=(10, 5), subplot_kw={"projection": "3d"}, constrained_layout=True)
fig3.patch.set_facecolor((0.0, 0.0, 0.0))

ax9 = axs3[0]
kappa_grid2 = np.linspace(-0.2, 0.2, 101)
alpha_grid2 = deg2rad(np.linspace(-12.0, 12.0, 101))
Kmesh2, Amesh2 = np.meshgrid(kappa_grid2, alpha_grid2)
Fx_surf = make_Fx(Kmesh2, Amesh2, Fz_nom, 0.0)
surf1 = ax9.plot_surface(Kmesh2, rad2deg(Amesh2), Fx_surf, edgecolor="none")
ax9.view_init(elev=25, azim=135)
ax9.set_xlabel("Kappa (-)")
ax9.set_ylabel("Alpha (deg)")
ax9.set_zlabel("Fx (N)")
ax9.set_title("Fx(kappa, alpha), Fz = 4000 N")
cb1 = fig3.colorbar(surf1, ax=ax9, shrink=0.6)
cb1.ax.yaxis.set_tick_params(color=(1.0, 1.0, 1.0))
plt.setp(plt.getp(cb1.ax.axes, "yticklabels"), color=(1.0, 1.0, 1.0))
style_dark(ax9)

ax10 = axs3[1]
Fy_surf = make_Fy(Amesh2, Kmesh2, Fz_nom, 0.0)
surf2 = ax10.plot_surface(Kmesh2, rad2deg(Amesh2), Fy_surf, edgecolor="none")
ax10.view_init(elev=25, azim=135)
ax10.set_xlabel("Kappa (-)")
ax10.set_ylabel("Alpha (deg)")
ax10.set_zlabel("Fy (N)")
ax10.set_title("Fy(alpha, kappa), Fz = 4000 N")
cb2 = fig3.colorbar(surf2, ax=ax10, shrink=0.6)
cb2.ax.yaxis.set_tick_params(color=(1.0, 1.0, 1.0))
plt.setp(plt.getp(cb2.ax.axes, "yticklabels"), color=(1.0, 1.0, 1.0))
style_dark(ax10)

plt.draw()

# Export with black background
outDir = os.path.join(os.getcwd(), "output")
os.makedirs(outDir, exist_ok=True)
fig1.savefig(os.path.join(outDir, "pacejka_pure_black.png"),
             dpi=300, facecolor=fig1.get_facecolor())
fig2.savefig(os.path.join(outDir, "pacejka_combined_black.png"),
             dpi=300, facecolor=fig2.get_facecolor())
fig3.savefig(os.path.join(outDir, "pacejka_surfaces_black.png"),
             dpi=300, facecolor=fig3.get_facecolor())

# Demo printout
demo = {}
demo["kappa"] = 0.10
demo["alpha"] = deg2rad(6.0)
Fx0_demo = make_Fx0(demo["kappa"], Fz_nom)
Fy0_demo = make_Fy0(demo["alpha"], Fz_nom, 0.0)
Fx_demo = make_Fx(demo["kappa"], demo["alpha"], Fz_nom, 0.0)
Fy_demo = make_Fy(demo["alpha"], demo["kappa"], Fz_nom, 0.0)
Mz_demo = make_Mz(demo["alpha"], demo["kappa"], Fz_nom, 0.0)

print(
    "Fz={:.0f} N, kappa={:.3f}, alpha={:+.1f} deg -> "
    "Fx0={:.1f} N, Fy0={:.1f} N, Fx={:.1f} N, Fy={:.1f} N, Mz={:.1f} N*m".format(
        Fz_nom,
        demo["kappa"],
        rad2deg(demo["alpha"]),
        Fx0_demo,
        Fy0_demo,
        Fx_demo,
        Fy_demo,
        Mz_demo,
    )
)

plt.show()  # enable if you want an interactive window
