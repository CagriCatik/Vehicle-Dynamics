# --------------------------------------------------------------------
#  File: pacejka_tire.py
#  Description: Object-oriented implementation of a Pacejka-style tire model
#               with black-background plotting.
# --------------------------------------------------------------------

import numpy as np
import matplotlib.pyplot as plt
import os

# Global plot theme (black background)
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


class PacejkaTire:
    """
    Object-oriented Pacejka-style tire model.
    """

    def __init__(self, params=None):
        if params is None:
            params = self.defaultParams()
        self.par = params

        # vehicle
        self.mass = 1500.0
        self.g = 9.81
        self.static_wdist = 0.6

        # nominal per-tire load used in parameters
        self.Fz_nom = 4000.0

    # ----------------------------------------------------------------
    #  Pure-slip forces
    def calcFx0(self, kappa, Fz):
        Fz_arr = np.asarray(Fz)
        B = self.Kx_small(Fz_arr) / (self.par["Cx"] * (self.mu_x(Fz_arr) * Fz_arr))
        D = self.mu_x(Fz_arr) * Fz_arr
        return self.mf(kappa, B, self.par["Cx"], D, self.par["Ex"], 0.0, 0.0)

    def calcFy0(self, alpha, Fz, gamma):
        Fz_arr = np.asarray(Fz)
        gamma_arr = np.asarray(gamma)
        B = self.Ca_small(Fz_arr) / (self.par["Cy"] * (self.mu_y(Fz_arr) * Fz_arr))
        D = self.mu_y(Fz_arr) * Fz_arr
        Sh = self.par["Sh_y_gamma"] * gamma_arr
        Sv = (
            self.par["Sv_y_gamma"]
            * gamma_arr
            * (Fz_arr / self.Fz_nom)
            * Fz_arr
        )
        return self.mf(alpha, B, self.par["Cy"], D, self.par["Ey"], Sh, Sv)

    # ----------------------------------------------------------------
    #  Combined-slip forces
    def calcFx(self, kappa, alpha, Fz, gamma):
        return self.calcFx0(kappa, Fz) * self.Gx(alpha, Fz)

    def calcFy(self, alpha, kappa, Fz, gamma):
        return self.calcFy0(alpha, Fz, gamma) * self.Gy(kappa, Fz)

    # ----------------------------------------------------------------
    #  Aligning moment
    def calcMz(self, alpha, kappa, Fz, gamma):
        trail = self.trail_comb(alpha, kappa, Fz)
        return -trail * self.calcFy(alpha, kappa, Fz, gamma)

    # ----------------------------------------------------------------
    #  Helpers: load sensitivity and combined-slip weights
    def mu_x(self, Fz):
        Fz_arr = np.asarray(Fz)
        return self.par["mu_x0"] * (
            1.0 + self.par["mu_x_load_slope"] * ((Fz_arr / self.Fz_nom) - 1.0)
        )

    def mu_y(self, Fz):
        Fz_arr = np.asarray(Fz)
        return self.par["mu_y0"] * (
            1.0 + self.par["mu_y_load_slope"] * ((Fz_arr / self.Fz_nom) - 1.0)
        )

    def Kx_small(self, Fz):
        Fz_arr = np.asarray(Fz)
        return self.par["Kx0"] * (Fz_arr / self.Fz_nom) ** self.par["Kx_exp"]

    def Ca_small(self, Fz):
        Fz_arr = np.asarray(Fz)
        return self.par["Ca0"] * (Fz_arr / self.Fz_nom) ** self.par["Ca_exp"]

    def Gx(self, alpha, Fz):
        Fz_arr = np.asarray(Fz)
        alpha_arr = np.asarray(alpha)
        return np.cos(
            self.par["Cgx"] * np.arctan(self.Bgx(Fz_arr) * alpha_arr)
        )

    def Gy(self, kappa, Fz):
        Fz_arr = np.asarray(Fz)
        kappa_arr = np.asarray(kappa)
        return np.cos(
            self.par["Cgy"] * np.arctan(self.Bgy(Fz_arr) * kappa_arr)
        )

    def Bgx(self, Fz):
        Fz_arr = np.asarray(Fz)
        return np.maximum(
            0.1,
            self.par["Bgx0"] * (self.Fz_nom / Fz_arr) ** self.par["Bg_exp"],
        )

    def Bgy(self, Fz):
        Fz_arr = np.asarray(Fz)
        return np.maximum(
            0.1,
            self.par["Bgy0"] * (self.Fz_nom / Fz_arr) ** self.par["Bg_exp"],
        )

    def trail0(self, Fz):
        Fz_arr = np.asarray(Fz)
        return self.par["t0"] * (Fz_arr / self.Fz_nom) ** self.par["t_exp"]

    def trail_pure(self, alpha, Fz):
        Fz_arr = np.asarray(Fz)
        alpha_arr = np.asarray(alpha)
        return self.trail0(Fz_arr) * np.cos(
            self.par["Ct"] * np.arctan(self.Bt(Fz_arr) * alpha_arr)
        )

    def trail_comb(self, alpha, kappa, Fz):
        Fz_arr = np.asarray(Fz)
        alpha_arr = np.asarray(alpha)
        kappa_arr = np.asarray(kappa)
        return self.trail_pure(alpha_arr, Fz_arr) * np.cos(
            self.par["Ct"]
            * np.arctan(
                self.par["Gt_kappa"]
                * self.Btk(Fz_arr)
                * kappa_arr
            )
        )

    def Bt(self, Fz_unused):
        return self.par["Bt0"]

    def Btk(self, Fz_unused):
        return self.par["Btk0"]

    def mf(self, x, B, C, D, E, Sh, Sv):
        x_arr = np.asarray(x)
        B_arr = np.asarray(B)
        C_arr = np.asarray(C)
        D_arr = np.asarray(D)
        E_arr = np.asarray(E)
        Sh_arr = np.asarray(Sh)
        Sv_arr = np.asarray(Sv)
        x_shifted = x_arr + Sh_arr
        Bx = B_arr * x_shifted
        inner = Bx - E_arr * (Bx - np.arctan(Bx))
        return D_arr * np.sin(C_arr * np.arctan(inner)) + Sv_arr

    # ----------------------------------------------------------------
    #  Plotting
    def plotPureCurves(self):
        """
        Plot pure-slip Fx0(kappa), Fy0(alpha) vs load and camber, and Mz(alpha).
        Returns the created figure.
        """
        # Domains (same as in the script)
        kappa = np.linspace(-0.25, 0.25, 801)
        alpha = deg2rad(np.linspace(-15.0, 15.0, 801))
        gamma_list = deg2rad(np.array([0.0, -2.0, -4.0]))
        loads = np.array([3000.0, 4000.0, 5000.0])

        fig, axs = plt.subplots(2, 2, figsize=(10, 8), constrained_layout=True)
        fig.patch.set_facecolor((0.0, 0.0, 0.0))

        # Ax1: Fx0 vs kappa for different loads
        ax1 = axs[0, 0]
        ax1.grid(True)
        for Fz in loads:
            Fx0 = self.calcFx0(kappa, Fz)
            ax1.plot(kappa, Fx0, label=f"Fz = {Fz:.0f} N")
        ax1.set_xlabel("Kappa (-)")
        ax1.set_ylabel("Fx0 (N)")
        ax1.set_title("Pure longitudinal Fx0(kappa)")
        self._style_dark(ax1)
        lg1 = ax1.legend(loc="lower right")
        self._style_legend_dark(lg1)

        # Ax2: Fy0 vs alpha for different loads (gamma=0)
        ax2 = axs[0, 1]
        ax2.grid(True)
        for Fz in loads:
            Fy0 = self.calcFy0(alpha, Fz, 0.0)
            ax2.plot(rad2deg(alpha), Fy0, label=f"Fz = {Fz:.0f} N")
        ax2.set_xlabel("Alpha (deg)")
        ax2.set_ylabel("Fy0 (N)")
        ax2.set_title("Pure lateral Fy0(alpha), gamma = 0 deg")
        self._style_dark(ax2)
        lg2 = ax2.legend(loc="lower right")
        self._style_legend_dark(lg2)

        # Ax3: Fy0 vs alpha for different camber at nominal load
        ax3 = axs[1, 0]
        ax3.grid(True)
        for gamma in gamma_list:
            Fy0g = self.calcFy0(alpha, self.Fz_nom, gamma)
            ax3.plot(
                rad2deg(alpha),
                Fy0g,
                label=f"gamma = {rad2deg(gamma):+g} deg",
            )
        ax3.set_xlabel("Alpha (deg)")
        ax3.set_ylabel("Fy0 (N)")
        ax3.set_title("Fy0(alpha) vs camber at Fz = 4000 N")
        self._style_dark(ax3)
        lg3 = ax3.legend(loc="lower right")
        self._style_legend_dark(lg3)

        # Ax4: Mz vs alpha at nominal conditions
        ax4 = axs[1, 1]
        ax4.grid(True)
        Mz0 = self.calcMz(alpha, 0.0, self.Fz_nom, 0.0)
        ax4.plot(rad2deg(alpha), Mz0)
        ax4.set_xlabel("Alpha (deg)")
        ax4.set_ylabel("Mz (N*m)")
        ax4.set_title(
            "Aligning moment Mz(alpha), Fz = 4000 N, kappa = 0, gamma = 0"
        )
        self._style_dark(ax4)

        return fig

    def exportFigures(self, out_dir):
        """
        Export all current matplotlib figures to PNG files with black background.
        """
        os.makedirs(out_dir, exist_ok=True)
        for num in plt.get_fignums():
            fig = plt.figure(num)
            filename = os.path.join(out_dir, f"figure_{num}.png")
            fig.savefig(filename, dpi=300, facecolor=fig.get_facecolor())

    # ----------------------------------------------------------------
    #  Internal styling helpers (dark theme)
    def _style_dark(self, ax):
        ax.set_facecolor((0.0, 0.0, 0.0))
        ax.tick_params(colors=(1.0, 1.0, 1.0))
        for spine in ax.spines.values():
            spine.set_edgecolor((1.0, 1.0, 1.0))
        ax.grid(True, color=(0.28, 0.28, 0.28))
        ax.title.set_color((1.0, 1.0, 1.0))
        ax.xaxis.label.set_color((1.0, 1.0, 1.0))
        ax.yaxis.label.set_color((1.0, 1.0, 1.0))

    def _style_legend_dark(self, lg):
        if lg is None:
            return
        frame = lg.get_frame()
        frame.set_facecolor("none")
        frame.set_edgecolor((1.0, 1.0, 1.0))
        for text in lg.get_texts():
            text.set_color((1.0, 1.0, 1.0))

    # ----------------------------------------------------------------
    #  Default parameter set (matches script layout)
    def defaultParams(self):
        params = {}
        # Longitudinal pure-slip
        params["mu_x0"] = 1.00          # peak friction at Fz_nom
        params["Cx"] = 1.65             # shape factor
        params["Ex"] = 0.97             # curvature
        params["Kx0"] = 110e3           # small-slip stiffness at Fz_nom [N]
        params["Kx_exp"] = 0.8          # load sensitivity exponent for Kx
        params["mu_x_load_slope"] = -0.05  # mu_x(Fz) slope

        # Lateral pure-slip
        params["mu_y0"] = 0.95
        params["Cy"] = 1.30
        params["Ey"] = 1.00
        params["Ca0"] = 80e3            # cornering stiffness at Fz_nom [N/rad]
        params["Ca_exp"] = 0.9          # load sensitivity exponent for Ca
        params["mu_y_load_slope"] = -0.06

        # Camber influence (shifts)
        params["Sh_y_gamma"] = 0.015    # horizontal shift per rad camber [rad/rad]
        params["Sv_y_gamma"] = 0.0      # vertical shift gain per rad camber

        # Combined-slip weighting
        params["Cgx"] = 1.1             # Fx reduction vs alpha
        params["Cgy"] = 1.1             # Fy reduction vs kappa
        params["Bgx0"] = 6.0            # B for Gx(alpha) at Fz_nom
        params["Bgy0"] = 8.0            # B for Gy(kappa) at Fz_nom
        params["Bg_exp"] = 0.6          # load exponent for Bgx/Bgy

        # Pneumatic trail model for Mz = -t * Fy
        params["t0"] = 0.15             # nominal trail at small slip [m]
        params["t_exp"] = 0.6           # load sensitivity exponent for trail
        params["Bt0"] = 5.0             # trail drop-off vs alpha
        params["Ct"] = 1.1              # trail shape factor
        params["Gt_kappa"] = 1.0        # trail reduction vs kappa weight
        params["Btk0"] = 5.0            # trail reduction vs kappa sensitivity

        return params
