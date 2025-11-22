# --------------------------------------------------------------------
#  File: run_pacejka_demo.py
# --------------------------------------------------------------------

import numpy as np
from pacejka_tire import PacejkaTire, deg2rad, rad2deg  # reuse helpers

# 1) Load the class
tire = PacejkaTire()  # or pass a custom params dict

# 2) Pick a slip state
demo = {}
demo["kappa"] = 0.10
demo["alpha"] = deg2rad(6.0)
demo["gamma"] = 0.0
demo["Fz"] = 4000.0

# 3) Compute forces and moment
Fx0 = tire.calcFx0(demo["kappa"], demo["Fz"])
Fy0 = tire.calcFy0(demo["alpha"], demo["Fz"], demo["gamma"])
Fx = tire.calcFx(demo["kappa"], demo["alpha"], demo["Fz"], demo["gamma"])
Fy = tire.calcFy(demo["alpha"], demo["kappa"], demo["Fz"], demo["gamma"])
Mz = tire.calcMz(demo["alpha"], demo["kappa"], demo["Fz"], demo["gamma"])

print("Fx0={:.1f} N, Fy0={:.1f} N, Fx={:.1f} N, Fy={:.1f} N, Mz={:.1f} N*m".format(Fx0, Fy0, Fx, Fy, Mz))

# 4) Plot and export
fig = tire.plotPureCurves()
tire.exportFigures("output")

# Optionally show interactively
import matplotlib.pyplot as plt
plt.show()
