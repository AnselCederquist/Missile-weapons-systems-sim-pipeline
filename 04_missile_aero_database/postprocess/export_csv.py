import numpy as np
import pandas as pd

MACH = [0.8, 1.2, 1.6, 2.0, 3.0]
ALPHA = [0., 4., 8., 12., 16., 20.]

CL = [
    [0.000, 0.971, 1.942, 2.914, 3.885, 4.856],
    [0.000, 0.907, 1.813, 2.720, 3.627, 4.533],
    [None]*6,
    [None]*6,
    [None]*6,
]
CM = [
    [None]*6,
    [None]*6,
    [0.000, -2.919, -5.833, -8.672, None, None],
    [0.000, -2.128, -4.354, -6.743, -9.391, None],
    [0.000, -1.263, -2.630, -4.189, -6.011, -8.011],
]
CD = [
    [0.236, None, None, None, None, None],
    [0.429, None, None, None, None, None],
    [None]*6,
    [None]*6,
    [None]*6,
]

rows = []
for i,m in enumerate(MACH):
    for j,a in enumerate(ALPHA):
        rows.append({"Mach": m, "Alpha_deg": a, "CL": CL[i][j], "CM": CM[i][j], "CD": CD[i][j]})

import pandas as pd
df = pd.DataFrame(rows)
df.to_csv("D:/Weapons-systems-sim-pipeline/04_missile_aero_database/results/aero_database.csv", index=False)
print(df.to_string())
print("aero_database.csv written")
