import numpy as np
from scipy.interpolate import RegularGridInterpolator

MACH = np.array([0.8, 1.2, 1.6, 2.0, 3.0])
ALPHA = np.array([0., 4., 8., 12., 16., 20.])

CL = np.array([
    [0.000, 0.971, 1.942, 2.914, 3.885, 4.856],
    [0.000, 0.907, 1.813, 2.720, 3.627, 4.533],
    [np.nan]*6,
    [np.nan]*6,
    [np.nan]*6,
])
CM = np.array([
    [np.nan]*6,
    [np.nan]*6,
    [0.000, -2.919, -5.833, -8.672, np.nan, np.nan],
    [0.000, -2.128, -4.354, -6.743, -9.391, np.nan],
    [0.000, -1.263, -2.630, -4.189, -6.011, -8.011],
])
CD = np.array([
    [0.236, np.nan, np.nan, np.nan, np.nan, np.nan],
    [0.429, np.nan, np.nan, np.nan, np.nan, np.nan],
    [np.nan]*6,
    [np.nan]*6,
    [np.nan]*6,
])

def _fill(arr):
    out=arr.copy()
    out[np.isnan(out)]=0.0
    return out

_CL=RegularGridInterpolator((MACH,ALPHA),_fill(CL),method="linear",bounds_error=False,fill_value=None)
_CM=RegularGridInterpolator((MACH,ALPHA),_fill(CM),method="linear",bounds_error=False,fill_value=None)
_CD=RegularGridInterpolator((MACH,ALPHA),_fill(CD),method="linear",bounds_error=False,fill_value=None)

def get_CL(mach,alpha_deg): return float(_CL([[mach,alpha_deg]])[0])
def get_CM(mach,alpha_deg): return float(_CM([[mach,alpha_deg]])[0])
def get_CD(mach,alpha_deg): return float(_CD([[mach,alpha_deg]])[0])

if __name__=="__main__":
    print("CL(M=1.0, a=8) =", get_CL(1.0, 8.0))
    print("CM(M=2.0, a=8) =", get_CM(2.0, 8.0))
    print("CD(M=0.8, a=0) =", get_CD(0.8, 0.0))
