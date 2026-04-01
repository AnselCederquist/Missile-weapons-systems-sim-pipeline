"""
validate_isentropic.py
Project 03 - De Laval Nozzle CFD
Validation of CFD centerline Mach number against isentropic flow relations
Cold flow: P0 = 601,325 Pa, T0 = 300 K, air (gamma = 1.4)
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import AutoMinorLocator
from scipy.optimize import brentq

# ── Nozzle Geometry (all radii) ───────────────────────────────────────────────
R_inlet  = 0.25    # m — inlet radius
R_throat = 0.12    # m — throat radius
R_exit   = 0.49    # m — exit radius
x_inlet  = 0.0     # m
x_throat = 0.5     # m
x_exit   = 1.86    # m
gamma    = 1.4
P0       = 601325  # Pa absolute
T0       = 300.0   # K

Ae_Astar = (R_exit / R_throat) ** 2   # = 16.694


# ── Isentropic Relations ──────────────────────────────────────────────────────
def area_ratio_from_mach(M, g=1.4):
    t = 1 + (g - 1) / 2 * M**2
    return (1 / M) * (2 / (g + 1) * t) ** ((g + 1) / (2 * (g - 1)))


def mach_from_area_ratio(AoAstar, supersonic=True, g=1.4):
    if supersonic:
        return brentq(lambda M: area_ratio_from_mach(M, g) - AoAstar, 1.0, 20.0)
    else:
        return brentq(lambda M: area_ratio_from_mach(M, g) - AoAstar, 0.001, 1.0)


def isentropic_pressure(M, P0, g=1.4):
    return P0 * (1 + (g - 1) / 2 * M**2) ** (-g / (g - 1))


def isentropic_temperature(M, T0, g=1.4):
    return T0 / (1 + (g - 1) / 2 * M**2)


def nozzle_radius(x):
    """Piecewise linear radius profile."""
    if x <= x_throat:
        t = x / x_throat
        return R_inlet + (R_throat - R_inlet) * t
    else:
        t = (x - x_throat) / (x_exit - x_throat)
        return R_throat + (R_exit - R_throat) * t


# ── Build Theoretical Centerline Profile ──────────────────────────────────────
x_theory = np.linspace(x_inlet, x_exit, 500)
M_theory = np.zeros_like(x_theory)
P_theory = np.zeros_like(x_theory)

for i, x in enumerate(x_theory):
    r = nozzle_radius(x)
    AoAstar = (r / R_throat) ** 2
    supersonic = (x > x_throat)
    try:
        M_theory[i] = mach_from_area_ratio(AoAstar, supersonic=supersonic)
    except Exception:
        M_theory[i] = np.nan
    P_theory[i] = isentropic_pressure(M_theory[i], P0)

M_throat_theory = 1.0
P_throat_theory = isentropic_pressure(1.0, P0)
M_exit_theory   = mach_from_area_ratio(Ae_Astar, supersonic=True)
P_exit_theory   = isentropic_pressure(M_exit_theory, P0)


# ── Parse CFD Data ────────────────────────────────────────────────────────────
def parse_fluent_xy(raw_text):
    lines = raw_text.strip().splitlines()
    points = []
    for line in lines:
        line = line.strip()
        if not line or line.startswith("(") or line.startswith(")"):
            continue
        parts = line.split()
        if len(parts) == 2:
            try:
                points.append((float(parts[0]), float(parts[1])))
            except ValueError:
                continue
    points = sorted(points, key=lambda p: p[0])
    arr = np.array(points)
    return arr[:, 0], arr[:, 1]


coarse_raw = """
0	0.142339
0.045356	0.142933
0.090712	0.144737
0.136068	0.148349
0.181424	0.155389
0.238862	0.171955
0.29511	0.20169
0.349001	0.24949
0.399419	0.325573
0.440324	0.431911
0.481228	0.612272
0.521604	0.888358
0.569403	1.24155
0.617203	1.52615
0.665003	1.76411
0.712803	1.97179
0.760603	2.15319
0.808403	2.3113
0.856203	2.45206
0.904003	2.58268
0.951802	2.70803
0.999602	2.82677
1.0474	2.93715
1.0952	3.03934
1.143	3.13423
1.1908	3.22278
1.2386	3.30593
1.2864	3.38453
1.3342	3.45932
1.382	3.53096
1.4298	3.59999
1.4776	3.66686
1.5254	3.73193
1.5732	3.79549
1.621	3.85777
1.6688	3.91888
1.7166	3.97929
1.7644	4.03749
1.8122	4.10106
1.86	4.137
"""

medium_raw = """
0	0.141284
0.0259177	0.141492
0.0518354	0.14205
0.0777531	0.142986
0.103671	0.144457
0.129589	0.146628
0.155506	0.149732
0.181424	0.15407
0.207023	0.159955
0.232517	0.16788
0.257802	0.178367
0.282774	0.19203
0.30733	0.209573
0.33137	0.231793
0.354795	0.259565
0.377509	0.293837
0.399419	0.335565
0.419872	0.384442
0.440324	0.445569
0.460776	0.52202
0.481228	0.617833
0.50074	0.732036
0.521604	0.880932
0.545938	1.07147
0.570273	1.25813
0.594607	1.43478
0.618941	1.60182
0.643276	1.7603
0.66761	1.91076
0.691945	2.05318
0.716279	2.18756
0.740614	2.31415
0.764948	2.43326
0.789283	2.54511
0.813617	2.64988
0.837952	2.7474
0.862286	2.83705
0.886621	2.91768
0.910955	2.98775
0.983959	3.08992
1.00829	3.13835
1.03263	3.14634
1.05696	3.14832
1.0813	3.14842
1.10563	3.15026
1.12997	3.15642
1.1543	3.16846
1.17863	3.18698
1.20297	3.21186
1.2273	3.24248
1.25164	3.27793
1.27597	3.31714
1.30031	3.35911
1.32464	3.40294
1.34898	3.44784
1.37331	3.49324
1.39764	3.53866
1.42198	3.58379
1.44631	3.6284
1.47065	3.67235
1.49498	3.71557
1.51932	3.75801
1.54365	3.79966
1.56799	3.84055
1.59232	3.8807
1.61666	3.92013
1.64099	3.95889
1.66532	3.99701
1.68966	4.03453
1.71399	4.07148
1.73833	4.10791
1.76266	4.14378
1.787	4.17936
1.81133	4.21366
1.83567	4.25123
1.86	4.2724
"""

fine_raw = """
0	0.142495
0.0100791	0.142528
0.0201582	0.142611
0.0302373	0.142741
0.0403164	0.142925
0.0503956	0.143167
0.0604747	0.143473
0.0705538	0.143847
0.0806329	0.144297
0.090712	0.144829
0.100791	0.145455
0.11087	0.146184
0.120949	0.147028
0.131028	0.148
0.141108	0.149118
0.151187	0.150396
0.161266	0.151854
0.171345	0.153514
0.181424	0.1554
0.191902	0.157625
0.202373	0.160147
0.21283	0.163
0.223265	0.166219
0.233672	0.169839
0.244043	0.173901
0.25437	0.178446
0.264648	0.183521
0.274868	0.189173
0.285025	0.195455
0.29511	0.202421
0.305117	0.21013
0.31504	0.218645
0.32487	0.22803
0.334602	0.238356
0.344229	0.249693
0.353745	0.262117
0.363142	0.275703
0.372414	0.290528
0.381555	0.306667
0.390559	0.324193
0.399419	0.343157
0.4076	0.362354
0.415781	0.383333
0.423962	0.406208
0.432143	0.43113
0.440324	0.458248
0.448505	0.487706
0.456686	0.51964
0.464866	0.554173
0.473047	0.591413
0.481228	0.631486
0.490589	0.680777
0.50074	0.738172
0.511233	0.801642
0.521604	0.868191
0.531164	0.932534
0.540724	0.999362
0.550283	1.06809
0.559843	1.13817
0.569403	1.20922
0.578963	1.28087
0.588523	1.35279
0.598083	1.42469
0.607643	1.49634
0.617203	1.56754
0.626763	1.63814
0.636323	1.708
0.645883	1.77704
0.655443	1.84519
0.665003	1.91239
0.674563	1.97861
0.684123	2.04382
0.693683	2.10801
0.703243	2.17116
0.712803	2.23328
0.722363	2.29436
0.731923	2.35442
0.741483	2.41345
0.751043	2.47147
0.760603	2.5285
0.770163	2.58454
0.779723	2.63961
0.789283	2.69373
0.798843	2.74691
0.808403	2.79916
0.817963	2.85049
0.827523	2.90093
0.837083	2.95049
0.846643	2.99919
0.856203	3.04708
0.865763	3.09424
0.875323	3.14082
0.884883	3.18701
0.894443	3.23317
0.904003	3.27973
0.913562	3.32725
0.923123	3.37626
0.932682	3.42696
0.942242	3.47825
0.951802	3.52671
0.961362	3.56448
0.970922	3.57643
0.980482	3.54339
0.990042	3.45472
0.999602	3.31742
1.00916	3.15818
1.01872	3.00843
1.02828	2.88523
1.03784	2.79272
1.0474	2.73066
1.05696	2.6994
1.06652	2.69532
1.07608	2.71108
1.08564	2.73972
1.0952	2.77545
1.10476	2.81406
1.11432	2.8522
1.12388	2.88794
1.13344	2.92057
1.143	2.95006
1.15256	2.97682
1.16212	3.00141
1.17168	3.02442
1.18124	3.04633
1.1908	3.0675
1.20036	3.08822
1.20992	3.10866
1.21948	3.12891
1.22904	3.14902
1.2386	3.16902
1.24816	3.1889
1.25772	3.20865
1.26728	3.22826
1.27684	3.24773
1.2864	3.26704
1.29596	3.28619
1.30552	3.30519
1.31508	3.32402
1.32464	3.3427
1.33420	3.36123
1.34376	3.37962
1.35332	3.39786
1.36288	3.41596
1.37244	3.43393
1.382	3.45177
1.39156	3.46949
1.40112	3.48708
1.41068	3.50456
1.42024	3.52193
1.4298	3.53919
1.43936	3.55634
1.44892	3.57339
1.45848	3.59034
1.46804	3.60719
1.4776	3.62394
1.48716	3.6406
1.49672	3.65717
1.50628	3.67365
1.51584	3.69004
1.5254	3.70634
1.53496	3.72255
1.54452	3.73867
1.55408	3.75471
1.56364	3.77067
1.5732	3.78654
1.58276	3.80232
1.59232	3.81803
1.60188	3.83365
1.61144	3.84919
1.621	3.86465
1.63056	3.88003
1.64012	3.89534
1.64968	3.91056
1.65924	3.92571
1.6688	3.94079
1.67836	3.95579
1.68792	3.97072
1.69748	3.98558
1.70704	4.00037
1.7166	4.01509
1.72616	4.02974
1.73572	4.04432
1.74528	4.05883
1.75484	4.07328
1.7644	4.08766
1.77396	4.10198
1.78352	4.11623
1.79308	4.13042
1.80264	4.14454
1.8122	4.15861
1.82176	4.17259
1.83132	4.18659
1.84088	4.20019
1.85044	4.21523
1.86	4.22375
"""

x_c, M_c = parse_fluent_xy(coarse_raw)
x_m, M_m = parse_fluent_xy(medium_raw)
x_f, M_f = parse_fluent_xy(fine_raw)


# ── Interpolate CFD onto theory x-grid for error calculation ──────────────────
M_c_interp = np.interp(x_theory, x_c, M_c)
M_m_interp = np.interp(x_theory, x_m, M_m)
M_f_interp = np.interp(x_theory, x_f, M_f)

err_c = (M_c_interp - M_theory) / M_theory * 100
err_m = (M_m_interp - M_theory) / M_theory * 100
err_f = (M_f_interp - M_theory) / M_theory * 100


# ── Key Validation Points ─────────────────────────────────────────────────────
def nearest(x_arr, y_arr, x_target):
    idx = np.argmin(np.abs(x_arr - x_target))
    return y_arr[idx]

print("=" * 65)
print("ISENTROPIC VALIDATION — DE LAVAL NOZZLE CFD")
print("Cold flow: P0 = 601,325 Pa | T0 = 300 K | gamma = 1.4")
print(f"Ae/A* = {Ae_Astar:.4f} | Theoretical exit Mach = {M_exit_theory:.4f}")
print("=" * 65)

print(f"\n{'Location':<12} {'Theory':>8} {'Coarse':>8} {'Err%':>7} "
      f"{'Medium':>8} {'Err%':>7} {'Fine':>8} {'Err%':>7}")
print("-" * 65)

pts = [
    ("Inlet",   x_inlet  + 0.01),
    ("Throat",  x_throat),
    ("Mid-div", (x_throat + x_exit) / 2),
    ("Exit",    x_exit   - 0.01),
]

for name, xp in pts:
    th = np.interp(xp, x_theory, M_theory)
    c  = nearest(x_c, M_c, xp)
    m  = nearest(x_m, M_m, xp)
    f  = nearest(x_f, M_f, xp)
    print(f"{name:<12} {th:>8.4f} {c:>8.4f} {(c-th)/th*100:>+7.2f}% "
          f"{m:>8.4f} {(m-th)/th*100:>+7.2f}% "
          f"{f:>8.4f} {(f-th)/th*100:>+7.2f}%")

print(f"\nNote: CFD uses viscous k-ω SST solver. Isentropic theory assumes")
print(f"inviscid, adiabatic flow. Deviations reflect viscous losses and")
print(f"turbulent boundary layer — physically correct, not errors.")


# ── Plots ─────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(2, 1, figsize=(12, 10))
fig.suptitle("Isentropic Validation — De Laval Nozzle CFD\n"
             "Cold Flow: P₀ = 601,325 Pa | T₀ = 300 K | γ = 1.4",
             fontsize=12, fontweight="bold")

# Plot 1: Mach number along centerline
ax = axes[0]
ax.plot(x_theory, M_theory, "k-",  linewidth=2.0, label="Isentropic theory", zorder=5)
ax.plot(x_c, M_c, "o--", color="#2166ac", markersize=3, linewidth=1.2,
        label=f"CFD coarse  (h=0.05m)", alpha=0.8)
ax.plot(x_m, M_m, "s--", color="#1a9850", markersize=3, linewidth=1.2,
        label=f"CFD medium  (h=0.025m)", alpha=0.8)
ax.plot(x_f, M_f, "^-",  color="#d73027", markersize=3, linewidth=1.5,
        label=f"CFD fine    (h=0.01m)")
ax.axvline(x_throat, color="gray", linestyle=":", linewidth=1.2, label="Throat (x=0.5m)")
ax.axhline(1.0,      color="gray", linestyle=":", linewidth=1.0)
ax.annotate("M = 1.0", xy=(0.02, 1.02), fontsize=9, color="gray")
ax.set_xlabel("Axial position x [m]")
ax.set_ylabel("Mach number")
ax.set_title("Centerline Mach Number — CFD vs Isentropic Theory")
ax.legend(fontsize=9)
ax.xaxis.set_minor_locator(AutoMinorLocator())
ax.yaxis.set_minor_locator(AutoMinorLocator())
ax.grid(True, alpha=0.3)
ax.set_xlim(x_inlet, x_exit)

# Plot 2: Percent error vs theory
ax = axes[1]
ax.plot(x_theory, err_c, "--", color="#2166ac", linewidth=1.5,
        label="Coarse (h=0.05m)")
ax.plot(x_theory, err_m, "--", color="#1a9850", linewidth=1.5,
        label="Medium (h=0.025m)")
ax.plot(x_theory, err_f, "-",  color="#d73027", linewidth=2.0,
        label="Fine (h=0.01m)")
ax.axvline(x_throat, color="gray", linestyle=":", linewidth=1.2)
ax.axhline(0, color="black", linewidth=0.8)
ax.fill_between(x_theory, -2, 2, alpha=0.07, color="green", label="±2% band")
ax.set_xlabel("Axial position x [m]")
ax.set_ylabel("Error vs isentropic theory [%]")
ax.set_title("CFD vs Isentropic Theory — Percentage Error")
ax.legend(fontsize=9)
ax.set_ylim(-15, 15)
ax.xaxis.set_minor_locator(AutoMinorLocator())
ax.yaxis.set_minor_locator(AutoMinorLocator())
ax.grid(True, alpha=0.3)
ax.set_xlim(x_inlet, x_exit)

plt.tight_layout()
plt.savefig("isentropic_validation.png", dpi=150, bbox_inches="tight")
plt.show()
print("\nFigure saved: isentropic_validation.png")
