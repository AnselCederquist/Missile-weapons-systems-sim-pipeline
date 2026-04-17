"""
zdata_export.py -- ZDATA Format Export for Warhead Fragmentation Data
Project 12 -- Lethality

Exports Gurney+Mott warhead characterization in ZDATA format, the
standard fragment mass distribution format used by AMSAA/JMEM lethality
codes. Provides interoperability with published warhead databases.

ZDATA format (Yager, 2013):
  - Header: warhead identification and geometry
  - Per-zone: fragment mass distribution in tabular form
  - Shape factor for fragment drag computation

Reference:
  Yager, R.J. "ZDATA File Format for Warhead Fragmentation Data",
  US Army Research Laboratory, 2013.

Usage:
  python zdata_export.py                  # exports default warhead
  python zdata_export.py --output my.zdata  # custom output path
"""

import os
import sys
import numpy as np
from datetime import datetime

THIS_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, THIS_DIR)

from lethality import (
    WarheadParams, gurney_velocity, mott_distribution,
    fragment_velocity_at_range,
)


def export_zdata(wp: WarheadParams = WarheadParams(),
                 output_path: str = None,
                 n_bins: int = 50) -> str:
    """
    Export warhead fragmentation data in ZDATA-compatible format.

    The ZDATA format stores fragment mass distributions per spherical zone.
    This simplified implementation exports a single zone (broadside belt)
    consistent with the current lethality model's assumptions.

    Parameters
    ----------
    wp : WarheadParams
    output_path : str — output file path (default: ../results/warhead.zdata)
    n_bins : int — number of mass bins in the distribution

    Returns
    -------
    output_path : str — path to written file
    """
    V0 = gurney_velocity(wp)
    masses, counts, N_total, mu = mott_distribution(wp, n_bins=n_bins)

    if output_path is None:
        output_path = os.path.join(THIS_DIR, '..', 'results', 'warhead.zdata')

    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)

    cm_ratio = wp.M_charge / wp.M_case

    lines = []

    # Header
    lines.append(f"# ZDATA Fragment Mass Distribution Export")
    lines.append(f"# Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append(f"# Source: Project 12 Lethality Model (Gurney+Mott)")
    lines.append(f"#")
    lines.append(f"# Warhead Identification")
    lines.append(f"WARHEAD_ID          Project12_SAM_Warhead")
    lines.append(f"CASE_MATERIAL       Steel_AISI_1020")
    lines.append(f"CASE_MASS_KG        {wp.M_case:.3f}")
    lines.append(f"CHARGE_MASS_KG      {wp.M_charge:.3f}")
    lines.append(f"CHARGE_TYPE         Comp_B")
    lines.append(f"CM_RATIO            {cm_ratio:.4f}")
    lines.append(f"CASE_LENGTH_M       {wp.case_length:.4f}")
    lines.append(f"CASE_OD_M           {2*wp.case_radius:.4f}")
    lines.append(f"GURNEY_SQRT2E       {wp.sqrt_2E:.1f}")
    lines.append(f"FRAG_VELOCITY_MPS   {V0:.1f}")
    lines.append(f"FRAG_SHAPE          {wp.frag_shape}")
    lines.append(f"FRAG_CD             {wp.frag_CD:.2f}")
    lines.append(f"FRAG_DENSITY_KGM3   {wp.frag_rho:.1f}")
    lines.append(f"#")
    lines.append(f"# Mott Distribution Parameters")
    lines.append(f"MOTT_MU_KG          {mu:.6f}")
    lines.append(f"MOTT_MU_G           {mu*1000:.3f}")
    lines.append(f"MOTT_B              0.68")
    lines.append(f"N_TOTAL             {N_total}")
    lines.append(f"#")
    lines.append(f"# Fragment Zone Definition")
    lines.append(f"N_ZONES             1")
    lines.append(f"#")
    lines.append(f"# Zone 1: Broadside equatorial belt")
    lines.append(f"ZONE_ID             1")
    lines.append(f"ZONE_THETA_START    {wp.beam_center_deg - wp.beam_half_angle:.1f}")
    lines.append(f"ZONE_THETA_END      {wp.beam_center_deg + wp.beam_half_angle:.1f}")
    lines.append(f"ZONE_N_FRAGS        {N_total}")
    lines.append(f"ZONE_VELOCITY_MPS   {V0:.1f}")
    lines.append(f"#")
    lines.append(f"# Fragment Mass Distribution (cumulative N(>m) = N_total * exp(-sqrt(m/mu)))")
    lines.append(f"# Columns: mass_kg  mass_g  count  cumulative_N_gt  frac_lethal_at_10m")
    lines.append(f"N_MASS_BINS         {n_bins}")
    lines.append(f"#")
    lines.append(f"# {'mass_kg':>12s}  {'mass_g':>8s}  {'count':>10s}  {'N_gt_m':>10s}  {'V_at_10m':>10s}  {'KE_at_10m':>10s}  {'lethal':>7s}")

    for i in range(n_bins):
        m = masses[i]
        c = counts[i]
        N_gt = N_total * np.exp(-np.sqrt(m / mu))
        V_10 = fragment_velocity_at_range(V0, m, 10.0, wp)
        KE_10 = 0.5 * m * V_10**2
        lethal = "YES" if KE_10 > 80.0 else "NO"
        lines.append(f"  {m:12.6f}  {m*1000:8.3f}  {c:10.1f}  {N_gt:10.1f}  {V_10:10.1f}  {KE_10:10.1f}  {lethal:>7s}")

    lines.append(f"#")
    lines.append(f"# End of ZDATA export")
    lines.append(f"# Reference: Yager (2013) ZDATA file format")

    content = '\n'.join(lines) + '\n'

    with open(output_path, 'w') as f:
        f.write(content)

    return output_path


# ---------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Export warhead data in ZDATA format')
    parser.add_argument('--output', type=str, default=None, help='Output file path')
    args = parser.parse_args()

    wp = WarheadParams()
    path = export_zdata(wp, output_path=args.output)

    print(f"\n  ZDATA export complete")
    print(f"  Output: {os.path.abspath(path)}")

    # Print summary
    V0 = gurney_velocity(wp)
    masses, counts, N_total, mu = mott_distribution(wp, n_bins=50)
    M_reconstructed = np.sum(masses * counts)

    print(f"\n  Warhead: {wp.M_case:.1f}kg case + {wp.M_charge:.1f}kg charge")
    print(f"  Gurney velocity: {V0:.0f} m/s")
    print(f"  Mott mu: {mu*1000:.2f}g  N={N_total}")
    print(f"  Mass conservation: {M_reconstructed/wp.M_case*100:.1f}%")
    print(f"  Zone: {wp.beam_center_deg - wp.beam_half_angle:.0f}-{wp.beam_center_deg + wp.beam_half_angle:.0f} deg from nose")