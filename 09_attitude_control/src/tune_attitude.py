"""
tune_attitude.py -- Bayesian LQR Parameter Optimization
Project 09 -- Attitude Control

Uses Optuna TPE. Persistent SQLite — kill and resume anytime.
Requires the rewritten attitude_sim.py with module-level tunable constants.

INSTALL:   pip install optuna
RUN:       python tune_attitude.py
RESUME:    python tune_attitude.py
RESULTS:   python tune_attitude.py --results
RESET:     python tune_attitude.py --reset
CUSTOM N:  python tune_attitude.py --n 1000
"""

import sys
import os
import numpy as np
import time
import argparse

sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__),
    '..', '..', '08_gnc_monte_carlo', 'src'))

DB_PATH    = os.path.join(os.path.dirname(__file__), 'attitude_tuning.db')
STUDY_NAME = 'attitude_lqr_v1'

# ---------------------------------------------------------------
# PARAMETER SPACE — every value changed during Project 09 tuning
# ---------------------------------------------------------------

PARAM_SPACE = {
    'guidance_clamp':   [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0],
    'launch_angle_deg': [21.0, 21.2, 21.4, 21.6],
    'thrust_sustain':   [750.0, 800.0],
    'tpn_N':            [4.0, 4.5, 5.0],
    'accel_limit':      [15.0, 20.0],
    'noise_std':        [0.0, 0.01],
    'Q_angle':          [8.0],
    'Q_rate':           [11.0],
    'R_roll':           [25.0],
    'R_pitch':          [7.0],
    'R_yaw':            [30.0],
    'control_start':    [2.0],
    'tau_des':          [0.4],
    'grav_comp_below':  [1.4],
    'grav_comp_above':  [-0.2],
    'alpha_clamp_deg':  [10.0],
}


# ---------------------------------------------------------------
# Silent sim runner
# ---------------------------------------------------------------

SNAP_ATTRS = [
    'THRUST_SUSTAIN', 'ALPHA_CLAMP_DEG', 'BETA_CLAMP_DEG',
    'AERO_ALPHA_CLAMP_DEG', 'GRAV_COMP_BELOW', 'GRAV_COMP_ABOVE',
    'CONTROL_START', 'TAU_DES', 'TPN_N', 'TPN_ACCEL_LIM', 'TPN_NOISE_STD',
    'LQR_Q_DIAG', 'LQR_R_DIAG',
]


def run_fast_sim(cfg):
    try:
        import attitude_sim as sim

        # Snapshot
        snap = {k: getattr(sim, k) for k in SNAP_ATTRS if hasattr(sim, k)}

        # Apply module-level constants
        sim.THRUST_SUSTAIN       = cfg['thrust_sustain']
        sim.ALPHA_CLAMP_DEG      = cfg['guidance_clamp']
        sim.BETA_CLAMP_DEG       = cfg['guidance_clamp']
        sim.AERO_ALPHA_CLAMP_DEG = cfg['alpha_clamp_deg']
        sim.GRAV_COMP_BELOW      = cfg['grav_comp_below']
        sim.GRAV_COMP_ABOVE      = cfg['grav_comp_above']
        sim.CONTROL_START        = cfg['control_start']
        sim.TAU_DES              = cfg['tau_des']
        sim.TPN_N                = cfg['tpn_N']
        sim.TPN_ACCEL_LIM        = cfg['accel_limit']
        sim.TPN_NOISE_STD        = cfg['noise_std']

        # Apply LQR weights via module-level lists
        # LQRController reads LQR_Q_DIAG / LQR_R_DIAG at instantiation
        sim.LQR_Q_DIAG = [cfg['Q_angle'], cfg['Q_angle'], cfg['Q_angle'],
                          cfg['Q_rate'],  cfg['Q_rate'],  cfg['Q_rate']]
        sim.LQR_R_DIAG = [cfg['R_roll'], cfg['R_pitch'], cfg['R_yaw']]

        result = sim.run_attitude_sim(
            launch_angle_deg = cfg['launch_angle_deg'],
            target_pos       = np.array([2500.0, 0.0, 500.0]),
            guidance_mode    = 'TPN',
            use_guidance     = True,
            dt               = 0.01,
            t_max            = 60.0,
            verbose          = False,
            gain_schedule    = True,
        )
        miss = result['miss_distance']
        hit  = result['hit']

    except Exception:
        miss = 9999.0
        hit  = False

    finally:
        try:
            for k, v in snap.items():
                setattr(sim, k, v)
        except Exception:
            pass

    return miss, hit


# ---------------------------------------------------------------
# Optuna objective
# ---------------------------------------------------------------

def objective(trial):
    cfg = {name: trial.suggest_categorical(name, vals)
           for name, vals in PARAM_SPACE.items()}
    miss, hit = run_fast_sim(cfg)
    trial.set_user_attr('hit', hit)
    return miss


# ---------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------

def print_best(study):
    best = study.best_trial
    p    = best.params
    hit  = best.user_attrs.get('hit', False)

    print(f"\n{'='*65}")
    print(f"  BEST — {best.value:.1f} m  {'(HIT!)' if hit else ''}  "
          f"| baseline 35.7 m  | improvement {35.7 - best.value:.1f} m")
    print(f"  Trial #{best.number}  |  Total: {len(study.trials)}")
    print(f"{'='*65}")
    print(f"\n  # Engagement:")
    print(f"  launch_angle_deg = {p['launch_angle_deg']}")
    print(f"  THRUST_SUSTAIN   = {p['thrust_sustain']}")
    print(f"\n  # Guidance:")
    print(f"  TPN_N         = {p['tpn_N']}")
    print(f"  TPN_ACCEL_LIM = {p['accel_limit']}")
    print(f"  ALPHA_CLAMP_DEG = {p['guidance_clamp']}")
    print(f"  TPN_NOISE_STD = {p['noise_std']}")
    print(f"\n  # LQR weights:")
    print(f"  LQR_Q_DIAG = [{p['Q_angle']}, {p['Q_angle']}, {p['Q_angle']}, "
          f"{p['Q_rate']}, {p['Q_rate']}, {p['Q_rate']}]")
    print(f"  LQR_R_DIAG = [{p['R_roll']}, {p['R_pitch']}, {p['R_yaw']}]")
    print(f"\n  # Controller:")
    print(f"  CONTROL_START   = {p['control_start']}")
    print(f"  TAU_DES         = {p['tau_des']}")
    print(f"  GRAV_COMP_BELOW = {p['grav_comp_below']}")
    print(f"  GRAV_COMP_ABOVE = {p['grav_comp_above']}")
    print(f"  AERO_ALPHA_CLAMP_DEG = {p['alpha_clamp_deg']}")
    print(f"{'='*65}\n")


def print_top_n(study, n=10):
    trials = sorted(
        [t for t in study.trials if t.value is not None],
        key=lambda t: t.value
    )[:n]
    print(f"\n  Top {n} trials ({len(study.trials)} total):")
    print(f"  {'#':>5}  {'miss':>7}  {'angle':>6}  {'N':>4}  "
          f"{'R_p':>5}  {'Q_r':>5}  {'tau':>5}  {'thrust':>7}")
    print(f"  {'-'*57}")
    for t in trials:
        p   = t.params
        tag = ' HIT' if t.user_attrs.get('hit') else ''
        print(f"  {t.number:5d}  {t.value:7.1f}m  "
              f"{p['launch_angle_deg']:6.1f}  {p['tpn_N']:4.1f}  "
              f"{p['R_pitch']:5.1f}  {p['Q_rate']:5.1f}  "
              f"{p['tau_des']:5.2f}  {p['thrust_sustain']:7.0f}{tag}")


def print_importance(study):
    try:
        from optuna.importance import get_param_importances
        imp = get_param_importances(study)
        print(f"\n  Parameter importance:")
        for param, score in sorted(imp.items(), key=lambda x: -x[1])[:12]:
            bar = '█' * int(score * 40)
            print(f"    {param:<22} {score:.3f}  {bar}")
    except Exception:
        pass


# ---------------------------------------------------------------
# Main
# ---------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--results', action='store_true')
    parser.add_argument('--reset',   action='store_true')
    parser.add_argument('--n', type=int, default=999_999_999)
    args = parser.parse_args()

    try:
        import optuna
    except ImportError:
        print("ERROR: pip install optuna")
        sys.exit(1)

    optuna.logging.set_verbosity(optuna.logging.WARNING)
    storage = f'sqlite:///{DB_PATH}'

    if args.reset:
        if os.path.exists(DB_PATH):
            os.remove(DB_PATH)
            print(f"Deleted {DB_PATH}")
        else:
            print("No DB to reset.")

    study = optuna.create_study(
        storage        = storage,
        study_name     = STUDY_NAME,
        direction      = 'minimize',
        load_if_exists = True,
        sampler        = optuna.samplers.TPESampler(
            n_startup_trials = 30,
            seed             = 42,
        ),
    )

    n_done = len([t for t in study.trials if t.value is not None])

    if args.results:
        if n_done == 0:
            print("No completed trials yet.")
        else:
            print_top_n(study)
            print_best(study)
            print_importance(study)
        return

    print(f"\n{'='*65}")
    print(f"  Bayesian LQR Tuner — Project 09")
    print(f"  DB: {DB_PATH}")
    print(f"  Trials done: {n_done}  |  Baseline: 35.7 m")
    print(f"  Ctrl+C to stop — resumes automatically")
    print(f"{'='*65}\n")

    t_start   = time.time()
    best_miss = study.best_value if n_done > 0 else np.inf
    session_n = 0

    def callback(study, trial):
        nonlocal best_miss, session_n
        session_n += 1
        miss = trial.value if trial.value is not None else 9999.0
        hit  = trial.user_attrs.get('hit', False)

        if miss < best_miss:
            best_miss = miss
            marker = ' *** NEW BEST ***'
        else:
            marker = ' *** HIT! ***' if hit else ''

        total   = len([t for t in study.trials if t.value is not None])
        elapsed = time.time() - t_start
        rate    = elapsed / session_n

        if session_n % 10 == 0 or miss < 25.0 or hit:
            print(f"  [{session_n:4d} | total {total:5d}]  "
                  f"miss={miss:7.1f}m  best={best_miss:7.1f}m  "
                  f"{rate:.2f}s/trial{marker}")

    try:
        study.optimize(
            objective,
            n_trials          = args.n,
            callbacks         = [callback],
            show_progress_bar = False,
        )
    except KeyboardInterrupt:
        print(f"\n  Stopped — {session_n} trials saved.")

    if len([t for t in study.trials if t.value is not None]) > 0:
        print_top_n(study)
        print_best(study)
        print_importance(study)

    total = time.time() - t_start
    print(f"  {session_n} trials in {total:.0f}s "
          f"({total/max(session_n,1):.2f}s/trial)")
    print(f"  DB: {DB_PATH}\n")


if __name__ == '__main__':
    main()