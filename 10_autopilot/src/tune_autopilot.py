"""
tune_autopilot.py -- Bayesian Autopilot Parameter Optimization
Project 10 -- Three-Loop Autopilot

Uses Optuna with TPE (Tree-structured Parzen Estimator) — the same algorithm
used to tune neural networks at scale. Smarter than random/grid search because
it builds a probabilistic model of the objective and focuses sampling where
miss distance is lowest.

PERSISTENCE: Results are stored in autopilot_tuning.db (SQLite).
Kill the run at any time. Re-run and it picks up exactly where it left off,
with the full history informing the next trials. Gets smarter every session.

INSTALL:
    pip install optuna

FIRST RUN:
    python tune_autopilot.py

RESUME (after killing or next session):
    python tune_autopilot.py          # automatically resumes

VIEW RESULTS WITHOUT RUNNING:
    python tune_autopilot.py --results

RESET AND START FRESH:
    python tune_autopilot.py --reset
"""

import sys
import os
import numpy as np
import time
import argparse

from scipy import io

sys.path.insert(0, os.path.dirname(__file__))

DB_PATH    = os.path.join(os.path.dirname(__file__), 'autopilot_tuning.db')
STUDY_NAME = 'autopilot_v1'
N_TRIALS   = 500    # trials per session — increase freely, resumes on next run

# ---------------------------------------------------------------
# PARAMETER SPACE — every value ever changed, defensible ranges
# ---------------------------------------------------------------

PARAM_SPACE = {
    'ka_nom':        [0.8],
    'ki_a_nom':      [0.45],
    'kp_rate_nom':   [-0.22, -0.25, -0.28, -0.30, -0.32],
    'kd_rate_nom':   [0.0, 0.003, 0.005, 0.008],
    'kp_roll_nom':   [0.0, -0.01, -0.02],
    'omega_cmd_max': [1.5, 1.75, 2.0, 2.5, 3.0, 3.5],
    'int_clip':      [1.2],
    'int_leakage':   [1.0],
    'q_bar_floor':   [12000.0],
    'scale_min':     [0.6],
    'scale_max':     [2.5],
    'ki_a_scale':    [0.6],
    'roll_damp':     [1.5],
    'eom_clip':      [200.0],
    'control_start': [2.5],
}


# ---------------------------------------------------------------
# Silent sim runner — identical across all search scripts
# ---------------------------------------------------------------

def run_fast_sim(cfg):
    import autopilot as ap_mod
    import autopilot_sim as sim_mod
    from quaternion import quat_norm

    snap_ap = {k: getattr(ap_mod, k) for k in
               ['OMEGA_CMD_MAX', 'INT_CLIP', 'INT_LEAKAGE',
                'Q_BAR_FLOOR', 'SCALE_MIN', 'SCALE_MAX',
                'KI_A_SCALE', 'PRINT_EVERY']}
    snap_cls = {k: getattr(ap_mod.AutopilotController, k) for k in
                ['Ka_nom', 'Ki_a_nom', 'Kp_rate_nom', 'Kd_rate_nom', 'Kp_roll_nom']}
    orig_rk4     = sim_mod.rk4_step
    orig_compute = ap_mod.AutopilotController.compute

    try:
        import io
        sys.stdout = io.StringIO()   # silence all prints during sim
        ap_mod.OMEGA_CMD_MAX = cfg['omega_cmd_max']
        ap_mod.INT_CLIP      = cfg['int_clip']
        ap_mod.INT_LEAKAGE   = cfg['int_leakage']
        ap_mod.Q_BAR_FLOOR   = cfg['q_bar_floor']
        ap_mod.SCALE_MIN     = cfg['scale_min']
        ap_mod.SCALE_MAX     = cfg['scale_max']
        ap_mod.KI_A_SCALE    = cfg['ki_a_scale']
        ap_mod.PRINT_EVERY   = 999999

        ap_mod.AutopilotController.Ka_nom      = cfg['ka_nom']
        ap_mod.AutopilotController.Ki_a_nom    = cfg['ki_a_nom']
        ap_mod.AutopilotController.Kp_rate_nom = cfg['kp_rate_nom']
        ap_mod.AutopilotController.Kd_rate_nom = cfg['kd_rate_nom']
        ap_mod.AutopilotController.Kp_roll_nom = cfg['kp_roll_nom']

        roll_damp = cfg['roll_damp']
        eom_clip  = np.radians(cfg['eom_clip'])

        def patched_rk4(t, state, fin_deflections, dt):
            from autopilot_sim import derivatives
            k1 = derivatives(t,        state,            fin_deflections)
            k2 = derivatives(t + dt/2, state + dt/2*k1,  fin_deflections)
            k3 = derivatives(t + dt/2, state + dt/2*k2,  fin_deflections)
            k4 = derivatives(t + dt,   state + dt*k3,    fin_deflections)
            ns = state + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
            ns[6:10]  = quat_norm(ns[6:10])
            ns[10:13] = np.clip(ns[10:13], -eom_clip, eom_clip)
            return ns

        def patched_compute(self, euler, omega, accel_body, accel_cmd, dt=0.01):
            import numpy as _np
            from autopilot import _euler_to_dcm, DELTA_MAX_RAD
            _OMEGA = ap_mod.OMEGA_CMD_MAX
            _CLIP  = ap_mod.INT_CLIP
            _LEAK  = ap_mod.INT_LEAKAGE
            dcm            = _euler_to_dcm(euler)
            accel_cmd_body = dcm @ accel_cmd
            a_meas         = accel_body[1:3]
            a_cmd_lat      = accel_cmd_body[1:3]
            a_error        = a_cmd_lat - a_meas
            self.int_error = _LEAK * self.int_error + a_error * dt
            self.int_error = _np.clip(self.int_error, -_CLIP, _CLIP)
            raw            = self.Ka * a_error + self.Ki_a * self.int_error
            clipped        = _np.clip(raw, -_OMEGA, _OMEGA)
            omega_cmd      = _np.array([0.0, clipped[0], clipped[1]])
            rate_err       = omega_cmd - omega
            rate_err[0]   *= roll_damp
            omega_dot      = (omega - self.prev_omega) / max(dt, 1e-6)
            self.prev_omega = omega.copy()
            self._call_count = getattr(self, '_call_count', 0) + 1
            u = _np.zeros(3)
            u[0] = self.Kp_roll * rate_err[0]
            u[1] = self.Kp_rate * rate_err[1] + self.Kd_rate * omega_dot[1]
            u[2] = self.Kp_rate * rate_err[2] + self.Kd_rate * omega_dot[2]
            return _np.clip(u, -DELTA_MAX_RAD, DELTA_MAX_RAD)

        sim_mod.rk4_step = patched_rk4
        ap_mod.AutopilotController.compute = patched_compute

        # Patch CONTROL_START
        orig_cs = getattr(sim_mod, 'CONTROL_START', None)
        if orig_cs is not None:
            sim_mod.CONTROL_START = cfg.get('control_start', orig_cs)

        result = sim_mod.run_autopilot_sim(
            launch_angle_deg=21.2,
            target_pos=np.array([2500.0, 0.0, 500.0]),
            use_guidance=True, dt=0.01, t_max=60.0,
            verbose=False, gain_schedule=True,
        )
        miss = result['miss_distance']
        hit  = result['hit']

    except Exception:
        miss = 9999.0
        hit  = False

    finally:
        sys.stdout = sys.__stdout__  # restore stdout
        for k, v in snap_ap.items():
            setattr(ap_mod, k, v)
        for k, v in snap_cls.items():
            setattr(ap_mod.AutopilotController, k, v)
        try:
            sim_mod.rk4_step = orig_rk4
            ap_mod.AutopilotController.compute = orig_compute
            if orig_cs is not None:
                sim_mod.CONTROL_START = orig_cs
        except Exception:
            pass

    return miss, hit


# ---------------------------------------------------------------
# Optuna objective
# ---------------------------------------------------------------

def objective(trial):
    cfg = {}
    for name, vals in PARAM_SPACE.items():
        # Optuna suggest_categorical works directly with discrete lists
        cfg[name] = trial.suggest_categorical(name, vals)

    miss, hit = run_fast_sim(cfg)

    # Store hit flag as user attribute for reporting
    trial.set_user_attr('hit', hit)
    trial.set_user_attr('miss_distance', miss)

    return miss   # minimize


# ---------------------------------------------------------------
# Print helpers
# ---------------------------------------------------------------

def print_best(study):
    best = study.best_trial
    p    = best.params
    miss = best.value
    hit  = best.user_attrs.get('hit', False)

    print(f"\n{'='*65}")
    print(f"  BEST so far — {miss:.1f} m {'(HIT!)' if hit else ''}")
    print(f"  Trial #{best.number}  |  "
          f"Total trials completed: {len(study.trials)}")
    print(f"{'='*65}")
    print(f"\n  # autopilot.py gains block:")
    print(f"  Ka_nom      =  {p['ka_nom']}")
    print(f"  Ki_a_nom    =  {p['ki_a_nom']}")
    print(f"  Kp_rate_nom = {p['kp_rate_nom']}")
    print(f"  Kd_rate_nom =  {p['kd_rate_nom']}")
    print(f"  Kp_roll_nom = {p['kp_roll_nom']}")
    print(f"\n  # autopilot.py TUNABLE CONSTANTS:")
    print(f"  OMEGA_CMD_MAX = {p['omega_cmd_max']}")
    print(f"  INT_LEAKAGE   = {p['int_leakage']}")
    print(f"  INT_CLIP      = {p['int_clip']}")
    print(f"  Q_BAR_FLOOR   = {p['q_bar_floor']}")
    print(f"  SCALE_MIN     = {p['scale_min']}")
    print(f"  SCALE_MAX     = {p['scale_max']}")
    print(f"  KI_A_SCALE    = {p['ki_a_scale']}")
    print(f"\n  # compute() roll damp:")
    print(f"  rate_error[0] *= {p['roll_damp']}")
    print(f"\n  # rk4_step EOM clip:")
    print(f"  np.radians({p['eom_clip']:.0f})")
    print(f"\n  # CONTROL_START:")
    print(f"  CONTROL_START = {p['control_start']}")
    print(f"{'='*65}\n")


def print_top_n(study, n=10):
    trials = sorted(
        [t for t in study.trials if t.value is not None],
        key=lambda t: t.value
    )[:n]
    print(f"\n  Top {n} trials ({len(study.trials)} total evaluated):")
    print(f"  {'#':>5}  {'miss':>7}  {'Ka':>5}  {'Kp_r':>6}  "
          f"{'Kp_roll':>7}  {'oCMD':>5}  {'EOM':>5}  {'leak':>5}")
    print(f"  {'-'*58}")
    for t in trials:
        p = t.params
        hit_tag = ' HIT' if t.user_attrs.get('hit') else ''
        print(f"  {t.number:5d}  {t.value:7.1f}m  "
              f"{p['ka_nom']:5.3f}  {p['kp_rate_nom']:6.3f}  "
              f"{p['kp_roll_nom']:7.3f}  {p['omega_cmd_max']:5.2f}  "
              f"{p['eom_clip']:5.0f}  {p['int_leakage']:5.2f}"
              f"{hit_tag}")


def print_importance(study):
    try:
        from optuna.importance import get_param_importances
        imp = get_param_importances(study)
        print(f"\n  Parameter importance (higher = more impact on miss distance):")
        for param, score in sorted(imp.items(), key=lambda x: -x[1])[:10]:
            bar = '█' * int(score * 40)
            print(f"    {param:<20} {score:.3f}  {bar}")
    except Exception:
        pass  # importance requires enough completed trials


# ---------------------------------------------------------------
# Main
# ---------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--results', action='store_true',
                        help='Print current best results without running')
    parser.add_argument('--reset', action='store_true',
                        help='Delete DB and start fresh')
    parser.add_argument('--n', type=int, default=N_TRIALS,
                        help=f'Number of trials this session (default {N_TRIALS})')
    args = parser.parse_args()

    try:
        import optuna
    except ImportError:
        print("ERROR: optuna not installed. Run:  pip install optuna")
        sys.exit(1)

    # Silence optuna's own logging
    optuna.logging.set_verbosity(optuna.logging.WARNING)

    storage = f'sqlite:///{DB_PATH}'

    if args.reset:
        if os.path.exists(DB_PATH):
            os.remove(DB_PATH)
            print(f"Deleted {DB_PATH} — starting fresh.")
        else:
            print("No DB found — nothing to reset.")

    # Load or create study
    study = optuna.create_study(
        storage    = storage,
        study_name = STUDY_NAME,
        direction  = 'minimize',
        load_if_exists = True,
        sampler    = optuna.samplers.TPESampler(
            n_startup_trials = 30,   # random exploration before TPE kicks in
            seed             = 42,
        ),
        pruner = optuna.pruners.NopPruner(),
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
    print(f"  Bayesian Autopilot Tuner (TPE / Optuna)")
    print(f"  DB: {DB_PATH}")
    print(f"  Study: {STUDY_NAME}")
    print(f"  Trials already completed: {n_done}")
    print(f"  Trials this session: {args.n}")
    print(f"  First {30 - min(n_done, 30)} trials: random exploration")
    print(f"  Remaining: TPE (learns from history)")
    print(f"  Ctrl+C to stop — resumes next run automatically")
    print(f"{'='*65}\n")

    t_start    = time.time()
    best_miss  = study.best_value if n_done > 0 else np.inf
    session_n  = 0

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

        total_done = len([t for t in study.trials if t.value is not None])
        elapsed    = time.time() - t_start
        rate       = elapsed / session_n

        if session_n % 10 == 0 or miss < 50.0 or hit:
            print(f"  [{session_n:4d}/{args.n} | total {total_done:5d}]  "
                  f"miss={miss:7.1f}m  best={best_miss:7.1f}m  "
                  f"{rate:.2f}s/trial{marker}")

    try:
        study.optimize(
            objective,
            n_trials  = args.n,
            callbacks = [callback],
            show_progress_bar = False,
        )
    except KeyboardInterrupt:
        print(f"\n  Interrupted — {session_n} trials this session saved to DB.")

    # Final report
    n_done_now = len([t for t in study.trials if t.value is not None])
    if n_done_now > 0:
        print_top_n(study, n=10)
        print_best(study)
        print_importance(study)

    total = time.time() - t_start
    print(f"  Session: {session_n} trials in {total:.0f}s "
          f"({total/max(session_n,1):.2f}s/trial)")
    print(f"  All results stored in: {DB_PATH}")
    print(f"  Resume anytime: python tune_autopilot.py\n")


if __name__ == '__main__':
    main()