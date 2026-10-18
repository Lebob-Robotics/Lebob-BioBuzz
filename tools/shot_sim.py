#!/usr/bin/env python3
"""
shot_sim.py - offline robust-shot simulator for an FTC hooded double-flywheel shooter.

METHOD (the "robust shot" approach used by FRC 4414 and 1690)
-------------------------------------------------------------
For every operating point (d, v_r) we want one (hood angle, exit speed) pair to
command.  Instead of picking any shot that scores, we pick the one that is the
MOST FORGIVING of real-world error:

  1. Simulate a dense inner grid of (theta, v_exit) shots with RK4 (gravity,
     quadratic drag, optional Magnus lift) in the 2D vertical plane
     perpendicular to the goal window (x toward the goal, z up).
  2. Mark each shot valid if the FIRST crossing of the ball centre through the
     window plane lies on the window segment, shrunk by (ball radius + margin)
     at both edges.  Crossing below the lip = front wall hit, above the top
     edge = top hit / overshoot; both invalid.
  3. Treat the valid mask on the (theta, v) grid as an image.  For each valid
     cell compute the Euclidean distance to the nearest invalid cell in
     normalised units (dtheta/sigma_theta, dv/sigma_v).  Grid edges count as
     invalid (hood and speed limits are hard).  The argmax is the most robust
     shot: it tolerates the largest combined angle/speed error.
  4. Repeat over an outer grid of d (distance shaft -> window lip midpoint)
     and v_r (robot closing speed, added to the ball's horizontal velocity).
  5. Fit 2D polynomials theta*(d, v_r), v*(d, v_r) (and tof) by least squares
     over the feasible points, and print Java constants to paste in the robot
     code.

Polynomial term order (identical for every fitted quantity):
    terms (i, j) with i + j <= degree, ordered by i ascending then j ascending
    value = sum_k coeff[k] * d^i * v_r^j      (d in m, v_r in m/s, RAW units)
e.g. degree 2:  1, vr, vr^2, d, d*vr, d^2

Geometry: shaft at x = 0, z = shaft_height.  Window lip midpoint at (d, lip_z).
The window face leans away from the shooter by `lean_deg` from vertical.
Spin: backspin, S = (r1 - r2)/(r1 + r2); exit speed = omega*(r1 + r2)/2.

USAGE (Windows: `py`, needs numpy; matplotlib only for --plot)
    py tools/shot_sim.py                       # full sweep, fit, Java snippet
    py tools/shot_sim.py --csv out.csv         # also write the full table
    py tools/shot_sim.py --single 2.5 0.8      # one (d, v_r) point in detail
    py tools/shot_sim.py --magnus 0.15         # enable Magnus lift (CL)
    py tools/shot_sim.py --plot --d 2.5 --vr 0 # sweep + plots
    py tools/shot_sim.py --workers 8           # parallel sweep (default min(cpus, 8))
    py tools/shot_sim.py --degree 4 --dtheta 1 --dv 0.1   # faster/coarser
"""

from __future__ import annotations

import argparse
import dataclasses
import math
import os
import sys
import time
from dataclasses import dataclass

import numpy as np

IN = 0.0254


# ----------------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------------
@dataclass
class Config:
    # --- ball ("Pollen") ---
    ball_diameter: float = 2.8 * IN        # m
    ball_mass: float = 0.0249              # kg  TODO: mass unknown, measure and update
    # --- goal ("Cell" window) ---
    window_width: float = 20 * IN          # m (not used in the 2D plane, informational)
    window_lip_z: float = 53.5 * IN        # m, bottom edge height (1.3589)
    window_length: float = 14 * IN         # m, along the face (0.3556)
    window_lean_deg: float = 30.0          # deg from vertical, leaning away from shooter
    box_depth: float = 12 * IN             # m, informational (bounce-outs ignored)
    # --- shooter ---
    r_main: float = 0.025                  # m, 50 mm main wheel radius (r1)
    r_hood: float = 0.0127                 # m, 25.4 mm hood wheel radius (r2)
    shaft_height: float = 0.2222           # m, main wheel shaft centre height
    # --- environment / aerodynamics ---
    air_density: float = 1.2               # kg/m^3
    cd: float = 0.47
    cl: float = 0.0                        # Magnus lift coefficient, 0 = off (--magnus)
    g: float = 9.81
    # --- search space ---
    theta_min: float = 51.0                # deg  real hood range 51..99; >90 fires backwards
    theta_max: float = 89.0                # deg
    v_min: float = 2.0                     # m/s
    v_max: float = 8.0                     # m/s  TODO: real ceiling after wheel slip (free speed ~8.3)
    dtheta: float = 0.5                    # deg grid step
    dv: float = 0.05                       # m/s grid step
    d_min: float = 0.5
    d_max: float = 3.5
    d_step: float = 0.1
    vr_min: float = -1.5
    vr_max: float = 1.5
    vr_step: float = 0.1
    margin: float = 0.01                   # m clearance per window edge
    sigma_theta: float = 1.0               # deg
    sigma_v: float = 0.10                  # m/s
    # --- numerics / fit ---
    dt: float = 0.002                      # s
    t_max: float = 6.0                     # s, hard cap on flight time
    degree: int = 3
    weight: bool = False                   # weight fit by robustness margin
    tie_tol: float = 0.0                   # sigma; cells within this of the best margin count as ties
    smooth_eps: float = 0.15               # candidate cells: rob >= (1-eps)*max_rob
    smooth_iters: int = 3                  # smoothing iterations (0 = off)
    verify: bool = True                    # re-simulate the fitted shots after the fit
    workers: int = 1                       # process count for the sweep (not physical)

    # derived helpers
    @property
    def rb(self) -> float:
        return self.ball_diameter / 2.0

    @property
    def area(self) -> float:
        return math.pi * self.rb ** 2

    @property
    def k_drag(self) -> float:             # a_drag = -k_drag*|v|*v
        return 0.5 * self.air_density * self.area * self.cd / self.ball_mass

    @property
    def k_mag(self) -> float:              # a_lift = k_mag*|v|*(-vz, vx)
        return 0.5 * self.air_density * self.area * self.cl / self.ball_mass

    @property
    def spin_param(self) -> float:
        return (self.r_main - self.r_hood) / (self.r_main + self.r_hood)

    def thetas(self) -> np.ndarray:
        n = int(round((self.theta_max - self.theta_min) / self.dtheta)) + 1
        return self.theta_min + self.dtheta * np.arange(n)

    def speeds(self) -> np.ndarray:
        n = int(round((self.v_max - self.v_min) / self.dv)) + 1
        return self.v_min + self.dv * np.arange(n)

    def d_values(self) -> np.ndarray:
        n = int(round((self.d_max - self.d_min) / self.d_step)) + 1
        return self.d_min + self.d_step * np.arange(n)

    def vr_values(self) -> np.ndarray:
        n = int(round((self.vr_max - self.vr_min) / self.vr_step)) + 1
        return self.vr_min + self.vr_step * np.arange(n)


# ----------------------------------------------------------------------------
# Integrator (vectorised RK4)
# ----------------------------------------------------------------------------
def _accel(vx, vz, kd, km, g):
    sp = np.sqrt(vx * vx + vz * vz)
    ax = -kd * sp * vx - km * sp * vz
    az = -g - kd * sp * vz + km * sp * vx
    return ax, az


def rk4_step(x, z, vx, vz, dt, kd, km, g):
    """One RK4 step for gravity + quadratic drag + optional Magnus."""
    a1x, a1z = _accel(vx, vz, kd, km, g)
    v2x, v2z = vx + 0.5 * dt * a1x, vz + 0.5 * dt * a1z
    a2x, a2z = _accel(v2x, v2z, kd, km, g)
    v3x, v3z = vx + 0.5 * dt * a2x, vz + 0.5 * dt * a2z
    a3x, a3z = _accel(v3x, v3z, kd, km, g)
    v4x, v4z = vx + dt * a3x, vz + dt * a3z
    a4x, a4z = _accel(v4x, v4z, kd, km, g)
    nx = x + dt / 6.0 * (vx + 2 * v2x + 2 * v3x + v4x)
    nz = z + dt / 6.0 * (vz + 2 * v2z + 2 * v3z + v4z)
    nvx = vx + dt / 6.0 * (a1x + 2 * a2x + 2 * a3x + a4x)
    nvz = vz + dt / 6.0 * (a1z + 2 * a2z + 2 * a3z + a4z)
    return nx, nz, nvx, nvz


def launch_state(cfg: Config, theta_deg, v, vr):
    """Release position (shaft at x=0) and ground-frame velocity."""
    th = np.radians(theta_deg)
    c, s = np.cos(th), np.sin(th)
    off = cfg.r_main + cfg.rb
    x0 = -off * s                         # (r1+rb)*(-sin, cos) from the shaft
    z0 = cfg.shaft_height + off * c
    return x0, z0, v * c + vr, v * s


class Window:
    """Window segment geometry in the (x, z) plane for a given d."""

    def __init__(self, cfg: Config, d: float):
        a = math.radians(cfg.window_lean_deg)
        self.lip = (d, cfg.window_lip_z)
        self.u = (math.sin(a), math.cos(a))        # along the face, upward
        self.n = (math.cos(a), -math.sin(a))       # shooter side -> inside
        self.length = cfg.window_length
        self.top = (d + self.length * self.u[0], cfg.window_lip_z + self.length * self.u[1])

    def signed_dist(self, x, z):
        return (x - self.lip[0]) * self.n[0] + (z - self.lip[1]) * self.n[1]

    def along(self, x, z):
        return (x - self.lip[0]) * self.u[0] + (z - self.lip[1]) * self.u[1]


def simulate_grid(cfg: Config, d: float, vr: float, thetas, speeds):
    """Simulate every (theta, speed) shot at once.

    Returns (valid[nth, nv] bool, tof[nth, nv] float (nan if not valid)).
    """
    nth, nv = len(thetas), len(speeds)
    th = np.repeat(thetas, nv)
    v = np.tile(speeds, nth)
    x, z, vx, vz = launch_state(cfg, th, v, vr)
    win = Window(cfg, d)
    sd = win.signed_dist(x, z)

    valid = np.zeros(nth * nv, dtype=bool)
    tof = np.full(nth * nv, np.nan)
    idx = np.nonzero(sd < 0.0)[0]          # shots already past the plane are invalid
    x, z, vx, vz, sd = x[idx], z[idx], vx[idx], vz[idx], sd[idx]

    kd, km, g, dt = cfg.k_drag, cfg.k_mag, cfg.g, cfg.dt
    s_lo = cfg.rb + cfg.margin
    s_hi = cfg.window_length - cfg.rb - cfg.margin
    x_stop = d + 1.0
    t = 0.0
    for _ in range(int(cfg.t_max / dt)):
        if idx.size == 0:
            break
        nx, nz, nvx, nvz = rk4_step(x, z, vx, vz, dt, kd, km, g)
        nsd = win.signed_dist(nx, nz)
        crossed = nsd >= 0.0
        if crossed.any():
            ci = np.nonzero(crossed)[0]
            f = -sd[ci] / (nsd[ci] - sd[ci])              # linear interpolation
            cx = x[ci] + f * (nx[ci] - x[ci])
            cz = z[ci] + f * (nz[ci] - z[ci])
            s = win.along(cx, cz)
            ok = (s >= s_lo) & (s <= s_hi)
            gi = idx[ci[ok]]
            valid[gi] = True
            tof[gi] = t + f[ok] * dt
        done = crossed | (nz < 0.0) | (nx > x_stop)
        t += dt
        if done.any():
            keep = ~done
            idx, x, z, vx, vz, sd = (idx[keep], nx[keep], nz[keep], nvx[keep], nvz[keep], nsd[keep])
        else:
            x, z, vx, vz, sd = nx, nz, nvx, nvz, nsd
    return valid.reshape(nth, nv), tof.reshape(nth, nv)


def trajectories(cfg: Config, d: float, vr: float, thetas, speeds, t_end=3.0):
    """Record full paths (for plotting).  Returns array [nshots, nsteps+1, 2]."""
    thetas, speeds = np.atleast_1d(thetas).astype(float), np.atleast_1d(speeds).astype(float)
    x, z, vx, vz = launch_state(cfg, thetas, speeds, vr)
    out = [np.stack([x, z], axis=-1)]
    for _ in range(int(t_end / cfg.dt)):
        x, z, vx, vz = rk4_step(x, z, vx, vz, cfg.dt, cfg.k_drag, cfg.k_mag, cfg.g)
        out.append(np.stack([x, z], axis=-1))
    return np.stack(out, axis=1)


# ----------------------------------------------------------------------------
# Robustness (distance transform, plain numpy)
# ----------------------------------------------------------------------------
def robustness_map(valid: np.ndarray, h0: float, h1: float) -> np.ndarray:
    """Distance from each valid cell to the nearest invalid cell, in normalised units.

    h0, h1 = cell size along axis 0 / axis 1 in sigma units.  The grid is padded
    with an invalid ring so cells outside the grid count as invalid.  Exact
    separable Euclidean distance transform (1D sweeps along axis 0, then a
    min-plus pass along axis 1).  Invalid cells get 0.
    """
    m = np.pad(valid, 1, constant_values=False)
    n0, n1 = m.shape
    big = 1e9
    f = np.where(m, big, 0.0)
    for i in range(1, n0):                       # forward sweep, vectorised over columns
        np.minimum(f[i], f[i - 1] + h0, out=f[i])
    for i in range(n0 - 2, -1, -1):              # backward sweep
        np.minimum(f[i], f[i + 1] + h0, out=f[i])
    f2 = f * f
    d2 = f2.copy()
    for s in range(1, n1):
        w = (h1 * s) ** 2
        np.minimum(d2[:, s:], f2[:, :-s] + w, out=d2[:, s:])
        np.minimum(d2[:, :-s], f2[:, s:] + w, out=d2[:, :-s])
    dist = np.sqrt(d2)
    dist[~m] = 0.0
    return dist[1:-1, 1:-1]


@dataclass
class PointResult:
    d: float
    vr: float
    feasible: bool = False
    theta: float = float("nan")
    v: float = float("nan")
    margin: float = float("nan")
    tof: float = float("nan")
    nvalid: int = 0
    verified: bool = True      # False if a fit shot at/around this point missed
    max_rob: float = float("nan")
    # candidate cells for smoothing: theta, v, rob, tof arrays (rob >= (1-eps)*max_rob)
    cand: tuple = None


def solve_point(cfg: Config, d: float, vr: float, thetas=None, speeds=None, want_maps=False):
    """Simulate and pick the most robust shot for one (d, vr)."""
    thetas = cfg.thetas() if thetas is None else thetas
    speeds = cfg.speeds() if speeds is None else speeds
    valid, tof = simulate_grid(cfg, d, vr, thetas, speeds)
    res = PointResult(d, vr, nvalid=int(valid.sum()))
    rob = None
    if res.nvalid:
        rob = robustness_map(valid, cfg.dtheta / cfg.sigma_theta, cfg.dv / cfg.sigma_v)
        best = rob.max()
        ties = np.argwhere(rob >= best - cfg.tie_tol - 1e-9)
        # plateau tie-break: cell nearest the centroid of the tied cells
        cen = ties.mean(axis=0)
        i, j = ties[np.argmin(((ties - cen) ** 2).sum(axis=1))]
        res.feasible = True
        res.theta, res.v = float(thetas[i]), float(speeds[j])
        res.margin, res.tof = float(rob[i, j]), float(tof[i, j])
        res.max_rob = float(best)
        ci, cj = np.nonzero(rob >= (1.0 - cfg.smooth_eps) * best - 1e-9)
        res.cand = (thetas[ci], speeds[cj], rob[ci, cj], tof[ci, cj])
    if want_maps:
        return res, valid, rob, tof
    return res


# ----------------------------------------------------------------------------
# Sweep and fit
# ----------------------------------------------------------------------------
def _solve_row(args):
    """Worker: solve every vr for one d (top-level so it can be pickled)."""
    cfg, d = args
    thetas, speeds = cfg.thetas(), cfg.speeds()
    return [solve_point(cfg, float(d), float(vr), thetas, speeds) for vr in cfg.vr_values()]


def sweep(cfg: Config):
    ds, vrs = cfg.d_values(), cfg.vr_values()
    results = []
    t0 = time.time()
    jobs = [(cfg, float(d)) for d in ds]
    if cfg.workers > 1:
        import multiprocessing as mp
        pool = mp.Pool(cfg.workers)
        rows = pool.imap(_solve_row, jobs)       # ordered, so the table stays sorted
    else:
        pool, rows = None, map(_solve_row, jobs)
    for a, (d, row) in enumerate(zip(ds, rows)):
        results.extend(row)
        nf = sum(r.feasible for r in row)
        el = time.time() - t0
        eta = el / (a + 1) * (len(ds) - a - 1)
        print(f"  d={d:4.2f} m  [{a + 1:2d}/{len(ds)}]  feasible {nf:2d}/{len(vrs)}  "
              f"elapsed {el:6.1f}s  eta {eta:6.1f}s", flush=True)
    if pool is not None:
        pool.close()
        pool.join()
    return results


def poly_terms(degree: int):
    """(i, j) with i + j <= degree, ordered by i then j."""
    return [(i, j) for i in range(degree + 1) for j in range(degree + 1 - i)]


def design(d, vr, terms):
    d, vr = np.asarray(d, float), np.asarray(vr, float)
    return np.stack([d ** i * vr ** j for i, j in terms], axis=-1)


def fit_poly(d, vr, y, terms, w=None):
    A = design(d, vr, terms)
    if w is not None:
        sw = np.sqrt(w)
        coef, *_ = np.linalg.lstsq(A * sw[:, None], y * sw, rcond=None)
    else:
        coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    resid = A @ coef - y
    return coef, resid


# ----------------------------------------------------------------------------
# Checks
# ----------------------------------------------------------------------------
def fit_all(cfg: Config, results, degree):
    """Fit theta, v, tof over feasible points.  Returns (feas, fits{name: (coef, resid)})."""
    feas = np.array([r.feasible for r in results])
    terms = poly_terms(degree)
    fd = np.array([r.d for r in results])[feas]
    fv = np.array([r.vr for r in results])[feas]
    w = np.array([r.margin for r in results])[feas] if cfg.weight else None
    fits = {}
    for name in ("theta", "v", "tof"):
        y = np.array([getattr(r, name) for r in results])[feas]
        fits[name] = fit_poly(fd, fv, y, terms, w)
    return feas, fits


def resid_line(fits, tag):
    t, v = fits["theta"][1], fits["v"][1]
    return (f"{tag}: theta max {np.abs(t).max():.3f} / RMS {np.sqrt((t ** 2).mean()):.3f} deg, "
            f"v max {np.abs(v).max():.3f} / RMS {np.sqrt((v ** 2).mean()):.3f} m/s")


def smooth_selection(cfg: Config, results):
    """Iteratively pull each point's choice toward the polynomial prediction.

    Candidates are cells with rob >= (1-eps)*max_rob; the one nearest the
    current fit (in sigma-normalised theta, v units) is chosen, then refit.
    """
    terms = poly_terms(cfg.degree)
    feas, fits = fit_all(cfg, results, cfg.degree)
    print("[smooth] " + resid_line(fits, "pass 1 (argmax)"))
    for it in range(1, cfg.smooth_iters + 1):
        for r in results:
            if not r.feasible:
                continue
            A = design(np.array([r.d]), np.array([r.vr]), terms)[0]
            pt, pv = float(A @ fits["theta"][0]), float(A @ fits["v"][0])
            ct, cv, cr, cf = r.cand
            k = int(np.argmin(((ct - pt) / cfg.sigma_theta) ** 2 + ((cv - pv) / cfg.sigma_v) ** 2))
            r.theta, r.v, r.margin, r.tof = float(ct[k]), float(cv[k]), float(cr[k]), float(cf[k])
        feas, fits = fit_all(cfg, results, cfg.degree)
        ratio = np.array([r.margin / r.max_rob for r in results if r.feasible])
        print("[smooth] " + resid_line(fits, f"iter {it}") +
              f" | margin kept: mean {ratio.mean():.3f}, min {ratio.min():.3f} of max")
    return feas, fits


def _verify_item(args):
    """Worker: simulate the fitted shot at (d, vr).  Returns (hit, margin, theta, v)."""
    cfg, d, vr, c_theta, c_v, terms = args
    A = design(np.array([d]), np.array([vr]), terms)[0]
    th, v = float(A @ c_theta), float(A @ c_v)
    if not (cfg.theta_min <= th <= cfg.theta_max and cfg.v_min <= v <= cfg.v_max):
        return False, float("nan"), th, v          # outside hood / speed limits
    hit, _ = simulate_grid(cfg, d, vr, np.array([th]), np.array([v]))
    if not hit[0, 0]:
        return False, float("nan"), th, v
    thetas, speeds = cfg.thetas(), cfg.speeds()
    valid, _ = simulate_grid(cfg, d, vr, thetas, speeds)
    rob = robustness_map(valid, cfg.dtheta / cfg.sigma_theta, cfg.dv / cfg.sigma_v)
    i = int(np.clip(round((th - cfg.theta_min) / cfg.dtheta), 0, len(thetas) - 1))
    j = int(np.clip(round((v - cfg.v_min) / cfg.dv), 0, len(speeds) - 1))
    return True, float(rob[i, j]), th, v


def verify_fit(cfg: Config, results, fits):
    """Re-simulate fitted shots at feasible grid points and grid-cell midpoints.

    A miss at a grid point marks it unverified; a miss at a midpoint (d+h/2, vr+h/2)
    marks its four surrounding grid points unverified.  Unverified points are removed
    from the printed VR_MIN/VR_MAX domain.
    """
    ds, vrs = cfg.d_values(), cfg.vr_values()
    nv = len(vrs)
    terms = poly_terms(cfg.degree)
    ct, cv = fits["theta"][0], fits["v"][0]
    items, owners = [], []
    for a, d in enumerate(ds):
        for b, vr in enumerate(vrs):
            r = results[a * nv + b]
            if r.feasible:
                items.append((cfg, float(d), float(vr), ct, cv, terms))
                owners.append(("grid", [a * nv + b]))
            if a + 1 < len(ds) and b + 1 < nv:
                corners = [a * nv + b, a * nv + b + 1, (a + 1) * nv + b, (a + 1) * nv + b + 1]
                if all(results[k].feasible for k in corners):
                    items.append((cfg, float(d + cfg.d_step / 2), float(vr + cfg.vr_step / 2), ct, cv, terms))
                    owners.append(("mid", corners))
    print(f"[verify] re-simulating {len(items)} fitted shots (grid points + midpoints)...", flush=True)
    t0 = time.time()
    if cfg.workers > 1:
        import multiprocessing as mp
        with mp.Pool(cfg.workers) as pool:
            out = pool.map(_verify_item, items, chunksize=8)
    else:
        out = [_verify_item(it) for it in items]
    recs = []
    for it, (kind, ks), (hit, m, th, v) in zip(items, owners, out):
        recs.append((it[1], it[2], th, v, hit, m, kind))
        if not hit:
            for k in ks:
                results[k].verified = False
    nmiss = sum(1 for r in recs if not r[4])
    margins = np.array([r[5] for r in recs if r[4]])
    print(f"[verify] done in {time.time() - t0:.1f}s: {nmiss}/{len(recs)} fit shots miss "
          f"({100.0 * nmiss / max(len(recs), 1):.1f}%); "
          f"margin of scoring shots: min {margins.min():.2f}, mean {margins.mean():.2f} sigma")
    worst = sorted(recs, key=lambda r: (r[4], r[5] if r[4] else 0.0))[:10]
    print("[verify] worst 10 (d, vr, theta_fit, v_fit, margin):")
    for d, vr, th, v, hit, m, kind in worst:
        mm = f"{m:.2f} sigma" if hit else "MISS"
        print(f"   d={d:.2f} vr={vr:+.2f} theta={th:6.2f} v={v:5.2f}  {mm}  ({kind})")
    print(f"[verify] {sum(1 for r in results if r.feasible and not r.verified)} grid points "
          f"removed from the domain")
    return recs


def self_check(cfg: Config) -> bool:
    """RK4 (drag and Magnus off) must match the closed-form parabola within 1 mm."""
    c0 = dataclasses.replace(cfg, cd=0.0, cl=0.0)
    th, v, vr = 52.0, 6.5, 0.7
    x0, z0, vx0, vz0 = launch_state(c0, np.array([th]), np.array([v]), vr)
    x, z, vx, vz = x0.copy(), z0.copy(), vx0.copy(), vz0.copy()
    n_total = int(round(0.6 / c0.dt))
    marks = {int(round(t / c0.dt)): t for t in (0.2, 0.4, 0.6)}
    worst = 0.0
    for n in range(1, n_total + 1):
        x, z, vx, vz = rk4_step(x, z, vx, vz, c0.dt, 0.0, 0.0, c0.g)
        if n in marks:
            t = marks[n]
            ex, ez = x0 + vx0 * t, z0 + vz0 * t - 0.5 * c0.g * t * t
            err = float(max(abs(x - ex)[0], abs(z - ez)[0]))
            worst = max(worst, err)
    ok = worst < 1e-3
    print(f"[self-check] RK4 vs parabola at t=0.2/0.4/0.6 s: max error {worst * 1e3:.4f} mm "
          f"-> {'OK' if ok else 'FAIL'}")
    return ok


def sanity_checks(cfg: Config, results):
    """Warn-only physical sanity checks on the table."""
    ds, vrs = cfg.d_values(), cfg.vr_values()
    grid = {(round(r.d, 6), round(r.vr, 6)): r for r in results}
    # v* increases with d at vr = 0
    k0 = int(np.argmin(np.abs(vrs)))
    seq = [(d, grid[(round(float(d), 6), round(float(vrs[k0]), 6))]) for d in ds]
    seq = [(d, r.v) for d, r in seq if r.feasible]
    if len(seq) >= 2:
        dv = np.diff([s[1] for s in seq])
        bad = int((dv < -1e-9).sum())
        if bad:
            print(f"[sanity] WARNING: at vr={vrs[k0]:+.1f}, v* decreases with d in {bad} of "
                  f"{len(dv)} steps (expected generally increasing).")
        else:
            print(f"[sanity] OK: at vr={vrs[k0]:+.1f}, v* increases monotonically with d "
                  f"({seq[0][1]:.2f} -> {seq[-1][1]:.2f} m/s).")
    # closing speed should lower v* at fixed d
    viol = tot = 0
    for d in ds:
        row = [grid[(round(float(d), 6), round(float(v), 6))] for v in vrs]
        row = [r for r in row if r.feasible]
        for a, b in zip(row[:-1], row[1:]):
            tot += 1
            if b.v > a.v + 1e-9:
                viol += 1
    if tot:
        pct = 100.0 * viol / tot
        print(f"[sanity] v* rises with vr (opposite to expected) in {viol}/{tot} adjacent pairs "
              f"({pct:.0f}%)" + ("  <-- WARNING" if pct > 25 else ""))


# ----------------------------------------------------------------------------
# Output
# ----------------------------------------------------------------------------
def fmt_arr(a):
    return ", ".join("Double.NaN" if not np.isfinite(x) else f"{x:.9g}" for x in a)


def config_header(cfg: Config) -> str:
    lines = ["// Generated by tools/shot_sim.py -- config used:"]
    items = [f"{f.name}={getattr(cfg, f.name)}" for f in dataclasses.fields(cfg)]
    lines.append("//   " + ", ".join(items))
    lines.append(f"//   derived: spin_param S={cfg.spin_param:.4f}")
    return "\n".join(lines)


def java_snippet(cfg: Config, terms, c_theta, c_v, c_tof, results) -> str:
    ds, vrs = cfg.d_values(), cfg.vr_values()
    vmin, vmax = [], []
    for d in ds:
        fe = [r.vr for r in results if abs(r.d - d) < 1e-9 and r.feasible and r.verified]
        vmin.append(min(fe) if fe else float("nan"))
        vmax.append(max(fe) if fe else float("nan"))
    order = ", ".join(f"d^{i}*vr^{j}" for i, j in terms)
    out = [config_header(cfg), ""]
    out.append(f"// Polynomial degree {cfg.degree}; d in metres (shaft to window lip midpoint), "
               f"vr in m/s (positive = closing).")
    out.append(f"// Term order (i+j<=deg, by i then j), value = sum coeff[k]*d^i*vr^j: {order}")
    out.append("// HOOD_ANGLE_COEFFS: degrees.  EXIT_SPEED_COEFFS: m/s.  TOF_COEFFS: seconds.")
    out.append(f"public static double[] HOOD_ANGLE_COEFFS = {{{fmt_arr(c_theta)}}};")
    out.append(f"public static double[] EXIT_SPEED_COEFFS = {{{fmt_arr(c_v)}}};")
    out.append(f"public static double[] TOF_COEFFS = {{{fmt_arr(c_tof)}}};")
    out.append("")
    ver = ("Domain is VERIFIED by re-simulating the fitted shots at grid points and midpoints; "
           "points with a miss were removed. " if cfg.verify else "Domain NOT verified (--no-verify). ")
    out.append("// Feasibility domain: per d, min/max feasible vr (NaN = no feasible shot). " + ver +
               "Gaps inside [min,max] are possible.")
    out.append(f"public static double[] D_GRID_M = {{{fmt_arr(ds)}}};")
    out.append(f"public static double[] VR_MIN_MPS = {{{fmt_arr(vmin)}}};")
    out.append(f"public static double[] VR_MAX_MPS = {{{fmt_arr(vmax)}}};")
    return "\n".join(out)


def write_csv(path, results):
    with open(path, "w", encoding="ascii") as fh:
        fh.write("d_m,vr_mps,feasible,theta_deg,v_mps,margin_sigma,tof_s,nvalid\n")
        for r in results:
            fh.write(f"{r.d:.3f},{r.vr:.3f},{int(r.feasible)},{r.theta:.3f},{r.v:.3f},"
                     f"{r.margin:.4f},{r.tof:.4f},{r.nvalid}\n")
    print(f"[csv] wrote {len(results)} rows to {path}")


# ----------------------------------------------------------------------------
# Plotting (lazy matplotlib)
# ----------------------------------------------------------------------------
def _import_plt():
    try:
        import matplotlib.pyplot as plt
        return plt
    except ImportError:
        print("[plot] matplotlib is not installed; skipping plots "
              "(pip install matplotlib to enable --plot).")
        return None


def plot_point(cfg: Config, d, vr):
    plt = _import_plt()
    if plt is None:
        return
    res, valid, rob, _ = solve_point(cfg, d, vr, want_maps=True)
    thetas, speeds = cfg.thetas(), cfg.speeds()
    fig, ax = plt.subplots(1, 2, figsize=(13, 5))
    ext = [speeds[0], speeds[-1], thetas[0], thetas[-1]]
    ax[0].imshow(rob if rob is not None else valid, origin="lower", aspect="auto", extent=ext)
    ax[0].contour(speeds, thetas, valid.astype(float), levels=[0.5], colors="w", linewidths=0.8)
    if res.feasible:
        ax[0].plot(res.v, res.theta, "r*", ms=14, label=f"chosen {res.theta:.1f} deg, {res.v:.2f} m/s")
        ax[0].legend()
    ax[0].set_xlabel("exit speed [m/s]")
    ax[0].set_ylabel("hood angle [deg]")
    ax[0].set_title(f"valid region / robustness [sigma]  d={d} vr={vr}")
    # trajectory fan
    win = Window(cfg, d)
    if res.feasible:
        pairs = [(res.theta + a, res.v + b) for a in (-2, 0, 2) for b in (-0.2, 0, 0.2)]
        tr = trajectories(cfg, d, vr, [p[0] for p in pairs], [p[1] for p in pairs])
        for k, (th, v) in enumerate(pairs):
            p = tr[k]
            m = (p[:, 1] >= 0) & (p[:, 0] <= d + 1)
            ax[1].plot(p[m, 0], p[m, 1], "r" if k == 4 else "0.6", lw=2 if k == 4 else 1)
    ax[1].plot([win.lip[0], win.top[0]], [win.lip[1], win.top[1]], "b-", lw=4, label="window")
    ax[1].plot([0], [cfg.shaft_height], "ko", label="shaft")
    ax[1].set_xlim(-0.3, d + 1)
    ax[1].set_ylim(0, 3)
    ax[1].set_aspect("equal")
    ax[1].set_xlabel("x [m]")
    ax[1].set_ylabel("z [m]")
    ax[1].set_title("trajectory fan (red = chosen)")
    ax[1].legend()
    fig.tight_layout()


def plot_tables(cfg: Config, results, resid_theta, resid_v, feas):
    plt = _import_plt()
    if plt is None:
        return
    ds, vrs = cfg.d_values(), cfg.vr_values()
    shape = (len(ds), len(vrs))
    ext = [vrs[0], vrs[-1], ds[0], ds[-1]]

    def img(attr):
        return np.array([getattr(r, attr) for r in results]).reshape(shape)

    fig, axs = plt.subplots(1, 3, figsize=(15, 4.5))
    for a, (attr, title) in zip(axs, [("theta", "theta* [deg]"), ("v", "v* [m/s]"),
                                       ("margin", "margin [sigma]")]):
        im = a.imshow(img(attr), origin="lower", aspect="auto", extent=ext)
        fig.colorbar(im, ax=a)
        a.set_title(title)
        a.set_xlabel("vr [m/s]")
        a.set_ylabel("d [m]")
    fig.tight_layout()

    fig2, axs2 = plt.subplots(1, 2, figsize=(11, 4.5))
    for a, resid, title in zip(axs2, (resid_theta, resid_v),
                               ("theta fit residual [deg]", "v fit residual [m/s]")):
        full = np.full(len(results), np.nan)
        full[feas] = resid
        im = a.imshow(full.reshape(shape), origin="lower", aspect="auto", extent=ext, cmap="coolwarm")
        fig2.colorbar(im, ax=a)
        a.set_title(title)
        a.set_xlabel("vr [m/s]")
        a.set_ylabel("d [m]")
    fig2.tight_layout()


# ----------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------
def build_parser():
    p = argparse.ArgumentParser(description="Offline robust-shot simulator (see module docstring).")
    cfg = Config()
    add = p.add_argument
    add("--magnus", type=float, default=cfg.cl, metavar="CL", help="Magnus lift coefficient (0=off)")
    add("--cd", type=float, default=cfg.cd)
    add("--mass", type=float, default=cfg.ball_mass, help="ball mass kg (TODO: measure)")
    add("--dt", type=float, default=cfg.dt)
    add("--theta-min", type=float, default=cfg.theta_min)
    add("--theta-max", type=float, default=cfg.theta_max)
    add("--v-min", type=float, default=cfg.v_min)
    add("--v-max", type=float, default=cfg.v_max)
    add("--dtheta", type=float, default=cfg.dtheta)
    add("--dv", type=float, default=cfg.dv)
    add("--d-min", type=float, default=cfg.d_min)
    add("--d-max", type=float, default=cfg.d_max)
    add("--d-step", type=float, default=cfg.d_step)
    add("--vr-min", type=float, default=cfg.vr_min)
    add("--vr-max", type=float, default=cfg.vr_max)
    add("--vr-step", type=float, default=cfg.vr_step)
    add("--margin", type=float, default=cfg.margin)
    add("--sigma-theta", type=float, default=cfg.sigma_theta)
    add("--sigma-v", type=float, default=cfg.sigma_v)
    add("--degree", type=int, default=cfg.degree)
    add("--tie-tol", type=float, default=cfg.tie_tol,
        help="sigma; treat cells within this of the best margin as ties and pick their centroid")
    add("--workers", type=int, default=min(os.cpu_count() or 1, 8),
        help="parallel processes for the sweep (1 = serial)")
    add("--smooth-eps", type=float, default=cfg.smooth_eps,
        help="smoothing candidates: rob >= (1-eps)*max_rob")
    add("--smooth-iters", type=int, default=cfg.smooth_iters, help="smoothing iterations (0 = off)")
    add("--no-verify", action="store_true", help="skip re-simulation of the fitted shots")
    add("--weight", action="store_true", help="weight the fit by robustness margin")
    add("--csv", type=str, default=None, help="write full table to this CSV path")
    add("--plot", action="store_true", help="show plots (needs matplotlib)")
    add("--d", type=float, default=2.5, help="d for the --plot point views")
    add("--vr", type=float, default=0.0, help="vr for the --plot point views")
    add("--single", type=float, nargs=2, metavar=("D", "VR"), help="run one point and print details")
    return p


def config_from_args(a) -> Config:
    return Config(cl=a.magnus, cd=a.cd, ball_mass=a.mass, dt=a.dt,
                  theta_min=a.theta_min, theta_max=a.theta_max, v_min=a.v_min, v_max=a.v_max,
                  dtheta=a.dtheta, dv=a.dv, d_min=a.d_min, d_max=a.d_max, d_step=a.d_step,
                  vr_min=a.vr_min, vr_max=a.vr_max, vr_step=a.vr_step, margin=a.margin,
                  sigma_theta=a.sigma_theta, sigma_v=a.sigma_v, degree=a.degree, weight=a.weight,
                  tie_tol=a.tie_tol, smooth_eps=a.smooth_eps,
                  smooth_iters=a.smooth_iters, verify=not a.no_verify, workers=a.workers)


def run_single(cfg: Config, d, vr, plot):
    t0 = time.time()
    res, valid, rob, tof = solve_point(cfg, d, vr, want_maps=True)
    win = Window(cfg, d)
    print(f"single point d={d} m, vr={vr:+} m/s  ({time.time() - t0:.1f}s)")
    print(f"  window lip=({win.lip[0]:.3f}, {win.lip[1]:.4f})  top=({win.top[0]:.3f}, {win.top[1]:.4f})")
    print(f"  grid {len(cfg.thetas())} x {len(cfg.speeds())}, valid cells: {res.nvalid}")
    if not res.feasible:
        print("  INFEASIBLE")
        return
    ths, vs = cfg.thetas(), cfg.speeds()
    rows, cols = np.nonzero(valid)
    print(f"  valid theta range {ths[rows].min():.1f}..{ths[rows].max():.1f} deg, "
          f"speed range {vs[cols].min():.2f}..{vs[cols].max():.2f} m/s")
    print(f"  chosen theta*={res.theta:.2f} deg, v*={res.v:.2f} m/s, "
          f"margin={res.margin:.2f} sigma, tof={res.tof:.3f} s")
    tr = trajectories(cfg, d, vr, res.theta, res.v)[0]
    sd = win.signed_dist(tr[:, 0], tr[:, 1])
    k = int(np.argmax(sd >= 0))
    f = -sd[k - 1] / (sd[k] - sd[k - 1])
    p = tr[k - 1] + f * (tr[k] - tr[k - 1])
    print(f"  crossing at ({p[0]:.3f}, {p[1]:.3f}) m, s={win.along(*p):.3f} m of {cfg.window_length:.3f} m "
          f"(allowed {cfg.rb + cfg.margin:.3f}..{cfg.window_length - cfg.rb - cfg.margin:.3f})")
    apex = tr[:, 1].max()
    print(f"  apex height {apex:.2f} m")
    if plot:
        plot_point(cfg, d, vr)


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    cfg = config_from_args(args)
    if not self_check(cfg):
        return 1
    if args.single:
        run_single(cfg, args.single[0], args.single[1], args.plot)
        if args.plot:
            plt = _import_plt()
            if plt:
                plt.show()
        return 0

    print(f"[sweep] {len(cfg.d_values())} d x {len(cfg.vr_values())} vr, inner grid "
          f"{len(cfg.thetas())} x {len(cfg.speeds())}, dt={cfg.dt}, CL={cfg.cl}")
    t0 = time.time()
    results = sweep(cfg)
    runtime = time.time() - t0
    feas = np.array([r.feasible for r in results])
    print(f"[sweep] done in {runtime:.1f}s; feasible {int(feas.sum())}/{len(results)}")
    sanity_checks(cfg, results)

    if args.csv:
        write_csv(args.csv, results)
    if feas.sum() == 0:
        print("No feasible points; nothing to fit.")
        return 2
    terms = poly_terms(cfg.degree)
    if feas.sum() < max(len(terms), len(poly_terms(4))):
        print(f"Too few feasible points ({int(feas.sum())}) for the fit.")
        return 2
    if cfg.smooth_iters > 0:
        feas, fits = smooth_selection(cfg, results)
    else:
        feas, fits = fit_all(cfg, results, cfg.degree)
    for name, unit in (("theta", "deg"), ("v", "m/s"), ("tof", "s")):
        resid = fits[name][1]
        print(f"[fit] {name:5s} degree {cfg.degree} ({len(terms)} terms, {len(resid)} pts): "
              f"max |resid| {np.abs(resid).max():.4f} {unit}, RMS {np.sqrt((resid ** 2).mean()):.4f} {unit}")
    for deg in sorted({3, 4}):
        _, f2 = fit_all(cfg, results, deg)
        print("[degree compare] " + resid_line(f2, f"degree {deg} ({len(poly_terms(deg))} terms)"))
    if cfg.verify:
        verify_fit(cfg, results, fits)
    print()
    print(java_snippet(cfg, terms, fits["theta"][0], fits["v"][0], fits["tof"][0], results))
    print()
    if args.plot:
        plot_tables(cfg, results, fits["theta"][1], fits["v"][1], feas)
        plot_point(cfg, args.d, args.vr)
        plt = _import_plt()
        if plt:
            plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
