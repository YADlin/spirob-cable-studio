"""Minimal CMA-ES (Hansen 2016 tutorial form), bounded, no external dependency.

Used as the global, derivative-free search for model identification (as in PACE)
when the objective has kinks: slack/taut cable switching, contact rejection or
equilibrium branch changes defeat gradient methods. Polish the result with a
local least-squares step afterwards.

Parameters are searched in normalised coordinates u ∈ [0, 1] per bound; samples
outside are reflected back, so every evaluated point respects the bounds.
"""
import numpy as np


def _reflect(u):
    u = np.mod(u, 2.)
    return np.where(u > 1., 2. - u, u)


def minimize(f, x0, lower, upper, sigma0=.2, popsize=None, max_evaluations=2000, seed=0,
             tol=1e-6, callback=None, cancel=None):
    """Minimise scalar f(x). Returns dict(x, f, evaluations, generations, history)."""
    lo, hi = np.asarray(lower, float), np.asarray(upper, float)
    span = hi - lo
    n = len(lo)
    rng = np.random.default_rng(seed)
    lam = popsize or 4 + int(3 * np.log(n))
    mu = lam // 2
    w = np.log(mu + .5) - np.log(np.arange(1, mu + 1)); w /= w.sum()
    mueff = 1. / np.sum(w ** 2)
    cc = (4 + mueff / n) / (n + 4 + 2 * mueff / n)
    cs = (mueff + 2) / (n + mueff + 5)
    c1 = 2 / ((n + 1.3) ** 2 + mueff)
    cmu = min(1 - c1, 2 * (mueff - 2 + 1 / mueff) / ((n + 2) ** 2 + mueff))
    damps = 1 + 2 * max(0, np.sqrt((mueff - 1) / (n + 1)) - 1) + cs
    chin = np.sqrt(n) * (1 - 1 / (4 * n) + 1 / (21 * n * n))
    m = np.clip((np.asarray(x0, float) - lo) / span, 0, 1)
    sigma, pc, ps, C = float(sigma0), np.zeros(n), np.zeros(n), np.eye(n)
    B, D = np.eye(n), np.ones(n)
    best_x, best_f = lo + m * span, float(f(lo + m * span))
    evals, gen, history, eigen_at = 1, 0, [best_f], 0
    while evals + lam <= max_evaluations:
        if cancel is not None and cancel.is_set():
            break
        z = rng.standard_normal((lam, n))
        y = z @ (B * D).T
        u = _reflect(m + sigma * y)
        fx = np.array([f(lo + ui * span) for ui in u]); evals += lam
        order = np.argsort(fx)
        if fx[order[0]] < best_f:
            best_f, best_x = float(fx[order[0]]), lo + u[order[0]] * span
        y_sel = (u[order[:mu]] - m) / sigma          # use the reflected samples
        m_old, m = m, m + sigma * (w @ y_sel)
        invsqrt = B @ np.diag(1 / D) @ B.T
        ps = (1 - cs) * ps + np.sqrt(cs * (2 - cs) * mueff) * invsqrt @ (m - m_old) / sigma
        hsig = np.linalg.norm(ps) / np.sqrt(1 - (1 - cs) ** (2 * (gen + 1))) / chin < 1.4 + 2 / (n + 1)
        pc = (1 - cc) * pc + hsig * np.sqrt(cc * (2 - cc) * mueff) * (m - m_old) / sigma
        C = ((1 - c1 - cmu) * C + c1 * (np.outer(pc, pc) + (1 - hsig) * cc * (2 - cc) * C)
             + cmu * (y_sel.T * w) @ y_sel)
        sigma *= np.exp((cs / damps) * (np.linalg.norm(ps) / chin - 1))
        gen += 1
        if evals - eigen_at > lam / (c1 + cmu) / n / 10:
            eigen_at = evals
            C = np.triu(C) + np.triu(C, 1).T
            D2, B = np.linalg.eigh(C)
            D = np.sqrt(np.clip(D2, 1e-20, None))
        history.append(best_f)
        if callback:
            callback(evals, max_evaluations, best_f)
        if sigma * D.max() < tol or (len(history) > 30 and abs(history[-30] - history[-1]) < tol * max(1., abs(history[-1]))):
            break
    return dict(x=best_x, f=best_f, evaluations=evals, generations=gen, history=history)
