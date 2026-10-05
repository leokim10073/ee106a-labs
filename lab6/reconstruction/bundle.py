"""
Levenberg-Marquardt bundle adjustment with sparse Jacobians.

Optimizes every camera (R, t, world -> camera), every point X, and optionally
the focal length. The cost is the Huber reprojection error plus an optional
prior pulling each camera toward its SLAM pose.

Rotation updates are R <- exp([d]x) R, so d(R X + t)/dd = -[R X]x.
"""
from dataclasses import dataclass, field

import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla
from scipy.spatial.transform import Rotation


@dataclass
class Prior:
    R: np.ndarray  # (N, 3, 3) SLAM rotations (world -> camera)
    t: np.ndarray  # (N, 3)
    rot_deg: float = 1.0  # how far (1 sigma) a camera is allowed to turn
    pos_m: float = 0.03   # and move


@dataclass
class State:
    f: float
    R: np.ndarray  # (N, 3, 3)
    t: np.ndarray  # (N, 3)
    X: np.ndarray  # (M, 3)

    def copy(self):
        return State(self.f, self.R.copy(), self.t.copy(), self.X.copy())

    def centers(self):
        return -np.einsum('nji,nj->ni', self.R, self.t)


@dataclass
class Result:
    state: State
    history: list = field(default_factory=list)  # one entry per iteration
    converged: bool = False
    iterations: int = 0


def skew_batch(v):
    S = np.zeros(v.shape[:-1] + (3, 3))
    S[..., 0, 1], S[..., 0, 2] = -v[..., 2], v[..., 1]
    S[..., 1, 0], S[..., 1, 2] = v[..., 2], -v[..., 0]
    S[..., 2, 0], S[..., 2, 1] = -v[..., 1], v[..., 0]
    return S


def project(state, cam_idx, pt_idx, cx, cy):
    Xc = np.einsum('nij,nj->ni', state.R[cam_idx], state.X[pt_idx]) + state.t[cam_idx]
    uv = state.f * Xc[:, :2] / Xc[:, 2:3] + [cx, cy]
    return uv, Xc


def huber_weights(norms, delta):
    w = np.ones_like(norms)
    big = norms > delta
    w[big] = delta / norms[big]
    return w


def huber_cost(norms, delta):
    return float(np.sum(np.where(norms <= delta, 0.5 * norms ** 2, delta * (norms - 0.5 * delta))))


def bundle_adjust(state, cam_idx, pt_idx, uv_obs, cx, cy, prior=None, optimize_focal=True,
                  fixed_cams=(), huber_px=2.0, max_iters=100, tol=1e-7, record=False):
    n_cams, n_pts, n_obs = len(state.R), len(state.X), len(cam_idx)
    # params are [f, 6 per camera, 3 per point]
    n_par = 1 + 6 * n_cams + 3 * n_pts
    free = np.ones(n_par, bool)
    free[0] = optimize_focal
    for c in fixed_cams:
        free[1 + 6 * c: 7 + 6 * c] = False
    cam_col = 1 + 6 * cam_idx
    pt_col = 1 + 6 * n_cams + 3 * pt_idx
    rows2 = np.arange(2 * n_obs).reshape(n_obs, 2)

    def evaluate(s):
        uv, Xc = project(s, cam_idx, pt_idx, cx, cy)
        r = uv - uv_obs
        norms = np.linalg.norm(r, axis=1)
        cost = huber_cost(norms, huber_px)
        pr = None
        if prior is not None:
            rot_dev = Rotation.from_matrix(np.einsum('nij,nkj->nik', s.R, prior.R)).as_rotvec()
            pos_dev = s.centers() - prior_centers
            pr = np.hstack([rot_dev / np.radians(prior.rot_deg), pos_dev / prior.pos_m])
            cost += 0.5 * float(np.sum(pr ** 2))
        return r, norms, Xc, pr, cost

    if prior is not None:
        prior_centers = -np.einsum('nji,nj->ni', prior.R, prior.t)

    def jacobian(s, r, norms, Xc, pr):
        w = np.sqrt(huber_weights(norms, huber_px))
        x, y, z = Xc[:, 0], Xc[:, 1], Xc[:, 2]
        # d(uv)/d(Xc)
        dP = np.zeros((n_obs, 2, 3))
        dP[:, 0, 0] = s.f / z
        dP[:, 0, 2] = -s.f * x / z ** 2
        dP[:, 1, 1] = s.f / z
        dP[:, 1, 2] = -s.f * y / z ** 2
        RX = Xc - s.t[cam_idx]
        J_rot = -dP @ skew_batch(RX)
        J_t = dP
        J_X = dP @ s.R[cam_idx]
        J_f = np.stack([x / z, y / z], axis=1)

        wr = (r * w[:, None]).ravel()
        blocks_r, blocks_c, blocks_v = [], [], []

        def add(rows, cols, vals):
            rows, cols, vals = np.broadcast_arrays(rows, cols, vals)
            blocks_r.append(rows.ravel())
            blocks_c.append(cols.ravel())
            blocks_v.append(vals.ravel())

        W = w[:, None, None]
        rr = np.repeat(rows2[:, :, None], 3, axis=2)
        add(rows2, np.zeros_like(rows2), J_f * w[:, None])
        add(rr, cam_col[:, None, None] + np.arange(3)[None, None, :], J_rot * W)
        add(rr, cam_col[:, None, None] + 3 + np.arange(3)[None, None, :], J_t * W)
        add(rr, pt_col[:, None, None] + np.arange(3)[None, None, :], J_X * W)
        n_rows = 2 * n_obs
        if pr is not None:
            # prior rows: rotation is ~identity in d, and the center is c = -R^T t
            base = n_rows + 6 * np.arange(n_cams)
            sr, sc = np.radians(prior.rot_deg), prior.pos_m
            eye = np.broadcast_to(np.eye(3) / sr, (n_cams, 3, 3))
            rr3 = base[:, None, None] + np.arange(3)[None, :, None]
            cc3 = 1 + 6 * np.arange(n_cams)[:, None, None] + np.arange(3)[None, None, :]
            add(np.broadcast_to(rr3, (n_cams, 3, 3)), np.broadcast_to(cc3, (n_cams, 3, 3)), eye)
            Rt = np.transpose(s.R, (0, 2, 1))
            dc_drot = -Rt @ skew_batch(s.t) / sc
            dc_dt = -Rt / sc
            add(np.broadcast_to(rr3 + 3, (n_cams, 3, 3)), np.broadcast_to(cc3, (n_cams, 3, 3)), dc_drot)
            add(np.broadcast_to(rr3 + 3, (n_cams, 3, 3)), np.broadcast_to(cc3 + 3, (n_cams, 3, 3)), dc_dt)
            wr = np.concatenate([wr, pr.ravel()])
            n_rows += 6 * n_cams
        J = sp.csr_matrix((np.concatenate(blocks_v), (np.concatenate(blocks_r), np.concatenate(blocks_c))),
                          shape=(n_rows, n_par))[:, free]
        return J, wr

    def apply(s, dx_free):
        dx = np.zeros(n_par)
        dx[free] = dx_free
        out = s.copy()
        out.f = s.f + dx[0]
        d = dx[1:1 + 6 * n_cams].reshape(n_cams, 6)
        out.R = Rotation.from_rotvec(d[:, :3]).as_matrix() @ s.R
        out.t = s.t + d[:, 3:]
        out.X = s.X + dx[1 + 6 * n_cams:].reshape(n_pts, 3)
        return out

    def snapshot(s, norms, cost, lam):
        return dict(f=float(s.f), R=s.R.copy(), t=s.t.copy(), X=s.X.copy(), cost=cost,
                    rms=float(np.sqrt(np.mean(norms ** 2))), median=float(np.median(norms)), lam=lam)

    s = state.copy()
    r, norms, Xc, pr, cost = evaluate(s)
    result = Result(s)
    lam = 1e-3
    if record:
        result.history.append(snapshot(s, norms, cost, lam))
    for it in range(max_iters):
        J, wr = jacobian(s, r, norms, Xc, pr)
        Hm = (J.T @ J).tocsc()
        g = J.T @ wr
        diag = Hm.diagonal()
        improved = False
        for _ in range(12):  # keep increasing lambda until the cost goes down
            A = Hm + sp.diags(lam * np.maximum(diag, 1e-9))
            dx = spla.spsolve(A, -g)
            s_new = apply(s, dx)
            r_n, norms_n, Xc_n, pr_n, cost_n = evaluate(s_new)
            if np.all(Xc_n[:, 2] > 1e-6) and cost_n < cost:
                improved = True
                break
            lam *= 10
        if not improved:
            result.converged = True
            break
        rel = (cost - cost_n) / max(cost, 1e-12)
        s, r, norms, Xc, pr, cost = s_new, r_n, norms_n, Xc_n, pr_n, cost_n
        lam = max(lam / 3, 1e-9)
        result.iterations = it + 1
        if record:
            result.history.append(snapshot(s, norms, cost, lam))
        if rel < tol:
            result.converged = True
            break
    result.state = s
    return result
