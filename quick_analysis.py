#!/usr/bin/env python3
"""
Position and orientation error over time: ArUco estimate vs OptiTrack ground truth.

Python equivalent of plot_position_error.m.

Usage:
    python3 plot_position_error.py                    # uses BAG_FOLDER below
    python3 plot_position_error.py aruco_trial_05     # or pass the bag folder

Requires: pip install rosbags numpy matplotlib
(rosbags reads ROS 2 bags directly; no ROS installation needed.)
"""

import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

# ============================== SETTINGS ====================================
BAG_FOLDER = 'baseline_dark_set_ground_truth'
ERR_FRAME = 'camera'        # 'camera' (X lateral, Y vertical, Z depth) or 'world'
SAVE_FIGURES = False        # True -> PNGs written next to the bag

# OptiTrack dropout rejection
MIN_FROZEN_SAMPLES = 5      # identical consecutive mocap poses = lost tracking
JUMP_THRESHOLD = 0.02       # m between consecutive mocap samples (100 Hz)
DROPOUT_MARGIN = 0.10       # s excluded either side of a dropout

# ArUco outlier (bad frame) detection: a frame is an outlier if its error
# departs from the running median of its neighbours by more than
# max(minimum, K_MAD * local robust std). Outliers are reported, not hidden.
OUTLIER_WINDOW = 7          # frames in running median
K_MAD = 6                   # robust threshold multiplier
MIN_ORI_OUTLIER = 2.0       # deg, never flag smaller orientation deviations
MIN_POS_OUTLIER = 10.0      # mm,  never flag smaller position deviations

# Camera rigid body -> camera optical frame
BC_T_C = np.array([
    [0.99931745, -0.01430626,  0.03405815,  0.00422558],
    [0.03325054, -0.05333160, -0.99802312, -0.01757960],
    [0.01609435,  0.99847437, -0.05281951,  0.01983206],
    [0, 0, 0, 1]])

#-0.52892899, 0.49732282, 0.50269367, 0.46925815) # @ M_fix
#        self.BC_T_C[0:3, 3] = [0.05060061, 0.00010308, 0.00637311]




# Marker rigid body -> ArUco centre
BM_T_M = np.eye(4)
BM_T_M[:3, :3] = [[0, 1, 0], [0, 0, 1], [1, 0, 0]]
BM_T_M[:3, 3] = [0.075, -0.0146, -0.117]

TOPIC_ARUCO = '/aruco/pose'
TOPIC_CAM_RB = '/vrpn_mocap/camera_rigidbody/pose'
TOPIC_MARKER_RB = '/vrpn_mocap/marker_rigidbody/pose'


# ============================== HELPERS =====================================
def read_poses(reader, topic):
    """[t x y z qx qy qz qw] per message, t from the header stamp, sorted."""
    conns = [c for c in reader.connections if c.topic == topic]
    rows = []
    for conn, _, raw in reader.messages(connections=conns):
        m = reader.deserialize(raw, conn.msgtype)
        p, q = m.pose.position, m.pose.orientation
        rows.append([m.header.stamp.sec + 1e-9 * m.header.stamp.nanosec,
                     p.x, p.y, p.z, q.x, q.y, q.z, q.w])
    if not rows:
        return np.empty((0, 8))
    P = np.array(rows)
    _, idx = np.unique(P[:, 0], return_index=True)   # sort, drop duplicate stamps
    return P[idx]


def quat_to_rotm(q):
    """q = [x y z w] (N,4) -> (N,3,3)."""
    q = q / np.linalg.norm(q, axis=-1, keepdims=True)
    x, y, z, w = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    R = np.empty(q.shape[:-1] + (3, 3))
    R[..., 0, 0] = 1 - 2 * (y*y + z*z); R[..., 0, 1] = 2 * (x*y - z*w);     R[..., 0, 2] = 2 * (x*z + y*w)
    R[..., 1, 0] = 2 * (x*y + z*w);     R[..., 1, 1] = 1 - 2 * (x*x + z*z); R[..., 1, 2] = 2 * (y*z - x*w)
    R[..., 2, 0] = 2 * (x*z - y*w);     R[..., 2, 1] = 2 * (y*z + x*w);     R[..., 2, 2] = 1 - 2 * (x*x + y*y)
    return R


def poses_to_T(v):
    """v = (N,7) [x y z qx qy qz qw] -> (N,4,4)."""
    T = np.tile(np.eye(4), (len(v), 1, 1))
    T[:, :3, :3] = quat_to_rotm(v[:, 3:7])
    T[:, :3, 3] = v[:, :3]
    return T


def interp_poses(P, tq):
    """Poses at times tq: linear position, normalised-lerp quaternion."""
    t = P[:, 0]
    tq = np.clip(tq, t[0], t[-1])
    i = np.clip(np.searchsorted(t, tq, side='right') - 1, 0, len(t) - 2)
    s = ((tq - t[i]) / (t[i + 1] - t[i]))[:, None]
    q1, q2 = P[i, 4:8], P[i + 1, 4:8].copy()
    q2[np.sum(q1 * q2, axis=1) < 0] *= -1             # same hemisphere
    v = np.hstack([(1 - s) * P[i, 1:4] + s * P[i + 1, 1:4], (1 - s) * q1 + s * q2])
    return poses_to_T(v)


def rotm_to_rotvec(R):
    """(N,3,3) -> (N,3) rotation vectors (axis * angle, rad)."""
    c = np.clip((np.trace(R, axis1=1, axis2=2) - 1) / 2, -1, 1)
    th = np.arccos(c)
    w = np.stack([R[:, 2, 1] - R[:, 1, 2], R[:, 0, 2] - R[:, 2, 0], R[:, 1, 0] - R[:, 0, 1]], 1)
    rv = np.zeros((len(R), 3))
    ok = (th > 1e-9) & (np.pi - th > 1e-6)
    rv[ok] = (th[ok] / (2 * np.sin(th[ok])))[:, None] * w[ok]
    for k in np.where(np.pi - th <= 1e-6)[0]:         # ~180 deg: eigenvector
        vals, vecs = np.linalg.eig(R[k])
        rv[k] = th[k] * np.real(vecs[:, np.argmin(np.abs(vals - 1))])
    return rv


def find_dropouts(P, min_frozen, jump_thr):
    """[t_start, t_end] windows where mocap froze (identical poses) or jumped."""
    W = []
    same = np.all(np.abs(np.diff(P[:, 1:8], axis=0)) < 1e-9, axis=1)
    k, n = 0, len(same)
    while k < n:
        if same[k]:
            j = k
            while j < n and same[j]:
                j += 1
            if j - k >= min_frozen:
                W.append((P[k, 0], P[j, 0]))
            k = j
        else:
            k += 1
    jumps = np.where(np.linalg.norm(np.diff(P[:, 1:4], axis=0), axis=1) > jump_thr)[0]
    W += [(P[j, 0], P[j + 1, 0]) for j in jumps]
    return W


def movmedian(x, win):
    """Centred running median ignoring NaN (like MATLAB movmedian 'omitnan')."""
    x = np.asarray(x, float)
    out = np.full_like(x, np.nan)
    h = win // 2
    for i in range(len(x)):
        seg = x[max(0, i - h):i + h + 1]
        if np.any(~np.isnan(seg)):
            out[i] = np.nanmedian(seg, axis=0)
    return out


def find_outliers(E, valid, win, k, min_dev):
    """Flag frames whose error departs from the running median of neighbours.
    The threshold adapts locally (robust std over ~31 frames)."""
    E = E.copy()
    E[~valid] = np.nan
    dev = np.linalg.norm(E - movmedian(E, win), axis=1)
    dev[~valid] = np.nan
    loc_med = movmedian(dev, 31)
    loc_mad = movmedian(np.abs(dev - loc_med), 31)
    thr = np.maximum(min_dev, k * 1.4826 * loc_mad)
    with np.errstate(invalid='ignore'):
        return valid & (dev > thr)


def shade(ax, t, mask, color):
    """Shade contiguous True regions of mask."""
    d = np.diff(np.r_[0, mask.astype(int), 0])
    for s, e in zip(np.where(d == 1)[0], np.where(d == -1)[0] - 1):
        ax.axvspan(t[s], t[e], color=color, alpha=0.5, lw=0, zorder=0)


def rms(x):
    return np.sqrt(np.mean(x ** 2))


# ================================ MAIN ======================================
def main(bag_folder):
    bag = Path(bag_folder)
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    with AnyReader([bag], default_typestore=typestore) as reader:
        A = read_poses(reader, TOPIC_ARUCO)        # camera -> marker
        CB = read_poses(reader, TOPIC_CAM_RB)      # world -> camera RB
        MB = read_poses(reader, TOPIC_MARKER_RB)   # world -> marker RB
    if len(A) == 0:
        sys.exit(f'No ArUco poses in {bag_folder}')
    if len(CB) == 0 or len(MB) == 0:
        sys.exit(f'Missing OptiTrack rigid-body data in {bag_folder}')

    n = len(A)
    t = A[:, 0] - A[0, 0]

    # ---- OptiTrack dropouts -------------------------------------------------
    drop_win = (find_dropouts(MB, MIN_FROZEN_SAMPLES, JUMP_THRESHOLD)
                + find_dropouts(CB, MIN_FROZEN_SAMPLES, JUMP_THRESHOLD))
    excluded = (A[:, 0] < max(CB[0, 0], MB[0, 0])) | (A[:, 0] > min(CB[-1, 0], MB[-1, 0]))
    for a, b in drop_win:
        excluded |= (A[:, 0] >= a - DROPOUT_MARGIN) & (A[:, 0] <= b + DROPOUT_MARGIN)
    valid = ~excluded

    # ---- Errors at each ArUco frame ----------------------------------------
    O_T_C = interp_poses(CB, A[:, 0]) @ BC_T_C
    C_T_M = poses_to_T(A[:, 1:8])
    est = O_T_C @ C_T_M                              # ArUco estimate in world
    gt = interp_poses(MB, A[:, 0]) @ BM_T_M          # ground truth in world

    e = est[:, :3, 3] - gt[:, :3, 3]
    if ERR_FRAME == 'camera':
        e = np.einsum('nji,nj->ni', O_T_C[:, :3, :3], e)   # R_OC^T * e
    err = 1000 * e                                    # mm

    R_err = np.einsum('nji,njk->nik', gt[:, :3, :3], est[:, :3, :3])  # in GT marker frame
    ori_axes = np.degrees(rotm_to_rotvec(R_err))
    ori_tot = np.linalg.norm(ori_axes, axis=1)
    ori_tilt = np.degrees(np.arccos(np.clip(np.sum(est[:, :3, 2] * gt[:, :3, 2], 1), -1, 1)))
    gt_pos = gt[:, :3, 3]
    view_ang = np.degrees(np.arccos(np.clip(np.abs(C_T_M[:, 2, 2]), 0, 1)))

    err_norm = np.linalg.norm(err, axis=1)
    for arr in (err, err_norm, ori_axes, ori_tot, ori_tilt):
        arr[excluded] = np.nan                        # gaps in the plots

    # ---- ArUco outlier frames ----------------------------------------------
    ori_out = find_outliers(ori_axes, valid, OUTLIER_WINDOW, K_MAD, MIN_ORI_OUTLIER)
    pos_out = find_outliers(err, valid, OUTLIER_WINDOW, K_MAD, MIN_POS_OUTLIER)
    outlier = valid & (ori_out | pos_out)
    inlier = valid & ~outlier

    e_all, e_in = err[valid], err[inlier]
    o_all, o_in = ori_axes[valid], ori_axes[inlier]

    # ---- Plots ---------------------------------------------------------------
    if ERR_FRAME == 'camera':
        labels = ['X (lateral)', 'Y (vertical)', 'Z (depth)']
    else:
        labels = ['World X', 'World Y', 'World Z']
    ori_labels = ['About marker X', 'About marker Y', 'About marker Z (normal)']
    cols = ['#0072BD', '#77AC30', '#A2142F']
    name = bag.name
    figs = []

    fig, axs = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    for a in range(3):
        ax = axs[a]
        ax.plot(t, err[:, a], '.-', color=cols[a], ms=3, lw=0.8)
        ax.plot(t[outlier], err[outlier, a], 'o', mfc='none', mec='r', ms=8, mew=1.5)
        ax.axhline(0, color='k', ls=':', lw=0.8)
        mu = np.mean(e_in[:, a])
        ax.axhline(mu, color=cols[a], ls='--', lw=1)
        ax.text(1.0, mu, f' mean {mu:.1f} mm', color=cols[a], transform=ax.get_yaxis_transform(),
                va='bottom', ha='right')
        shade(ax, t, excluded, '#ff9999')
        ax.set_ylabel(f'{labels[a]} (mm)'); ax.grid(True, alpha=0.4)
    axs[3].plot(t, err_norm, 'k.-', ms=3, lw=0.8)
    axs[3].plot(t[outlier], err_norm[outlier], 'o', mfc='none', mec='r', ms=8, mew=1.5)
    shade(axs[3], t, excluded, '#ff9999')
    axs[3].set_ylabel('3D error (mm)'); axs[3].set_xlabel('Time (s)'); axs[3].grid(True, alpha=0.4)
    fig.suptitle(f'{name}: position error ({ERR_FRAME} frame)  |  o = ArUco outlier, '
                 'red band = mocap dropout')
    fig.tight_layout(); figs.append(('position_error', fig))

    fig, axs = plt.subplots(4, 1, figsize=(10, 9), sharex=True)
    for a in range(3):
        ax = axs[a]
        ax.plot(t, ori_axes[:, a], '.-', color=cols[a], ms=3, lw=0.8)
        ax.plot(t[outlier], ori_axes[outlier, a], 'o', mfc='none', mec='r', ms=8, mew=1.5)
        ax.axhline(0, color='k', ls=':', lw=0.8)
        mu = np.mean(o_in[:, a])
        ax.axhline(mu, color=cols[a], ls='--', lw=1)
        ax.text(1.0, mu, f' mean {mu:.2f} deg', color=cols[a], transform=ax.get_yaxis_transform(),
                va='bottom', ha='right')
        shade(ax, t, excluded, '#ff9999')
        ax.set_ylabel(f'{ori_labels[a]} (deg)'); ax.grid(True, alpha=0.4)
    axs[3].plot(t, ori_tot, 'k.-', ms=3, lw=0.8, label='Total')
    axs[3].plot(t, ori_tilt, '.-', color='#D95319', ms=3, lw=0.8, label='Normal tilt')
    axs[3].plot(t[outlier], ori_tot[outlier], 'o', mfc='none', mec='r', ms=8, mew=1.5, label='Outlier')
    shade(axs[3], t, excluded, '#ff9999')
    axs[3].legend(loc='best'); axs[3].set_ylabel('Total (deg)'); axs[3].set_xlabel('Time (s)')
    axs[3].grid(True, alpha=0.4)
    fig.suptitle(f'{name}: orientation error (marker frame)  |  o = ArUco outlier, '
                 'red band = mocap dropout')
    fig.tight_layout(); figs.append(('orientation_error', fig))

    # ---- Statistics ----------------------------------------------------------
    print(f'==== {name} ({ERR_FRAME} frame) ====')
    print(f'Frames: {n} total, {excluded.sum()} excluded (mocap dropout), '
          f'{outlier.sum()} ArUco outliers, {inlier.sum()} inliers\n')

    hdr = '                         |  mean    std    RMSE   |  mean    std    RMSE'
    print('POSITION (mm)            |  all valid frames      |  outliers removed')
    print(hdr)
    for a in range(3):
        print(f'  {labels[a]:<22} | {np.mean(e_all[:, a]):6.1f} {np.std(e_all[:, a], ddof=1):6.1f} '
              f'{rms(e_all[:, a]):6.1f}   | {np.mean(e_in[:, a]):6.1f} '
              f'{np.std(e_in[:, a], ddof=1):6.1f} {rms(e_in[:, a]):6.1f}')
    print(f'  {"3D norm":<22} | {np.mean(err_norm[valid]):6.1f}   max {np.max(err_norm[valid]):6.1f}   '
          f'| {np.mean(err_norm[inlier]):6.1f}   max {np.max(err_norm[inlier]):6.1f}')

    print('\nORIENTATION (deg)        |  all valid frames      |  outliers removed')
    print(hdr)
    for a in range(3):
        print(f'  {ori_labels[a]:<22} | {np.mean(o_all[:, a]):6.2f} {np.std(o_all[:, a], ddof=1):6.2f} '
              f'{rms(o_all[:, a]):6.2f}   | {np.mean(o_in[:, a]):6.2f} '
              f'{np.std(o_in[:, a], ddof=1):6.2f} {rms(o_in[:, a]):6.2f}')
    print(f'  {"Total":<22} | {np.mean(ori_tot[valid]):6.2f}   max {np.max(ori_tot[valid]):6.2f}   '
          f'| {np.mean(ori_tot[inlier]):6.2f}   max {np.max(ori_tot[inlier]):6.2f}')

    if outlier.any():
        print(f'\nOUTLIER FRAMES (median view angle {np.median(view_ang[inlier]):.1f} deg)')
        print('   t (s)   ori dev (deg)  pos dev (mm)  view angle (deg)')
        ori_med, pos_med = movmedian(ori_axes, OUTLIER_WINDOW), movmedian(err, OUTLIER_WINDOW)
        for k in np.where(outlier)[0]:
            print(f'  {t[k]:6.2f}   {np.linalg.norm(ori_axes[k] - ori_med[k]):8.2f}       '
                  f'{np.linalg.norm(err[k] - pos_med[k]):8.1f}        {view_ang[k]:6.1f}')

    # ---- Automatic conclusions ----------------------------------------------
    print('\n==== CONCLUSIONS ====')
    gt_range = 1000 * (gt_pos[valid].max(0) - gt_pos[valid].min(0))
    static_trial = np.all(gt_range < 5)
    if static_trial:
        print(f'- Static trial (marker moved < 5 mm). Mean viewing angle '
              f'{np.mean(view_ang[valid]):.1f} deg.')
    else:
        print('- Moving trial (marker range {:.0f} x {:.0f} x {:.0f} mm).'.format(*gt_range))

    n_out = outlier.sum()
    print(f'- ArUco outliers: {n_out} of {valid.sum()} valid frames '
          f'({100 * n_out / valid.sum():.1f} %).', end='')
    if n_out:
        n_ori = np.sum(outlier & ori_out & ~pos_out)
        n_pos = np.sum(outlier & pos_out)
        print(f' {n_ori} affected orientation only, {n_pos} affected position.')
        if n_ori > 0 and n_pos == 0:
            print('  Orientation-only jumps with position unaffected are typical of the')
            print('  planar-marker tilt ambiguity, strongest when viewed close to face-on.')
    else:
        print()

    mu_p, sd_p = e_in.mean(0), e_in.std(0, ddof=1)
    print('- Position (outliers removed):')
    for a in range(3):
        kind = ('systematic bias dominates (calibration/offset, not detection noise)'
                if abs(mu_p[a]) > 3 * sd_p[a] else
                'random scatter dominates (detection noise / motion)')
        print(f'    {labels[a]:<13} bias {mu_p[a]:6.1f} mm, std {sd_p[a]:5.1f} mm -> {kind}')
    print(f'    Largest bias: {labels[np.argmax(np.abs(mu_p))]}.  '
          f'Least precise (largest std): {labels[np.argmax(sd_p)]}.')

    mu_o, sd_o = o_in.mean(0), o_in.std(0, ddof=1)
    print('- Orientation (outliers removed):')
    print(f'    Tilt precision (std about marker X/Y): {sd_o[0]:.2f} / {sd_o[1]:.2f} deg')
    print(f'    In-plane precision (std about normal): {sd_o[2]:.2f} deg')
    print(f'    Mean offsets {mu_o[0]:.2f} / {mu_o[1]:.2f} / {mu_o[2]:.2f} deg '
          f'(total {np.linalg.norm(mu_o):.2f} deg)')
    if np.all((np.abs(mu_o) > 3 * sd_o) | (np.abs(mu_o) < 0.2)):
        print('    Offsets are constant relative to the noise -> rigid-body/marker axis')
        print('    misalignment or camera extrinsic (BC_T_C) error, not ArUco error.')
    if sd_o[2] < sd_o[:2].min():
        print('    In-plane rotation is the most precise axis; tilt of the normal is the')
        print('    weakest, as expected for a single planar marker.')
    if static_trial and sd_o.max() < 0.15:
        print('    Precision is <= ~0.1 deg, close to the OptiTrack orientation noise floor,')
        print('    so quote it as an upper bound rather than an exact value.')

    if SAVE_FIGURES:
        for fname, fig in figs:
            fig.savefig(bag / f'{name}_{fname}.png', dpi=200)
        print(f'\nSaved figures to {bag}')
    plt.show()


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else BAG_FOLDER)