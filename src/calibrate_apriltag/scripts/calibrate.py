#!/usr/bin/env python3
"""
calibrate_bundle.py

Estimate relative poses of tags in a bundle for apriltag_ros tags.yaml.

Input: a CSV of detections with columns:
  frame_id,tag_id,tx,ty,tz,qx,qy,qz,qw
where [tx,ty,tz] and [qx,qy,qz,qw] are the pose of the TAG in the CAMERA frame
(i.e., T_cam_tag) for that frame. Units: meters. Quaternions are normalized.

Typical sources:
  - apriltag_ros detections converted to CSV
  - your own PnP pipeline

Output: YAML snippet for apriltag_ros tag bundle layout.

Usage:
  python calibrate_bundle.py detections.csv --master 0 --bundle-name my_bundle \
      --size 0.050 > bundle.yaml

Requires: numpy, scipy, pyyaml
"""
import argparse
import csv
import math
from collections import defaultdict
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R
import yaml

# ---------- SE3 helpers ----------
def se3_from_t_q(t, q):  # t: (3,), q: (4,) [x,y,z,w]
    return (R.from_quat(q), np.asarray(t, dtype=float))

def se3_inv(T):
    Rw, tw = T
    Rin = Rw.inv()
    return (Rin, -Rin.apply(tw))

def se3_mul(A, B):
    Ra, ta = A
    Rb, tb = B
    Rc = Ra * Rb
    tc = ta + Ra.apply(tb)
    return (Rc, tc)

def se3_to_vec(T):
    Rw, tw = T
    aa = Rw.as_rotvec()
    return np.concatenate([tw, aa])

def se3_from_vec(v6):
    t = np.asarray(v6[:3], dtype=float)
    aa = np.asarray(v6[3:], dtype=float)
    return (R.from_rotvec(aa), t)

def pose_error(T_pred, T_meas):
    # residual of T_pred * inv(T_meas)
    Rpred, tpred = T_pred
    Rmeas, tmeas = T_meas
    Rin, tin = se3_inv((Rmeas, tmeas))
    Terr = se3_mul((Rpred, tpred), (Rin, tin))
    return se3_to_vec(Terr)

# ---------- IO ----------
def load_detections(csv_path):
    # returns dict frame_id -> list of (tag_id, T_cam_tag)
    frames = defaultdict(list)
    tag_ids = set()
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        needed = {"frame_id","tag_id","tx","ty","tz","qx","qy","qz","qw"}
        if not needed.issubset(reader.fieldnames):
            raise ValueError(f"CSV must have columns: {sorted(needed)}")
        for row in reader:
            k = int(row["frame_id"])
            i = int(row["tag_id"])
            t = [float(row["tx"]), float(row["ty"]), float(row["tz"])]
            q = [float(row["qx"]), float(row["qy"]), float(row["qz"]), float(row["qw"])]
            # normalize quaternion defensively
            q = np.asarray(q); q = q / np.linalg.norm(q)
            frames[k].append((i, se3_from_t_q(t, q)))
            tag_ids.add(i)
    frames = dict(sorted(frames.items()))
    return frames, sorted(tag_ids)

# ---------- Initialization ----------
def initialize_states(frames, tag_ids, master_id):
    # Unknowns:
    #   X_k = T_cam_k_bundle  for each frame k
    #   Y_i = T_bundle_tag_i  for each tag i, with Y_master fixed = Identity
    Ks = sorted(frames.keys())
    X = {k: (R.identity(), np.zeros(3)) for k in Ks}
    Y = {i: (R.identity(), np.zeros(3)) for i in tag_ids}

    # Seed X_k using master if present in a frame: X_k = T_cam_master, since Y_master = I
    for k in Ks:
        obs = dict(frames[k])
        if master_id in obs:
            X[k] = obs[master_id]

    # Seed other tags using frames that saw both tag i and master
    for i in tag_ids:
        if i == master_id:
            continue
        acc_R = []
        acc_t = []
        for k in Ks:
            obs = dict(frames[k])
            if (master_id in obs) and (i in obs):
                Xk = X[k]
                T_cam_tag_i = obs[i]
                # Y_i ≈ X_k^{-1} * T_cam_tag_i
                Y_est = se3_mul(se3_inv(Xk), T_cam_tag_i)
                acc_R.append(Y_est[0].as_matrix())
                acc_t.append(Y_est[1])
        if acc_R:
            # average rotation via quaternion mean
            Rs = R.from_matrix(np.stack(acc_R))
            q_mean = _quat_avg(Rs.as_quat())
            t_mean = np.mean(np.stack(acc_t, axis=0), axis=0)
            Y[i] = (R.from_quat(q_mean), t_mean)
        else:
            # fallback
            Y[i] = (R.identity(), np.zeros(3))

    return Ks, X, Y

def _quat_avg(quats_xyzw):
    # quats: N x 4 [x,y,z,w]
    A = np.zeros((4,4))
    for q in quats_xyzw:
        q = q / np.linalg.norm(q)
        A += np.outer(q, q)
    eigvals, eigvecs = np.linalg.eigh(A)
    q = eigvecs[:, np.argmax(eigvals)]
    # enforce scalar-last convention
    return q

# ---------- Parameter packing ----------
def pack_params(Ks, X, Y, tag_ids, master_id):
    cam_idx = {k: idx for idx, k in enumerate(Ks)}
    tag_idx = {}
    j = 0
    for i in tag_ids:
        if i == master_id:
            continue
        tag_idx[i] = j
        j += 1
    # build vector
    vecs = []
    for k in Ks:
        vecs.append(se3_to_vec(X[k]))
    for i in tag_ids:
        if i == master_id:
            continue
        vecs.append(se3_to_vec(Y[i]))
    return np.concatenate(vecs), cam_idx, tag_idx

def unpack_params(theta, Ks, tag_ids, master_id):
    X = {}
    Y = {}
    n_cam = len(Ks)
    n_tag = len(tag_ids) - 1
    assert theta.size == 6*n_cam + 6*n_tag
    off = 0
    for k in Ks:
        X[k] = se3_from_vec(theta[off:off+6]); off += 6
    for i in tag_ids:
        if i == master_id:
            continue
        Y[i] = se3_from_vec(theta[off:off+6]); off += 6
    Y[master_id] = (R.identity(), np.zeros(3))
    return X, Y

# ---------- Residuals ----------
def build_residuals(frames, Ks, tag_ids, master_id):
    obs_list = []
    for k in Ks:
        for i, T_cam_tag in frames[k]:
            obs_list.append((k, i, T_cam_tag))
    def fun(theta):
        X, Y = unpack_params(theta, Ks, tag_ids, master_id)
        res = []
        for k, i, T_meas in obs_list:
            T_pred = se3_mul(X[k], Y[i])  # T_cam_tag = X_k * Y_i
            res.append(pose_error(T_pred, T_meas))
        return np.concatenate(res)
    return fun

# ---------- YAML output ----------
def to_yaml(bundle_name, tag_ids, Y, default_size=None):
    layout = []
    for i in sorted(tag_ids):
        Ri, ti = Y[i]
        q = Ri.as_quat()  # x y z w
        entry = {
            "id": int(i),
            "x": float(ti[0]),
            "y": float(ti[1]),
            "z": float(ti[2]),
            "qx": float(q[0]),
            "qy": float(q[1]),
            "qz": float(q[2]),
            "qw": float(q[3]),
        }
        if default_size is not None:
            entry["size"] = float(default_size)
        layout.append(entry)
    return {
        "tag_bundles": [
            {
                "name": bundle_name,
                "layout": layout
            }
        ]
    }

# ---------- Main ----------
def main():
    ap = argparse.ArgumentParser(description="AprilTag bundle calibration in Python")
    ap.add_argument("csv", help="detections CSV: frame_id,tag_id,tx,ty,tz,qx,qy,qz,qw (T_cam_tag)")
    ap.add_argument("--master", type=int, required=True, help="master tag id to fix as bundle origin")
    ap.add_argument("--bundle-name", type=str, default="bundle")
    ap.add_argument("--size", type=float, default=None, help="optional tag edge size in meters to include in YAML")
    ap.add_argument("--robust", action="store_true", help="use robust loss")
    ap.add_argument("--max-iters", type=int, default=200)
    args = ap.parse_args()

    frames, tag_ids = load_detections(args.csv)
    if args.master not in tag_ids:
        raise ValueError("Master tag id not present in detections")

    Ks, X0, Y0 = initialize_states(frames, tag_ids, args.master)
    theta0, _, _ = pack_params(Ks, X0, Y0, tag_ids, args.master)
    fun = build_residuals(frames, Ks, tag_ids, args.master)

    loss = "soft_l1" if args.robust else "linear"
    res = least_squares(fun, theta0, loss=loss, max_nfev=args.max_iters, verbose=1)

    X_opt, Y_opt = unpack_params(res.x, Ks, tag_ids, args.master)

    # Sanity: re-anchor so the master tag is exactly identity (already enforced) and normalize quats
    for i in tag_ids:
        Rw, tw = Y_opt[i]
        Y_opt[i] = (R.from_quat(Rw.as_quat()/np.linalg.norm(Rw.as_quat())), tw)

    out = to_yaml(args.bundle_name, tag_ids, Y_opt, args.size)
    yaml.safe_dump(out, stream=None)
    print(yaml.safe_dump(out, sort_keys=False))

if __name__ == "__main__":
    main()
