#!/usr/bin/env python3
"""
analisis_false_minima.py — Peta Ambiguitas LOKAL dari multi-run B-2 style
==========================================================================
TUJUAN:
  Temukan kapan/di mana AMCL diverge dari GT, cluster false minima,
  ekstrak pemicu geometris (posisi + heading robot saat partikel salah mulai menang),
  dan klasifikasi tiap cluster.

CATATAN EKSPLISIT (WAJIB BACA SEBELUM INTERPRETASI):
  Peta ini HANYA mencakup ambiguitas LOKAL (dalam radius initial particle spread,
  cov_xx=0.5 → σ=0.71m → 3σ≈2.1m dari spawn).

  Ambiguitas GLOBAL (kidnap/penalty skenario RoboCup) = MASALAH TERBUKA & TERBUKTI FATAL:
  cov_xx=8.13m² setelah 180s robot bergerak dari distribusi seragam = AMCL tidak bisa
  global relocalize sama sekali. Peta ini TIDAK mencakup skenario itu.

Usage:
  python3 analisis_false_minima.py pose_eval_langkahB2_run*.csv
  python3 analisis_false_minima.py *.csv --spawn-x -0.363 --spawn-y 0.0 --out-prefix b2
"""

import csv, math, sys, argparse, os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

# ─── konstanta ────────────────────────────────────────────────────────────────
EARLY_DIV_THR  = 0.5    # |err_amcl_pos_m| > 0.5m = awal divergensi terdeteksi
FAILURE_THR    = 1.5    # err > 1.5m = collapse/failure
FAILURE_SUSTAIN = 5     # harus bertahan N sample berturutan agar dihitung failure
CLUSTER_RADIUS = 0.5    # radius cluster posisi false minimum (m)
N_HEADING      = 15     # sample ke depan untuk compute heading robot
LATE_WINDOW_S  = 30.0   # window akhir run untuk hitung posisi "false minimum menetap"


# ─── dataclass ────────────────────────────────────────────────────────────────
@dataclass
class DivergenceEvent:
    t_s: float           # waktu divergensi awal
    gt_x: float          # posisi GT saat divergensi
    gt_y: float
    gt_yaw_deg: float    # heading robot (dari gt_yaw_deg atau computed)
    amcl_x: float        # posisi AMCL saat divergensi
    amcl_y: float
    delta_x: float       # amcl_x - gt_x (arah bias)
    delta_y: float

@dataclass
class RunResult:
    name: str
    n_rows: int
    duration_s: float
    rmse: float
    early_div: Optional[DivergenceEvent]   # divergensi awal (err > 0.5m)
    failure: Optional[DivergenceEvent]     # collapse (err > 1.5m, sustained)
    false_min_x: Optional[float]           # posisi AMCL di akhir run (false minimum menetap)
    false_min_y: Optional[float]
    false_min_offset_x: Optional[float]   # false_min_x - gt_x (rata-rata akhir run)
    false_min_offset_y: Optional[float]
    outcome: str                           # 'BENAR' / 'FALSE_MIN' / 'TIDAK_KONVERGEN'
    t_arr: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))
    gt_x_arr: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))
    gt_y_arr: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))
    amcl_x_arr: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))
    amcl_y_arr: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))
    err_arr: np.ndarray = field(repr=False, default_factory=lambda: np.array([]))


# ─── loader ───────────────────────────────────────────────────────────────────
def load_run(path: str) -> List[dict]:
    rows = []
    with open(path) as f:
        for row in csv.DictReader(f):
            clean = {}
            for k, v in row.items():
                try:
                    clean[k] = float(v) if v not in ('nan', '', 'None') else float('nan')
                except ValueError:
                    clean[k] = float('nan')
            rows.append(clean)
    return rows


def col(rows: List[dict], key: str) -> np.ndarray:
    return np.array([r.get(key, float('nan')) for r in rows], dtype=float)


# ─── analisis satu run ────────────────────────────────────────────────────────
def heading_at(gt_x_arr, gt_y_arr, idx, n_fwd=N_HEADING) -> float:
    """Hitung heading robot (derajat) dari vektor kecepatan GT di sekitar idx."""
    fwd = min(idx + n_fwd, len(gt_x_arr) - 1)
    dx = gt_x_arr[fwd] - gt_x_arr[idx]
    dy = gt_y_arr[fwd] - gt_y_arr[idx]
    if abs(dx) < 1e-5 and abs(dy) < 1e-5:
        return float('nan')
    return math.degrees(math.atan2(dy, dx))


def find_sustained_threshold(err_arr, threshold, sustain=FAILURE_SUSTAIN):
    """Cari indeks pertama di mana err > threshold selama SUSTAIN sample berturutan."""
    count = 0
    for i in range(len(err_arr)):
        if not math.isnan(err_arr[i]) and err_arr[i] > threshold:
            count += 1
            if count >= sustain:
                return i - sustain + 1
        else:
            count = 0
    return None


def find_first_threshold(err_arr, threshold):
    """Cari indeks pertama di mana err > threshold (tanpa sustain)."""
    for i in range(len(err_arr)):
        if not math.isnan(err_arr[i]) and err_arr[i] > threshold:
            return i
    return None


def analyze_run(rows: List[dict], name: str) -> RunResult:
    t       = col(rows, 't_s')
    gt_x    = col(rows, 'gt_x')
    gt_y    = col(rows, 'gt_y')
    gt_yaw  = col(rows, 'gt_yaw_deg')
    amcl_x  = col(rows, 'amcl_x')
    amcl_y  = col(rows, 'amcl_y')
    err     = col(rows, 'err_amcl_pos_m')

    valid_err = err[~np.isnan(err)]
    rmse = float(np.sqrt(np.mean(valid_err**2))) if len(valid_err) > 0 else float('nan')
    duration = float(t[-1] - t[0]) if len(t) > 1 else 0.0

    # ── early divergence (err > 0.5m, first occurrence) ──────────────────────
    early_div = None
    idx_early = find_first_threshold(err, EARLY_DIV_THR)
    if idx_early is not None:
        # Gunakan gt_yaw jika tersedia, fallback ke computed heading
        yaw = float(gt_yaw[idx_early]) if not math.isnan(gt_yaw[idx_early]) \
              else heading_at(gt_x, gt_y, idx_early)
        early_div = DivergenceEvent(
            t_s=float(t[idx_early]),
            gt_x=float(gt_x[idx_early]), gt_y=float(gt_y[idx_early]),
            gt_yaw_deg=yaw,
            amcl_x=float(amcl_x[idx_early]), amcl_y=float(amcl_y[idx_early]),
            delta_x=float(amcl_x[idx_early] - gt_x[idx_early]),
            delta_y=float(amcl_y[idx_early] - gt_y[idx_early]),
        )

    # ── failure/collapse (err > 1.5m, sustained) ─────────────────────────────
    failure = None
    idx_fail = find_sustained_threshold(err, FAILURE_THR)
    if idx_fail is not None:
        yaw = float(gt_yaw[idx_fail]) if not math.isnan(gt_yaw[idx_fail]) \
              else heading_at(gt_x, gt_y, idx_fail)
        failure = DivergenceEvent(
            t_s=float(t[idx_fail]),
            gt_x=float(gt_x[idx_fail]), gt_y=float(gt_y[idx_fail]),
            gt_yaw_deg=yaw,
            amcl_x=float(amcl_x[idx_fail]), amcl_y=float(amcl_y[idx_fail]),
            delta_x=float(amcl_x[idx_fail] - gt_x[idx_fail]),
            delta_y=float(amcl_y[idx_fail] - gt_y[idx_fail]),
        )

    # ── false minimum menetap (rata-rata AMCL di window akhir run) ───────────
    mask_late = t > (t[-1] - LATE_WINDOW_S)
    mask_high_err = err > FAILURE_THR
    mask_both = mask_late & mask_high_err & ~np.isnan(amcl_x) & ~np.isnan(amcl_y)

    false_min_x = false_min_y = None
    false_min_off_x = false_min_off_y = None
    if mask_both.sum() >= 3:
        false_min_x = float(np.nanmean(amcl_x[mask_both]))
        false_min_y = float(np.nanmean(amcl_y[mask_both]))
        gt_late_x   = float(np.nanmean(gt_x[mask_both]))
        gt_late_y   = float(np.nanmean(gt_y[mask_both]))
        false_min_off_x = false_min_x - gt_late_x
        false_min_off_y = false_min_y - gt_late_y

    # ── outcome ──────────────────────────────────────────────────────────────
    if rmse < 0.8 and idx_fail is None:
        outcome = 'BENAR'
    elif idx_fail is not None:
        outcome = 'FALSE_MIN'
    else:
        outcome = 'TIDAK_KONVERGEN'

    return RunResult(
        name=name, n_rows=len(rows), duration_s=duration, rmse=rmse,
        early_div=early_div, failure=failure,
        false_min_x=false_min_x, false_min_y=false_min_y,
        false_min_offset_x=false_min_off_x, false_min_offset_y=false_min_off_y,
        outcome=outcome,
        t_arr=t, gt_x_arr=gt_x, gt_y_arr=gt_y,
        amcl_x_arr=amcl_x, amcl_y_arr=amcl_y, err_arr=err,
    )


# ─── clustering false minima ──────────────────────────────────────────────────
def cluster_false_minima(results: List[RunResult], radius=CLUSTER_RADIUS) -> List[dict]:
    """Cluster posisi false minimum (amcl_x, amcl_y di akhir failure run)."""
    clusters = []
    for r in results:
        if r.outcome != 'FALSE_MIN' or r.false_min_x is None:
            continue
        fx, fy = r.false_min_x, r.false_min_y
        placed = False
        for c in clusters:
            cx, cy = c['cx'], c['cy']
            if math.sqrt((fx - cx)**2 + (fy - cy)**2) < radius:
                c['runs'].append(r)
                c['cx'] = sum(rr.false_min_x for rr in c['runs']) / len(c['runs'])
                c['cy'] = sum(rr.false_min_y for rr in c['runs']) / len(c['runs'])
                placed = True
                break
        if not placed:
            clusters.append({'cx': fx, 'cy': fy, 'runs': [r]})
    return sorted(clusters, key=lambda c: -len(c['runs']))


def classify_cluster(cluster: dict, spawn_x: float, spawn_y: float) -> str:
    """
    Klasifikasi false minimum:
    - SIMETRI_180: AMCL ≈ rotasi 180° dari spawn (cx≈-spawn_x, cy≈-spawn_y)
    - MIRROR_Y: AMCL ≈ refleksi sumbu-X (cx≈spawn_x, cy≈-spawn_y)
    - MIRROR_X: AMCL ≈ refleksi sumbu-Y (cx≈-spawn_x, cy≈spawn_y)
    - LOCAL_ALIAS: false min dekat spawn (dalam 1.5m) — bukan simetri global
    - UNKNOWN: tidak cocok pola apapun
    """
    cx, cy = cluster['cx'], cluster['cy']
    thr = 0.4  # toleransi untuk pencocokan simetri

    if (abs(cx - (-spawn_x)) < thr and abs(cy - (-spawn_y)) < thr):
        return 'SIMETRI_180'
    if (abs(cx - spawn_x) < thr and abs(cy - (-spawn_y)) < thr):
        return 'MIRROR_Y'
    if (abs(cx - (-spawn_x)) < thr and abs(cy - spawn_y) < thr):
        return 'MIRROR_X'
    dist = math.sqrt((cx - spawn_x)**2 + (cy - spawn_y)**2)
    if dist < 1.5:
        return 'LOCAL_ALIAS'
    return 'UNKNOWN'


# ─── report ───────────────────────────────────────────────────────────────────
def print_report(results: List[RunResult], clusters: List[dict],
                 spawn_x: float, spawn_y: float):
    n = len(results)
    n_benar  = sum(1 for r in results if r.outcome == 'BENAR')
    n_false  = sum(1 for r in results if r.outcome == 'FALSE_MIN')
    n_other  = n - n_benar - n_false

    sep = '=' * 65

    print(f"\n{sep}")
    print("PETA AMBIGUITAS LOKAL — ANALISIS MULTI-RUN B-2")
    print(f"N={n} runs | spawn=({spawn_x},{spawn_y})")
    print(sep)

    # Per-run table
    print(f"\n{'Run':<20}  {'RMSE':>6}  {'Outcome':<16}  {'Early div':>10}  {'Heading':>8}")
    print('─' * 70)
    for r in results:
        ediv = f"t={r.early_div.t_s:.1f}s" if r.early_div else '—'
        head = f"{r.early_div.gt_yaw_deg:.0f}°" if r.early_div and not math.isnan(r.early_div.gt_yaw_deg) else '—'
        print(f"{r.name:<20}  {r.rmse:>6.3f}m  {r.outcome:<16}  {ediv:>10}  {head:>8}")

    # Summary
    print(f"\n{sep}")
    print(f"RINGKASAN: {n_benar}/{n} BENAR ({100*n_benar/n:.0f}%)  "
          f"| {n_false}/{n} FALSE_MIN ({100*n_false/n:.0f}%)")

    # False minima clusters
    print(f"\n{sep}")
    print("CLUSTER FALSE MINIMA (posisi AMCL menetap setelah collapse)")
    print(sep)
    if not clusters:
        print("  Tidak ada cluster (semua run benar atau tidak ada false minimum).")
    for i, c in enumerate(clusters):
        label = classify_cluster(c, spawn_x, spawn_y)
        n_c   = len(c['runs'])
        print(f"\n  CLUSTER {i+1}: posisi=({c['cx']:+.3f}, {c['cy']:+.3f})  "
              f"N={n_c}/{n_false} fail  [{label}]")
        # Geometric triggers
        triggers = [r.failure for r in c['runs'] if r.failure is not None]
        if triggers:
            mean_gt_x = sum(e.gt_x for e in triggers) / len(triggers)
            mean_gt_y = sum(e.gt_y for e in triggers) / len(triggers)
            mean_head = [e.gt_yaw_deg for e in triggers if not math.isnan(e.gt_yaw_deg)]
            mean_t    = sum(e.t_s for e in triggers) / len(triggers)
            print(f"  Pemicu geometris (rata-rata saat collapse t>{FAILURE_THR}m sustained):")
            print(f"    Robot GT pos  : ({mean_gt_x:+.3f}, {mean_gt_y:+.3f})")
            if mean_head:
                mh = sum(mean_head) / len(mean_head)
                print(f"    Robot heading  : {mh:.0f}° (0°=kanan/+x, 90°=atas/+y)")
            print(f"    t rata-rata   : {mean_t:.1f}s")
            # Offset AMCL vs GT
            off_x = [r.false_min_offset_x for r in c['runs'] if r.false_min_offset_x is not None]
            off_y = [r.false_min_offset_y for r in c['runs'] if r.false_min_offset_y is not None]
            if off_x:
                print(f"    Offset konsisten: Δx={sum(off_x)/len(off_x):+.3f}m "
                      f"Δy={sum(off_y)/len(off_y):+.3f}m  "
                      f"(AMCL - GT saat failure)")
        # Implikasi detektor
        print(f"  Implikasi untuk feature layer: {_detector_hint(label, c, spawn_x, spawn_y)}")

    # Early divergence analysis
    early_divs = [r.early_div for r in results if r.early_div is not None and r.outcome == 'FALSE_MIN']
    if early_divs:
        print(f"\n{sep}")
        print("ANALISIS DIVERGENSI AWAL (saat err pertama > 0.5m, run GAGAL saja)")
        print(sep)
        mean_t    = sum(e.t_s   for e in early_divs) / len(early_divs)
        mean_gt_x = sum(e.gt_x  for e in early_divs) / len(early_divs)
        mean_gt_y = sum(e.gt_y  for e in early_divs) / len(early_divs)
        heads_ok  = [e.gt_yaw_deg for e in early_divs if not math.isnan(e.gt_yaw_deg)]
        mean_head = sum(heads_ok) / len(heads_ok) if heads_ok else float('nan')
        mean_dx   = sum(e.delta_x for e in early_divs) / len(early_divs)
        mean_dy   = sum(e.delta_y for e in early_divs) / len(early_divs)
        print(f"  Waktu divergensi awal (rata): t={mean_t:.1f}s")
        print(f"  GT posisi (rata-rata)        : ({mean_gt_x:+.3f}, {mean_gt_y:+.3f})")
        if not math.isnan(mean_head):
            print(f"  Robot heading saat divergensi: {mean_head:.0f}°")
        print(f"  AMCL bias awal (Δx, Δy)     : ({mean_dx:+.3f}, {mean_dy:+.3f})")
        print(f"  → AMCL sudah {'KIRI' if mean_dx < 0 else 'KANAN'} dari GT sejak awal "
              f"({abs(mean_dx):.2f}m) — die cast SEBELUM/SAAT t={mean_t:.0f}s pertama")

    # CATATAN EKSPLISIT — WAJIB
    print(f"\n{sep}")
    print("!! CATATAN EKSPLISIT — BACA SEBELUM MEMBUAT KEPUTUSAN STRATEGI !!")
    print(sep)
    print()
    print("  (1) Peta ini = ambiguitas LOKAL saja (radius ±2.1m dari spawn).")
    print("      Ini relevan untuk mengurangi false minima saat start normal.")
    print()
    print("  (2) Ambiguitas GLOBAL (kidnap/penalty RoboCup) = MASALAH TERBUKA.")
    print("      Terbukti fatal: AMCL cov_xx=8.13m² setelah 180s robot bergerak")
    print("      dari distribusi seragam. Scan garis tidak bisa disambiguate lapangan.")
    print("      Feature layer WAJIB untuk kidnap recovery — peta ini tidak menyentuhnya.")
    print()
    print("  (3) Tighten cov_xx (stopgap) hanya membantu skenario (1).")
    print("      Ia TIDAK membantu skenario (2) dan bisa memperburuk recovery.")


def _detector_hint(label: str, cluster: dict, spawn_x: float, spawn_y: float) -> str:
    cx, cy = cluster['cx'], cluster['cy']
    if label == 'SIMETRI_180':
        return "Landmark yang membedakan KIRI vs KANAN lapangan (contoh: nomor/warna tim)"
    if label == 'MIRROR_Y':
        return "Landmark yang membedakan ATAS vs BAWAH lapangan (goal kiri vs kanan)"
    if label == 'MIRROR_X':
        return "Landmark yang membedakan sisi lapangan dalam sumbu Y"
    if label == 'LOCAL_ALIAS':
        return f"Landmark unik di radius ±0.5m dari ({cx:.2f},{cy:.2f}) — mungkin garis atau tanda dekat spawn"
    return "Perlu investigasi lebih lanjut (posisi tidak cocok pola simetri standar)"


# ─── plot ─────────────────────────────────────────────────────────────────────
def plot_trajectories(results: List[RunResult], out_prefix: str, spawn_x: float, spawn_y: float):
    n = len(results)
    ncols = min(3, n)
    nrows = math.ceil(n / ncols)
    fig, axes = plt.subplots(nrows, ncols, figsize=(6 * ncols, 5 * nrows))
    if n == 1:
        axes = np.array([[axes]])
    elif nrows == 1:
        axes = axes.reshape(1, -1)

    fig.suptitle(f'Trajektori GT vs AMCL — {n} run B-2\n'
                 f'Spawn=({spawn_x},{spawn_y})  [peta lokal, bukan global]', fontsize=11)

    for idx, r in enumerate(results):
        row, col_ = divmod(idx, ncols)
        ax = axes[row][col_]

        # Lapangan
        field = plt.Rectangle((-4.5, -3.0), 9.0, 6.0, fill=False, ec='black', lw=1.5)
        ax.add_patch(field)
        ax.axhline(0, color='gray', lw=0.4, ls=':')
        ax.axvline(0, color='gray', lw=0.4, ls=':')

        # Spawn
        ax.plot(spawn_x, spawn_y, 'g*', ms=12, zorder=10, label='spawn')

        # GT trajektori
        valid_gt = ~np.isnan(r.gt_x_arr) & ~np.isnan(r.gt_y_arr)
        if valid_gt.sum() > 1:
            ax.plot(r.gt_x_arr[valid_gt], r.gt_y_arr[valid_gt],
                    'b-', lw=1.8, alpha=0.7, label='GT')

        # AMCL trajektori (warna = error magnitude)
        valid_ac = ~np.isnan(r.amcl_x_arr) & ~np.isnan(r.amcl_y_arr) & ~np.isnan(r.err_arr)
        if valid_ac.sum() > 1:
            from matplotlib.collections import LineCollection
            pts  = np.array([r.amcl_x_arr[valid_ac], r.amcl_y_arr[valid_ac]]).T.reshape(-1, 1, 2)
            segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
            norm_err = np.clip(r.err_arr[valid_ac][:-1] / 3.0, 0, 1)
            lc = LineCollection(segs, cmap='RdYlGn_r', norm=plt.Normalize(0, 1), lw=2, alpha=0.85)
            lc.set_array(norm_err)
            ax.add_collection(lc)

        # Tandai divergensi awal
        if r.early_div:
            ax.plot(r.early_div.gt_x, r.early_div.gt_y, 'mo', ms=8, zorder=11,
                    label=f'div t={r.early_div.t_s:.0f}s')

        # Tandai collapse
        if r.failure:
            ax.plot(r.failure.gt_x, r.failure.gt_y, 'kX', ms=10, zorder=12,
                    label=f'fail t={r.failure.t_s:.0f}s')

        # False minimum menetap
        if r.false_min_x is not None:
            ax.plot(r.false_min_x, r.false_min_y, 'r^', ms=10, zorder=12,
                    label=f'false min ({r.false_min_x:.2f},{r.false_min_y:.2f})')

        color = 'green' if r.outcome == 'BENAR' else ('red' if r.outcome == 'FALSE_MIN' else 'orange')
        ax.set_title(f'{r.name}\nRMSE={r.rmse:.3f}m [{r.outcome}]',
                     fontsize=8, color=color)
        ax.set_xlim(-4.8, 4.8); ax.set_ylim(-3.3, 3.3)
        ax.set_aspect('equal')
        ax.legend(fontsize=6, loc='upper right')
        ax.grid(True, alpha=0.2)

    # Sembunyikan axes kosong
    for idx in range(n, nrows * ncols):
        row, col_ = divmod(idx, ncols)
        axes[row][col_].set_visible(False)

    out = f'{out_prefix}_trajektori_multirun.png'
    plt.tight_layout()
    plt.savefig(out, dpi=110)
    print(f"Plot trajektori: {out}")
    plt.close()


def plot_error_timeseries(results: List[RunResult], out_prefix: str):
    n = len(results)
    fig, ax = plt.subplots(figsize=(14, 5))
    ax.set_title(f'Error AMCL vs Waktu — {n} run\n'
                 f'(Hijau = BENAR, Merah = FALSE_MIN)', fontsize=10)

    cmap_good = plt.cm.Greens
    cmap_bad  = plt.cm.Reds
    n_good = max(1, sum(1 for r in results if r.outcome == 'BENAR'))
    n_bad  = max(1, sum(1 for r in results if r.outcome == 'FALSE_MIN'))
    ig = ib = 0
    for r in results:
        valid = ~np.isnan(r.err_arr)
        if r.outcome == 'BENAR':
            c = cmap_good(0.4 + 0.5 * ig / n_good); ig += 1
        else:
            c = cmap_bad(0.4 + 0.5 * ib / n_bad);  ib += 1
        ax.plot(r.t_arr[valid], r.err_arr[valid], lw=1.0, color=c,
                alpha=0.8, label=f'{r.name} ({r.rmse:.2f}m)')

    ax.axhline(FAILURE_THR,  color='orange', lw=1.5, ls='--', label=f'failure >{FAILURE_THR}m')
    ax.axhline(EARLY_DIV_THR, color='gray',  lw=1.0, ls=':',  label=f'early div >{EARLY_DIV_THR}m')
    ax.set_xlabel('t (s)'); ax.set_ylabel('Error posisi AMCL (m)')
    ax.set_ylim(0, None); ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7, loc='upper right', ncol=2)

    out = f'{out_prefix}_error_timeseries.png'
    plt.tight_layout()
    plt.savefig(out, dpi=110)
    print(f"Plot error timeseries: {out}")
    plt.close()


# ─── main ─────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('csvfiles', nargs='+', help='CSV dari localization_evaluator.py')
    parser.add_argument('--spawn-x', type=float, default=-0.363)
    parser.add_argument('--spawn-y', type=float, default=0.0)
    parser.add_argument('--out-prefix', default='false_minima', help='Prefix output file')
    args = parser.parse_args()

    # Load semua run
    results = []
    for path in sorted(args.csvfiles):
        name = os.path.basename(path).replace('.csv', '')
        try:
            rows = load_run(path)
            if not rows:
                print(f"[SKIP] {path}: kosong")
                continue
            r = analyze_run(rows, name)
            results.append(r)
            print(f"[LOAD] {name}: {r.n_rows} rows, {r.duration_s:.0f}s, "
                  f"RMSE={r.rmse:.3f}m, outcome={r.outcome}")
        except Exception as e:
            print(f"[ERROR] {path}: {e}")

    if not results:
        print("Tidak ada data. Keluar.")
        sys.exit(1)

    # Cluster false minima
    clusters = cluster_false_minima(results, radius=CLUSTER_RADIUS)

    # Report
    print_report(results, clusters, args.spawn_x, args.spawn_y)

    # Plot
    plot_trajectories(results, args.out_prefix, args.spawn_x, args.spawn_y)
    plot_error_timeseries(results, args.out_prefix)

    print(f"\nSelesai. Jalankan analisis lagi dengan CSV baru jika run tambahan selesai.")


if __name__ == '__main__':
    main()
