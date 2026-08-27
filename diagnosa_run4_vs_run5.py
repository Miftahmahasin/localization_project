#!/usr/bin/env python3
"""
diagnosa_run4_vs_run5.py — Bedah Run 4 (3.37m) vs Run 5 (0.40m)
Tujuan: tentukan apakah collapse Run 4 DISKRET (trigger di zona) atau GRADUAL (drift).
Output: diagnosa_run4vs5_*.png
"""
import csv, math, sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

RUN4 = 'pose_eval_langkahB1_run4.csv'
RUN5 = 'pose_eval_langkahB1_run5.csv'

# Field RoboCup standard (half-field kiri = negatif x)
FIELD_X = (-4.5, 4.5)
FIELD_Y = (-3.0, 3.0)
# Goal area: x < -3.9 atau x > 3.9
GOAL_ZONE_X = 3.9


def load(path):
    rows = []
    with open(path) as f:
        for row in csv.DictReader(f):
            try:
                r = {k: float(v) if v != 'nan' and v != '' else float('nan')
                     for k, v in row.items()}
                rows.append(r)
            except ValueError:
                pass
    return rows


def col(rows, key):
    return np.array([r.get(key, float('nan')) for r in rows])


def find_collapse_point(t, err, threshold=1.5):
    """Cari titik pertama di mana error melewati threshold — jika ada lompatan tiba-tiba."""
    for i in range(1, len(err)):
        if not math.isnan(err[i]) and err[i] > threshold:
            return i, t[i]
    return None, None


r4 = load(RUN4)
r5 = load(RUN5)

t4 = col(r4, 't_s');  t5 = col(r5, 't_s')
gt4x = col(r4, 'gt_x'); gt4y = col(r4, 'gt_y')
gt5x = col(r5, 'gt_x'); gt5y = col(r5, 'gt_y')
amcl4x = col(r4, 'amcl_x'); amcl4y = col(r4, 'amcl_y')
amcl5x = col(r5, 'amcl_x'); amcl5y = col(r5, 'amcl_y')
ekf4x = col(r4, 'ekf_x'); ekf4y = col(r4, 'ekf_y')
ekf5x = col(r5, 'ekf_x'); ekf5y = col(r5, 'ekf_y')
err4 = col(r4, 'err_amcl_pos_m'); err5 = col(r5, 'err_amcl_pos_m')

# ------------------------------------------------------------------
# FIG 1: Error vs waktu (+ threshold line) — tampilkan karakter collapse
# ------------------------------------------------------------------
fig1, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=False)
fig1.suptitle('DIAGNOSA: Error AMCL vs Waktu\n(Run 4=3.37m RMSE  vs  Run 5=0.40m RMSE)', fontsize=12)

for ax, t, err, label, color in [
    (axes[0], t4, err4, 'Run 4 (COLLAPSE)', 'red'),
    (axes[1], t5, err5, 'Run 5 (BAIK)',     'green'),
]:
    ax.plot(t, err, color=color, lw=1.2, label='err_amcl_pos_m')
    ax.axhline(1.5, color='orange', lw=1, ls='--', label='threshold 1.5m')
    ax.axhline(0.5, color='gray',   lw=0.8, ls=':', label='threshold 0.5m')
    ax.set_title(label, fontsize=10)
    ax.set_xlabel('t (s)'); ax.set_ylabel('Error posisi (m)')
    ax.set_ylim(0, max(np.nanmax(err)*1.05, 2.0))
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # Tandai titik collapse (pertama kali > 1.5m)
    idx, tc = find_collapse_point(list(t), list(err), threshold=1.5)
    if idx is not None:
        ax.axvline(tc, color='purple', lw=1.5, ls='--', label=f'collapse t={tc:.1f}s')
        ax.legend(fontsize=8)
        print(f"[{label}] Collapse pertama kali err>1.5m: t={tc:.1f}s  idx={idx}")
    else:
        print(f"[{label}] Tidak ada collapse >1.5m")

plt.tight_layout()
plt.savefig('diagnosa_run4vs5_error_time.png', dpi=120)
print("Saved: diagnosa_run4vs5_error_time.png")
plt.close()

# ------------------------------------------------------------------
# FIG 2: Trajektori XY (GT vs AMCL) — side-by-side
# ------------------------------------------------------------------
fig2, axes = plt.subplots(1, 2, figsize=(14, 6))
fig2.suptitle('Trajektori XY: GT (biru) vs AMCL (merah/hijau)\nBingkai = lapangan RoboCup', fontsize=11)

for ax, gt_x, gt_y, est_x, est_y, t, err, label, ecolor in [
    (axes[0], gt4x, gt4y, amcl4x, amcl4y, t4, err4, 'Run 4 (RMSE=3.37m)', 'red'),
    (axes[1], gt5x, gt5y, amcl5x, amcl5y, t5, err5, 'Run 5 (RMSE=0.40m)', 'green'),
]:
    # Lapangan
    field = plt.Rectangle((FIELD_X[0], FIELD_Y[0]),
                            FIELD_X[1]-FIELD_X[0], FIELD_Y[1]-FIELD_Y[0],
                            fill=False, ec='black', lw=1.5)
    ax.add_patch(field)
    # Goal zone
    for xside in [-4.5, 3.9]:
        ax.axvspan(xside if xside < 0 else xside, 4.5 if xside > 0 else -3.9,
                   alpha=0.08, color='yellow')

    # Trajektori GT
    ax.plot(gt_x, gt_y, 'b-', lw=1.8, alpha=0.8, label='GT')
    ax.plot(gt_x[0], gt_y[0], 'bs', ms=8)  # start
    ax.plot(gt_x[-1], gt_y[-1], 'b^', ms=8)  # end

    # Trajektori AMCL (diwarnai berdasarkan error — gradient dari hijau ke merah)
    valid = ~np.isnan(est_x) & ~np.isnan(est_y) & ~np.isnan(err)
    if valid.sum() > 1:
        pts = np.array([est_x[valid], est_y[valid]]).T.reshape(-1, 1, 2)
        segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
        from matplotlib.collections import LineCollection
        from matplotlib.cm import RdYlGn
        err_v = err[valid]
        norm_err = np.clip(err_v[:-1] / 3.0, 0, 1)  # normalize 0-3m → 0-1
        colors = RdYlGn(1.0 - norm_err)  # merah = error tinggi, hijau = rendah
        lc = LineCollection(segs, colors=colors, lw=2.0, alpha=0.9)
        ax.add_collection(lc)
        ax.plot(est_x[valid][0], est_y[valid][0], 'o', color=ecolor, ms=8, label='AMCL start')

    # Tandai titik collapse di trajektori GT
    idx_c, tc = find_collapse_point(list(t), list(err), threshold=1.5)
    if idx_c is not None:
        ax.plot(gt_x[idx_c], gt_y[idx_c], 'k*', ms=15, label=f'collapse t={tc:.0f}s\nGT=({gt_x[idx_c]:.2f},{gt_y[idx_c]:.2f})')

    ax.set_xlim(FIELD_X[0]-0.3, FIELD_X[1]+0.3)
    ax.set_ylim(FIELD_Y[0]-0.3, FIELD_Y[1]+0.3)
    ax.set_aspect('equal')
    ax.set_title(label, fontsize=10)
    ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    ax.legend(fontsize=7, loc='upper right')
    ax.grid(True, alpha=0.2)

plt.tight_layout()
plt.savefig('diagnosa_run4vs5_trajektori.png', dpi=120)
print("Saved: diagnosa_run4vs5_trajektori.png")
plt.close()

# ------------------------------------------------------------------
# FIG 3: Error vs posisi GT_X — cek apakah collapse terkait zona X
# ------------------------------------------------------------------
fig3, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=True)
fig3.suptitle('Error AMCL vs Posisi GT_X\n(Cek: apakah collapse terjadi di zona tertentu?)', fontsize=11)

for ax, gt_x, err, label, color in [
    (axes[0], gt4x, err4, 'Run 4', 'red'),
    (axes[1], gt5x, err5, 'Run 5', 'green'),
]:
    valid = ~np.isnan(err) & ~np.isnan(gt_x)
    ax.scatter(gt_x[valid], err[valid], c=color, s=4, alpha=0.4, label=label)
    ax.axhline(1.5, color='orange', lw=1, ls='--')
    ax.axvline(-GOAL_ZONE_X, color='purple', lw=1, ls=':', label=f'goal zone x=±{GOAL_ZONE_X}')
    ax.axvline( GOAL_ZONE_X, color='purple', lw=1, ls=':')
    ax.set_ylabel('Error posisi (m)'); ax.set_title(label, fontsize=9)
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

axes[1].set_xlabel('GT_X (m)')
plt.tight_layout()
plt.savefig('diagnosa_run4vs5_error_vs_x.png', dpi=120)
print("Saved: diagnosa_run4vs5_error_vs_x.png")
plt.close()

# ------------------------------------------------------------------
# FIG 4: AMCL_x vs GT_x vs waktu — kapan dan ke mana AMCL kehilangan track?
# ------------------------------------------------------------------
fig4, axes = plt.subplots(2, 1, figsize=(12, 7), sharex=False)
fig4.suptitle('Pelacakan X-Koordinat: AMCL_x vs GT_x vs Waktu\n'
              '(Kapan AMCL berhenti mengikuti GT dan ke mana ia pergi?)', fontsize=11)

for ax, t, gt_x, gt_y, amcl_x, err, label, clr in [
    (axes[0], t4, gt4x, gt4y, amcl4x, err4, 'Run 4 (COLLAPSE)', 'red'),
    (axes[1], t5, gt5x, gt5y, amcl5x, err5, 'Run 5 (BAIK)',     'green'),
]:
    ax.plot(t, gt_x,   'b-',   lw=2,   label='GT_x (nyata)',     alpha=0.85)
    ax.plot(t, amcl_x, lw=1.5, ls='--', color=clr, label='AMCL_x (estimasi)', alpha=0.9)
    ax.axhline(-1.3,   color='purple', lw=1,   ls=':', label='false min x≈-1.3m', alpha=0.7)
    ax.axhline(-0.363, color='gray',   lw=0.8, ls=':', label='spawn x=-0.363m',   alpha=0.5)

    idx_c, tc = find_collapse_point(list(t), list(err), threshold=1.5)
    if idx_c is not None:
        ax.axvline(tc, color='orange', lw=1.5, ls='--', alpha=0.8,
                   label=f'err>1.5m @ t={tc:.0f}s')
        ax.plot(tc, gt_x[idx_c],   'ko', ms=9, zorder=10)
        ax.plot(tc, amcl_x[idx_c], 'o', color=clr, ms=9, zorder=10)

    ax.set_title(label, fontsize=10)
    ax.set_xlabel('t (s)'); ax.set_ylabel('X koordinat (m)')
    ax.set_ylim(-2.0, 1.0)
    ax.legend(fontsize=8); ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('diagnosa_run4vs5_amcl_x_time.png', dpi=120)
print("Saved: diagnosa_run4vs5_amcl_x_time.png")
plt.close()

# ------------------------------------------------------------------
# ANALISIS DIVERGENSI AWAL — kapan & di mana partikel salah mulai menang?
# ------------------------------------------------------------------
EARLY_DIV_THR = 0.3   # AMCL_x menyimpang > 0.3m dari GT_x = divergensi awal terdeteksi
print("\n" + "="*60)
print("ANALISIS DIVERGENSI AWAL — RUN 4 (kapan partikel salah menang?)")
print("="*60)

valid4 = ~np.isnan(amcl4x) & ~np.isnan(gt4x)
diff_x4 = amcl4x - gt4x
early_div_idx = None
for i in np.where(valid4)[0]:
    if abs(diff_x4[i]) > EARLY_DIV_THR:
        early_div_idx = i
        break

if early_div_idx is not None:
    ti = float(t4[early_div_idx])
    print(f"  Divergensi awal (|AMCL_x - GT_x| > {EARLY_DIV_THR}m) : t={ti:.1f}s")
    print(f"  GT_x saat divergensi awal  : {gt4x[early_div_idx]:.3f}m")
    print(f"  GT_y saat divergensi awal  : {gt4y[early_div_idx]:.3f}m")
    print(f"  AMCL_x saat divergensi awal: {amcl4x[early_div_idx]:.3f}m  "
          f"(delta={diff_x4[early_div_idx]:+.3f}m, drift ke {'kiri' if diff_x4[early_div_idx] < 0 else 'kanan'})")

    # Arah gerak robot: vektor dari n sample ke depan
    n_fwd = 10
    fwd = min(early_div_idx + n_fwd, len(gt4x) - 1)
    dx = gt4x[fwd] - gt4x[early_div_idx]
    dy = gt4y[fwd] - gt4y[early_div_idx]
    if abs(dx) > 1e-4 or abs(dy) > 1e-4:
        heading_deg = math.degrees(math.atan2(dy, dx))
        dt_seg = float(t4[fwd] - t4[early_div_idx]) + 1e-9
        speed_approx = math.sqrt(dx**2 + dy**2) / dt_seg
        print(f"  Arah gerak robot saat itu  : heading≈{heading_deg:.0f}° (0°=kanan, 90°=atas)")
        print(f"  Kecepatan GT (approx)      : {speed_approx:.3f} m/s")

    # Nilai AMCL_x di akhir run (tujuan drift)
    valid_amcl4 = amcl4x[~np.isnan(amcl4x)]
    if len(valid_amcl4) > 0:
        final_amcl_x = float(valid_amcl4[-1])
        label_mode = 'FALSE MINIMUM' if final_amcl_x < -0.8 else 'dekat GT'
        print(f"  AMCL_x akhir run           : {final_amcl_x:.3f}m  ({label_mode})")

    dist_to_false = abs(gt4x[early_div_idx] - (-1.3))
    print(f"  Jarak GT→false min x=-1.3m : {dist_to_false:.3f}m")
    print()
    print("  INTERPRETASI: Ini MOTION-INDUCED — partikel salah menang saat robot bergerak")
    print("  di zona di mana scan dari x_GT ≈ scan dari x_false_min (simetri lapangan).")
else:
    print("  Tidak ditemukan divergensi awal (|AMCL_x - GT_x| selalu ≤ 0.3m dalam data ini)")

# ------------------------------------------------------------------
# RINGKASAN TEKS
# ------------------------------------------------------------------
print("\n" + "="*60)
print("RINGKASAN DIAGNOSTIK")
print("="*60)
for label, t, err, gt_x, gt_y in [
    ('Run 4', t4, err4, gt4x, gt4y),
    ('Run 5', t5, err5, gt5x, gt5y),
]:
    valid_err = err[~np.isnan(err)]
    rmse = math.sqrt(np.mean(valid_err**2)) if len(valid_err) > 0 else float('nan')

    # Deteksi apakah collapse diskret atau gradual
    # Hitung rata-rata error 30s pertama vs 30s terakhir
    mask_early = t < 30
    mask_late  = t > (t[-1] - 30)
    early_mean = np.nanmean(err[mask_early]) if mask_early.sum() > 0 else float('nan')
    late_mean  = np.nanmean(err[mask_late])  if mask_late.sum()  > 0 else float('nan')

    # Cari kapan error pertama kali > 1.5m (jika ada)
    idx_c, tc = find_collapse_point(list(t), list(err), threshold=1.5)
    if idx_c is not None:
        gt_xc = gt_x[idx_c]; gt_yc = gt_y[idx_c]
        print(f"\n{label}:")
        print(f"  RMSE = {rmse:.4f}m")
        print(f"  Collapse pertama (err>1.5m): t={tc:.1f}s")
        print(f"  Posisi GT saat collapse: ({gt_xc:.3f}, {gt_yc:.3f})")
        print(f"  Early mean err (0-30s): {early_mean:.3f}m")
        print(f"  Late mean err (last 30s): {late_mean:.3f}m")
        jump = late_mean - early_mean
        print(f"  Delta (late-early): {jump:+.3f}m  -> {'DISKRET/TIBA2' if abs(jump) > 0.5 else 'GRADUAL'}")
    else:
        print(f"\n{label}:")
        print(f"  RMSE = {rmse:.4f}m  (tidak ada collapse >1.5m)")
        print(f"  Early mean err (0-30s): {early_mean:.3f}m")
        print(f"  Late mean err (last 30s): {late_mean:.3f}m")

print("\nPlot tersimpan:")
print("  diagnosa_run4vs5_error_time.png   — error vs waktu")
print("  diagnosa_run4vs5_trajektori.png   — trajektori XY")
print("  diagnosa_run4vs5_error_vs_x.png   — error vs posisi GT_X")
print("  diagnosa_run4vs5_amcl_x_time.png  — AMCL_x vs GT_x vs waktu (Pendekatan 1)")
