#!/usr/bin/env python3
"""Plot prediction RMSE vs horizon for all methods."""
import matplotlib.pyplot as plt
import numpy as np
import argparse, csv, os, sys

# Data: read prediction_error.csv from each method
methods = {
    'ours_full': '/data/rosbags/avoidance/pair_crossing/ours_full/run_3/prediction_error.csv',
    'ours_no_clip': '/data/rosbags/avoidance/pair_crossing/ours_no_clip/run_1/prediction_error.csv',
    'ours_no_sam': '/data/rosbags/avoidance/pair_crossing/ours_no_sam/run_2/prediction_error.csv',
    'DODT': '/data/rosbags/avoidance/pair_crossing/dodt/run_2/prediction_error.csv',
    'FAPP': '/data/rosbags/avoidance/pair_crossing/fapp/run_1/prediction_error.csv',
}

colors = {'ours_full': 'blue', 'ours_no_clip': 'orange', 'ours_no_sam': 'green', 'DODT': 'red', 'FAPP': 'purple'}
markers = {'ours_full': 'o', 'ours_no_clip': 's', 'ours_no_sam': '^', 'DODT': 'D', 'FAPP': 'v'}

fig, ax = plt.subplots(figsize=(8, 5))

for name, path in methods.items():
    horizons, rmses = [], []
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            horizons.append(float(row['horizon']))
            rmses.append(float(row['rmse']))
    ax.plot(horizons, rmses, color=colors[name], marker=markers[name],
            label=name, linewidth=2, markersize=8)

ax.set_xlabel('Prediction Horizon (s)', fontsize=12)
ax.set_ylabel('RMSE (m)', fontsize=12)
ax.set_title('Prediction RMSE vs Horizon -- pair_crossing', fontsize=14)
ax.legend(fontsize=10)
ax.grid(True, alpha=0.3)
ax.set_xlim(0, 3.2)

plt.tight_layout()
_ap = argparse.ArgumentParser()
_ap.add_argument('--output', default='figures/fig_prediction_rmse_horizon.pdf')
out = _ap.parse_args().output
os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
plt.savefig(out, dpi=150, bbox_inches='tight')
print(f'Saved to {out}')
