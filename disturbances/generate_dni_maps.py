"""
Generate high-fidelity, publication-grade DNI heat maps with explicit grid lines,
cell delineations, passive pipe hatching, and contour overlays.
"""

import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from matplotlib.patches import Rectangle, Patch

# 1. Physical Configuration & Scenarios
Nl = 10         # 10 Collector Loops
Ns = 174        # 174 Spatial Segments (1 m each)
D0 = 800.0      # Nominal Solar Irradiance [W/m]

# Passive pipe joints (0-indexed)
passive_ranges = [(36, 42), (78, 96), (132, 138)] # 37:42, 79:96, 133:138 in MATLAB
passive_idx = []
for start, end in passive_ranges:
    passive_idx.extend(range(start, end))

# Base optical efficiency map
eff = np.ones((Nl, Ns))
eff[[0, 8], 0:36] = 0.68       # Soiled inlet (Loops 1 & 9)
eff[9, 42:132] = 0.42          # Degraded middle (Loop 10)
eff[2:5, 96:132] = 0.20        # Defocused group 3 (Loops 3, 4, 5)
eff[2, 138:174] = 0.65         # Loop 3 end
eff[7, 138:174] = 0.82         # Loop 8 end
eff[:, passive_idx] = 0.0      # Passive crossover joints

# Cloud dynamic parameters
cloud = {
    't0': 63.0,     # Cloud start [min]
    't1': 76.0,     # Cloud end [min]
    'seg0': 10.0,   # Initial segment
    'speed': 11.0,  # 11 m/min propagation
    'len': 60.0,    # 60 m width
    'loops': list(range(2, 9)), # Loops 3 to 9 (0-indexed: 2 to 8)
    'depth': 0.80,  # 80% shadow reduction
    'edge': 4.0     # Sigmoid softness
}

def get_dni(t_sec):
    D = D0 * eff.copy()
    tm = t_sec / 60.0
    if cloud['t0'] <= tm <= cloud['t1']:
        front = cloud['seg0'] + (tm - cloud['t0']) * cloud['speed']
        seg = np.arange(1, Ns + 1)
        sh = (1.0 / (1.0 + np.exp(-(seg - (front - cloud['len'])) / cloud['edge']))) * \
             (1.0 - 1.0 / (1.0 + np.exp(-(seg - front) / cloud['edge'])))
        for lp in cloud['loops']:
            D[lp, :] *= (1.0 - cloud['depth'] * sh)
    return D

out_dir = os.path.join(os.path.dirname(__file__), '..', 'figures')
os.makedirs(out_dir, exist_ok=True)

# -----------------------------------------------------------------------------
# 1. OPTICAL EFFICIENCY HEATMAP WITH COMPREHENSIVE GRIDS & HATCHING
# -----------------------------------------------------------------------------
fig, ax = plt.subplots(figsize=(14, 6.2), dpi=220)

cmap_eff = plt.cm.YlOrRd.copy()
cmap_eff.set_under('#1a1a2e') # Dark blue for passive pipe joints (0.0)

im = ax.imshow(eff, aspect='auto', origin='lower', cmap=cmap_eff, vmin=0.05, vmax=1.0,
               extent=[0.5, Ns + 0.5, 0.5, Nl + 0.5], interpolation='nearest')

# Explicit Grid Lines
ax.set_xticks(np.arange(0, 181, 10))
ax.set_xticks(np.arange(1, 175), minor=True)
ax.set_yticks(np.arange(1, Nl + 1))
ax.set_yticks(np.arange(0.5, Nl + 1.5), minor=True)

# Draw horizontal lines separating every loop
ax.grid(which='minor', axis='y', color='white', linestyle='-', linewidth=1.2, alpha=0.9)
# Draw major vertical grid lines every 10 meters
ax.grid(which='major', axis='x', color='black', linestyle=':', linewidth=0.6, alpha=0.5)

# Highlight and cross-hatch passive crossover joints
for (s_start, s_end) in [(36.5, 42.5), (78.5, 96.5), (132.5, 138.5)]:
    rect = Rectangle((s_start, 0.5), s_end - s_start, Nl,
                     hatch='///', facecolor='#202040', edgecolor='cyan', linewidth=1.0, alpha=0.85)
    ax.add_patch(rect)

# Delineate collector modules
collector_bounds = [0.5, 36.5, 42.5, 78.5, 96.5, 132.5, 138.5, 174.5]
for cb in collector_bounds:
    ax.axvline(cb, color='black', linestyle='-', linewidth=1.5, alpha=0.8)

# Add top labels for collector modules
module_names = [
    (18.5, 'Collector Module 1\n(Active 36 m)'),
    (39.5, 'Joint 1\n(6 m)'),
    (60.5, 'Collector Module 2\n(Active 36 m)'),
    (87.5, 'Joint 2 (Crossover)\n(18 m)'),
    (114.5, 'Collector Module 3\n(Active 36 m)'),
    (135.5, 'Joint 3\n(6 m)'),
    (156.5, 'Collector Module 4\n(Active 36 m)')
]
for x_pos, name in module_names:
    ax.text(x_pos, 10.75, name, ha='center', va='bottom', fontsize=8, fontweight='bold',
            bbox=dict(boxstyle='square,pad=0.2', facecolor='white', edgecolor='#cccccc', alpha=0.85))

# Annotate specific efficiency values directly inside the cells
ax.text(18.5, 1.0, '$\\eta = 0.68$', ha='center', va='center', fontsize=8.5, fontweight='bold', color='black')
ax.text(18.5, 9.0, '$\\eta = 0.68$', ha='center', va='center', fontsize=8.5, fontweight='bold', color='black')
ax.text(114.5, 4.0, 'Defocused: $\\eta = 0.20$ (Loops 3–5)', ha='center', va='center', fontsize=9, fontweight='bold', color='white',
        bbox=dict(boxstyle='round,pad=0.25', facecolor='#b30000', edgecolor='white', lw=1.2))
ax.text(60.5, 10.0, '$\\eta = 0.42$', ha='center', va='center', fontsize=8.5, fontweight='bold', color='black')
ax.text(114.5, 10.0, '$\\eta = 0.42$', ha='center', va='center', fontsize=8.5, fontweight='bold', color='black')

ax.set_xlim(0.5, 174.5)
ax.set_ylim(0.5, 10.5)
ax.set_xlabel('Spatial Coordinate along Loop $z$ [m] (Total 174 m = 144 m Active + 30 m Passive)', fontsize=11, labelpad=8)
ax.set_ylabel('Collector Loop Number $j$', fontsize=11)
ax.set_title('ACUREX Solar Field: Precision Optical Cleanliness & Defocus Heatmap $\\eta_j(z)$', fontsize=13, fontweight='bold', pad=32)

cbar = fig.colorbar(im, ax=ax, orientation='vertical', pad=0.02, shrink=0.92, extend='min')
cbar.set_label('Optical Efficiency $\\eta_j(z)$ [-]', fontsize=11, fontweight='bold')
cbar.set_ticks([0.1, 0.2, 0.4, 0.6, 0.8, 1.0])

# Add custom legend for passive pipe hatching
legend_elements = [
    Patch(facecolor='#202040', edgecolor='cyan', hatch='///', label='Passive Pipe Joints ($\\eta = 0$)'),
    Patch(facecolor='#fed976', edgecolor='black', label='Clean Active Mirrors ($\\eta = 1.0$)')
]
ax.legend(handles=legend_elements, loc='upper left', bbox_to_anchor=(0.0, -0.16), ncol=2, frameon=True, fontsize=9.5)

plt.tight_layout()
fig1_path = os.path.join(out_dir, 'dni_optical_efficiency_map.png')
plt.savefig(fig1_path)
plt.close()
print(f"Generated enhanced: {fig1_path}")


# -----------------------------------------------------------------------------
# 2. DYNAMIC FIELD DNI SNAPSHOTS WITH HIGH-CONTRAST GRIDS & CONTOURS
# -----------------------------------------------------------------------------
fig, axes = plt.subplots(4, 1, figsize=(14, 11), dpi=220, sharex=True, sharey=True)
snapshot_times = [0, 65, 70, 75]
snapshot_titles = [
    '(a) $t = 0$ min: Baseline Clear-Sky Field with Soiling (Nominal $D_0 = 800$ W/m)',
    '(b) $t = 65$ min: Cloud Shadow Enters Field (Front at segment 32 m, speed = 11 m/min)',
    '(c) $t = 70$ min: Peak Shadow Center (Core coverage across loops 3–9, 80% attenuation)',
    '(d) $t = 75$ min: Cloud Shadow Trailing Edge Reaching Field Exit'
]

cmap_dni = plt.cm.turbo.copy()

for idx, (tm, title) in enumerate(zip(snapshot_times, snapshot_titles)):
    ax = axes[idx]
    D_snap = get_dni(tm * 60.0)
    
    im = ax.imshow(D_snap, aspect='auto', origin='lower', cmap=cmap_dni, vmin=0, vmax=800,
                   extent=[0.5, Ns + 0.5, 0.5, Nl + 0.5], interpolation='nearest')
    
    # Grid lines between every loop
    ax.set_yticks(np.arange(1, Nl + 1))
    ax.set_yticks(np.arange(0.5, Nl + 1.5), minor=True)
    ax.grid(which='minor', axis='y', color='white', linestyle='-', linewidth=0.8, alpha=0.6)
    
    # Grid lines every 10 meters along loops
    ax.set_xticks(np.arange(0, 181, 10))
    ax.grid(which='major', axis='x', color='black', linestyle=':', linewidth=0.5, alpha=0.4)
    
    # Delineate passive pipe zones with vertical red/cyan dashed bars
    for (s_start, s_end) in [(36.5, 42.5), (78.5, 96.5), (132.5, 138.5)]:
        ax.axvspan(s_start, s_end, color='black', alpha=0.35)
    
    # Add contour overlay for cloud shadow boundary if cloud is active
    if tm > 0:
        X_grid, Y_grid = np.meshgrid(np.arange(1, Ns + 1), np.arange(1, Nl + 1))
        cs = ax.contour(X_grid, Y_grid, D_snap, levels=[200, 400, 600],
                        colors=['#ffffff', '#ffff00', '#00ffff'], linewidths=1.1, linestyles='--')
        ax.clabel(cs, inline=True, fmt='%d W/m', fontsize=7.5)
    
    ax.set_title(title, fontsize=10.5, fontweight='bold', pad=4, loc='left')
    ax.set_ylabel('Loop $j$', fontsize=10)

axes[-1].set_xlabel('Spatial Position along Collector Loop $z$ [m] (Inlet $\\rightarrow$ Outlet)', fontsize=11)

fig.subplots_adjust(right=0.88, hspace=0.32)
cbar_ax = fig.add_axes([0.90, 0.15, 0.02, 0.7])
cbar = fig.colorbar(im, cax=cbar_ax)
cbar.set_label('Effective Irradiance $D_{j,s}(t)$ [W/m]', fontsize=11, fontweight='bold')
cbar.set_ticks(np.arange(0, 801, 100))

fig2_path = os.path.join(out_dir, 'dni_cloud_snapshots.png')
plt.savefig(fig2_path)
plt.close()
print(f"Generated enhanced: {fig2_path}")


# -----------------------------------------------------------------------------
# 3. SPATIOTEMPORAL SPACE-TIME HEATMAPS (z vs t) WITH DETAILED MESH GRIDS
# -----------------------------------------------------------------------------
time_min = np.linspace(0, 120, 241)
loops_to_plot = [
    (1, 'Loop 2 (Clean Reference Loop: No Cloud, Uniform Solar Input)'),
    (0, 'Loop 1 (Soiled Inlet Module: $\\eta = 0.68$, Outside Cloud Track)'),
    (3, 'Loop 4 (Cloud-Affected & Defocused: Diagonal Cloud Passage at 11 m/min)'),
    (9, 'Loop 10 (Permanently Degraded Middle Section: Collectors 43–132)')
]

fig, axes = plt.subplots(2, 2, figsize=(15, 10.5), dpi=220, sharex=True, sharey=True)
axes = axes.flatten()

for idx, (lp_idx, title) in enumerate(loops_to_plot):
    ax = axes[idx]
    D_matrix = np.zeros((Ns, len(time_min)))
    for t_idx, tm in enumerate(time_min):
        D_step = get_dni(tm * 60.0)
        D_matrix[:, t_idx] = D_step[lp_idx, :]
    
    im = ax.imshow(D_matrix, aspect='auto', origin='lower', cmap=plt.cm.turbo, vmin=0, vmax=800,
                   extent=[time_min[0], time_min[-1], 0.5, Ns + 0.5], interpolation='nearest')
    
    # Overlay Mesh Grids
    ax.set_xticks(np.arange(0, 121, 10))
    ax.set_xticks(np.arange(0, 121, 2), minor=True)
    ax.set_yticks(np.arange(0, 181, 20))
    ax.set_yticks(np.arange(0, 181, 5), minor=True)
    
    ax.grid(which='major', color='black', linestyle=':', linewidth=0.7, alpha=0.6)
    ax.grid(which='minor', color='white', linestyle='-', linewidth=0.2, alpha=0.3)
    
    # Highlight passive crossover joints across the entire time axis
    for (s_start, s_end) in [(36.5, 42.5), (78.5, 96.5), (132.5, 138.5)]:
        ax.axhspan(s_start, s_end, color='#111122', alpha=0.55)
        ax.axhline(s_start, color='cyan', linestyle=':', lw=0.6)
        ax.axhline(s_end, color='cyan', linestyle=':', lw=0.6)
    
    # Highlight cloud tracking in shadowed loops
    if lp_idx in cloud['loops']:
        ax.axvline(cloud['t0'], color='white', linestyle='--', lw=1.2)
        ax.axvline(cloud['t1'], color='white', linestyle='--', lw=1.2)
        
        # Plot cloud trajectory front and rear lines
        t_c = np.linspace(cloud['t0'], cloud['t1'], 100)
        z_front = cloud['seg0'] + (t_c - cloud['t0']) * cloud['speed']
        z_rear  = z_front - cloud['len']
        
        ax.plot(t_c, np.clip(z_front, 0, Ns), color='white', lw=1.8, linestyle='-', label='Cloud Front ($v = 11$ m/min)')
        ax.plot(t_c, np.clip(z_rear, 0, Ns), color='magenta', lw=1.8, linestyle='--', label='Cloud Rear')
        
        # Add text label
        ax.text(69.5, 150, 'Cloud Shadow\nWindow (63–76 min)', color='white', fontweight='bold',
                fontsize=8.5, ha='center', bbox=dict(boxstyle='round,pad=0.2', facecolor='black', alpha=0.65))
        ax.legend(loc='lower left', fontsize=8.5, framealpha=0.85)

    ax.set_title(title, fontsize=10, fontweight='bold', pad=6)
    ax.set_ylabel('Segment Along Loop $z$ [m]', fontsize=10)
    if idx >= 2:
        ax.set_xlabel('Simulation Time $t$ [min]', fontsize=10)

fig.subplots_adjust(right=0.88, hspace=0.25, wspace=0.15)
cbar_ax = fig.add_axes([0.90, 0.15, 0.02, 0.7])
cbar = fig.colorbar(im, cax=cbar_ax)
cbar.set_label('Effective Irradiance $D_{j}(z, t)$ [W/m]', fontsize=11, fontweight='bold')
cbar.set_ticks(np.arange(0, 801, 100))

fig3_path = os.path.join(out_dir, 'dni_spatiotemporal_loop_profiles.png')
plt.savefig(fig3_path)
plt.close()
print(f"Generated enhanced: {fig3_path}")


# -----------------------------------------------------------------------------
# 4. COARSENED MPC CONTROL GRID HEATMAP (29 BLOCKS x 12 HORIZON STEPS)
# -----------------------------------------------------------------------------
fig, axes = plt.subplots(1, 3, figsize=(16, 5.0), dpi=220, sharey=True)
t_preview_min = 65.0
preview_steps = [0, 5, 11] # n=1 (t=65), n=6 (t=70), n=12 (t=76)
preview_labels = [
    '$n = 1$ ($t = 65$ min: Cloud Entry)',
    '$n = 6$ ($t = 70$ min: Peak Shadow)',
    '$n = 12$ ($t = 76$ min: Cloud Exit)'
]

Dp_all = np.zeros((Nl, 29, 12))
for n in range(12):
    D_n = get_dni((t_preview_min + n) * 60.0)
    Dp_all[:, :, n] = np.mean(D_n.reshape(Nl, 29, 6), axis=2)

for idx, (p_step, p_lbl) in enumerate(zip(preview_steps, preview_labels)):
    ax = axes[idx]
    M_grid = Dp_all[:, :, p_step] # [10 x 29]
    
    # Use pcolormesh with explicit edge grid lines to display discrete control cells
    X = np.arange(0.5, 30.5)
    Y = np.arange(0.5, 11.5)
    mesh = ax.pcolormesh(X, Y, M_grid, cmap=plt.cm.turbo, vmin=0, vmax=800,
                         edgecolors='white', linewidth=0.6, shading='flat')
    
    ax.set_xticks(np.arange(1, 30, 2))
    ax.set_yticks(np.arange(1, 11))
    ax.set_title(f'Horizon Step {p_lbl}', fontsize=10.5, fontweight='bold', pad=6)
    ax.set_xlabel('Coarsened Segment $m$ (6 m Blocks, 1 to 29)', fontsize=10)
    if idx == 0:
        ax.set_ylabel('Collector Loop $j$', fontsize=10)

fig.subplots_adjust(right=0.88, wspace=0.15)
cbar_ax = fig.add_axes([0.90, 0.15, 0.015, 0.7])
cbar = fig.colorbar(mesh, cax=cbar_ax)
cbar.set_label('Coarsened MPC Solar Input $D^c_{j,m}$ [W/m]', fontsize=10.5, fontweight='bold')
cbar.set_ticks(np.arange(0, 801, 100))
plt.suptitle('Discretized Control Grid Heatmap: Coarsened Preview Matrix $D^c_{j,m}$ (Used by MPC Solvers)',
             fontsize=12, fontweight='bold', y=0.98)

fig5_path = os.path.join(out_dir, 'dni_mpc_preview.png')
plt.savefig(fig5_path)
plt.close()
print(f"Generated enhanced: {fig5_path}")

print("All enhanced grid heatmaps successfully generated!")
