#%%
# ==========================
# PYTHON IMPORTS
# ==========================
import os
import sys
import xml.etree.ElementTree as ET
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from collections import defaultdict

#%%
# ==========================
# CONFIGURATION
# ==========================
# CONFIGURE WHICH SITUATION TO PROCESS
SITUATION = "sit0"  # Options: "sit0", "sit1", "sit2"

# Get the directory where this script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

# Define paths based on situation
if SITUATION == "sit0":
    fcd_file = os.path.join(script_dir, '..', '..', 'simulation_output', 'scenario_0_2025', 'output_fcd_sit0.xml')
    situation_name = "Situation 0 (No Control)"
elif SITUATION == "sit1":
    fcd_file = os.path.join(script_dir, '..', '..', 'simulation_output', 'scenario_0_2025', 'output_fcd_sit1.xml')
    situation_name = "Situation 1 (ALINEA)"
elif SITUATION == "sit2":
    fcd_file = os.path.join(script_dir, '..', '..', 'simulation_output', 'scenario_0_2025', 'output_fcd_sit2.xml')
    situation_name = "Situation 2 (HERO)"
else:
    print(f"ERROR: Unknown situation '{SITUATION}'")
    sys.exit(1)

output_dir = os.path.join(script_dir, '..', '..', 'simulation_output', 'scenario_0_2025', f'plots_{SITUATION}')
os.makedirs(output_dir, exist_ok=True)

print(f"Processing: {situation_name}")
print(f"Reading FCD data from: {fcd_file}")
print(f"Saving plots to: {output_dir}")

#%%
# ==========================
# PARSE FCD DATA
# ==========================
print("\nParsing FCD XML file...")

# Data storage
vehicle_data = defaultdict(lambda: {'time': [], 'speed': [], 'x': [], 'y': [], 'lane': []})
time_data = defaultdict(lambda: {'speeds': [], 'count': 0})

# Parse XML incrementally to handle large files
context = ET.iterparse(fcd_file, events=('start', 'end'))
context = iter(context)
event, root = next(context)

timestep_count = 0
for event, elem in context:
    if event == 'end' and elem.tag == 'timestep':
        time = float(elem.get('time'))
        
        for vehicle in elem.findall('vehicle'):
            veh_id = vehicle.get('id')
            speed = float(vehicle.get('speed', 0))
            x = float(vehicle.get('x', 0))
            y = float(vehicle.get('y', 0))
            lane = vehicle.get('lane', '')
            
            # Store vehicle trajectory data
            vehicle_data[veh_id]['time'].append(time)
            vehicle_data[veh_id]['speed'].append(speed * 3.6)  # Convert m/s to km/h
            vehicle_data[veh_id]['x'].append(x)
            vehicle_data[veh_id]['y'].append(y)
            vehicle_data[veh_id]['lane'].append(lane)
            
            # Store aggregate time-based data
            time_data[time]['speeds'].append(speed * 3.6)
            time_data[time]['count'] += 1
        
        timestep_count += 1
        if timestep_count % 100 == 0:
            print(f"  Processed {timestep_count} timesteps...")
        
        # Clear element to free memory
        elem.clear()
        root.clear()

print(f"Parsing complete. Found {len(vehicle_data)} vehicles over {timestep_count} timesteps.")

#%%
# ==========================
# COMPUTE AGGREGATE STATISTICS
# ==========================
print("\nComputing aggregate statistics...")

# Time series data
times = sorted(time_data.keys())
avg_speeds = []
vehicle_counts = []
speed_std = []

for t in times:
    speeds = time_data[t]['speeds']
    if speeds:
        avg_speeds.append(np.mean(speeds))
        speed_std.append(np.std(speeds))
    else:
        avg_speeds.append(np.nan)
        speed_std.append(np.nan)
    vehicle_counts.append(time_data[t]['count'])

times = np.array(times)
avg_speeds = np.array(avg_speeds)
speed_std = np.array(speed_std)
vehicle_counts = np.array(vehicle_counts)

#%%
# ==========================
# PLOT 1: NETWORK-WIDE SPEED OVER TIME
# ==========================
print("\nGenerating Plot 1: Network-wide speed over time...")

fig, ax = plt.subplots(figsize=(14, 6))

ax.plot(times, avg_speeds, label='Average Speed', color='blue', linewidth=2)
ax.fill_between(times, 
                 avg_speeds - speed_std, 
                 avg_speeds + speed_std, 
                 alpha=0.3, color='blue', label='±1 Std Dev')

ax.axhline(y=80, color='green', linestyle='--', linewidth=1, alpha=0.7, label='80 km/h (free flow)')
ax.axhline(y=50, color='orange', linestyle='--', linewidth=1, alpha=0.7, label='50 km/h (moderate)')
ax.axhline(y=30, color='red', linestyle='--', linewidth=1, alpha=0.7, label='30 km/h (congestion)')

ax.set_xlabel('Time (seconds)', fontsize=12)
ax.set_ylabel('Speed (km/h)', fontsize=12)
ax.set_title(f'Network-Wide Average Speed Over Time - {situation_name}', fontsize=14, fontweight='bold')
ax.legend(loc='best')
ax.grid(True, alpha=0.3)
ax.set_xlim([0, max(times)])
ax.set_ylim([0, 120])

plt.tight_layout()
plt.savefig(os.path.join(output_dir, '01_network_speed_timeseries.png'), dpi=300, bbox_inches='tight')
plt.close()
print(f"  Saved: 01_network_speed_timeseries.png")

#%%
# ==========================
# PLOT 2: VEHICLE COUNT OVER TIME
# ==========================
print("\nGenerating Plot 2: Vehicle count over time...")

fig, ax = plt.subplots(figsize=(14, 6))

ax.plot(times, vehicle_counts, color='purple', linewidth=2)
ax.fill_between(times, vehicle_counts, alpha=0.3, color='purple')

ax.set_xlabel('Time (seconds)', fontsize=12)
ax.set_ylabel('Number of Vehicles', fontsize=12)
ax.set_title(f'Number of Vehicles in Network Over Time - {situation_name}', fontsize=14, fontweight='bold')
ax.grid(True, alpha=0.3)
ax.set_xlim([0, max(times)])

plt.tight_layout()
plt.savefig(os.path.join(output_dir, '02_vehicle_count_timeseries.png'), dpi=300, bbox_inches='tight')
plt.close()
print(f"  Saved: 02_vehicle_count_timeseries.png")

#%%
# ==========================
# PLOT 3: SPEED DISTRIBUTION (HISTOGRAM)
# ==========================
print("\nGenerating Plot 3: Speed distribution histogram...")

all_speeds = []
for veh_id in vehicle_data:
    all_speeds.extend(vehicle_data[veh_id]['speed'])

fig, ax = plt.subplots(figsize=(12, 6))

ax.hist(all_speeds, bins=50, color='steelblue', edgecolor='black', alpha=0.7)
ax.axvline(x=np.mean(all_speeds), color='red', linestyle='--', linewidth=2, label=f'Mean: {np.mean(all_speeds):.1f} km/h')
ax.axvline(x=np.median(all_speeds), color='green', linestyle='--', linewidth=2, label=f'Median: {np.median(all_speeds):.1f} km/h')

ax.set_xlabel('Speed (km/h)', fontsize=12)
ax.set_ylabel('Frequency', fontsize=12)
ax.set_title(f'Speed Distribution - {situation_name}', fontsize=14, fontweight='bold')
ax.legend(loc='best')
ax.grid(True, alpha=0.3, axis='y')

plt.tight_layout()
plt.savefig(os.path.join(output_dir, '03_speed_distribution.png'), dpi=300, bbox_inches='tight')
plt.close()
print(f"  Saved: 03_speed_distribution.png")

#%%
# ==========================
# PLOT 4: SPEED HEATMAP (TIME vs SPEED)
# ==========================
print("\nGenerating Plot 4: Speed heatmap...")

# Create 2D histogram
time_bins = np.linspace(0, max(times), 100)
speed_bins = np.linspace(0, 120, 60)

# Collect all time-speed pairs
all_time_speed_pairs = []
for veh_id in vehicle_data:
    for t, s in zip(vehicle_data[veh_id]['time'], vehicle_data[veh_id]['speed']):
        all_time_speed_pairs.append((t, s))

times_flat = [pair[0] for pair in all_time_speed_pairs]
speeds_flat = [pair[1] for pair in all_time_speed_pairs]

fig, ax = plt.subplots(figsize=(14, 6))

h = ax.hist2d(times_flat, speeds_flat, bins=[time_bins, speed_bins], cmap='YlOrRd', cmin=1)
cbar = plt.colorbar(h[3], ax=ax)
cbar.set_label('Vehicle Count', fontsize=12)

ax.set_xlabel('Time (seconds)', fontsize=12)
ax.set_ylabel('Speed (km/h)', fontsize=12)
ax.set_title(f'Speed Distribution Over Time (Heatmap) - {situation_name}', fontsize=14, fontweight='bold')

plt.tight_layout()
plt.savefig(os.path.join(output_dir, '04_speed_heatmap.png'), dpi=300, bbox_inches='tight')
plt.close()
print(f"  Saved: 04_speed_heatmap.png")

#%%
# ==========================
# PLOT 5: INDIVIDUAL VEHICLE TRAJECTORIES (SAMPLE)
# ==========================
print("\nGenerating Plot 5: Sample vehicle trajectories...")

# Select random sample of vehicles
sample_size = min(20, len(vehicle_data))
sample_vehicles = np.random.choice(list(vehicle_data.keys()), sample_size, replace=False)

fig, ax = plt.subplots(figsize=(14, 6))

for veh_id in sample_vehicles:
    times_veh = vehicle_data[veh_id]['time']
    speeds_veh = vehicle_data[veh_id]['speed']
    ax.plot(times_veh, speeds_veh, alpha=0.6, linewidth=1)

ax.set_xlabel('Time (seconds)', fontsize=12)
ax.set_ylabel('Speed (km/h)', fontsize=12)
ax.set_title(f'Individual Vehicle Speed Trajectories (Sample of {sample_size} vehicles) - {situation_name}', 
             fontsize=14, fontweight='bold')
ax.grid(True, alpha=0.3)
ax.set_xlim([0, max(times)])
ax.set_ylim([0, 120])

plt.tight_layout()
plt.savefig(os.path.join(output_dir, '05_vehicle_trajectories_sample.png'), dpi=300, bbox_inches='tight')
plt.close()
print(f"  Saved: 05_vehicle_trajectories_sample.png")

#%%
# ==========================
# PLOT 6: SPATIAL SPEED MAP (X-Y PLOT)
# ==========================
print("\nGenerating Plot 6: Spatial speed map...")

# Collect all position-speed data
x_coords = []
y_coords = []
speeds_spatial = []

for veh_id in vehicle_data:
    x_coords.extend(vehicle_data[veh_id]['x'])
    y_coords.extend(vehicle_data[veh_id]['y'])
    speeds_spatial.extend(vehicle_data[veh_id]['speed'])

fig, ax = plt.subplots(figsize=(16, 8))

scatter = ax.scatter(x_coords, y_coords, c=speeds_spatial, cmap='RdYlGn', 
                     s=1, alpha=0.5, vmin=0, vmax=100)
cbar = plt.colorbar(scatter, ax=ax)
cbar.set_label('Speed (km/h)', fontsize=12)

ax.set_xlabel('X Position (m)', fontsize=12)
ax.set_ylabel('Y Position (m)', fontsize=12)
ax.set_title(f'Spatial Speed Distribution - {situation_name}', fontsize=14, fontweight='bold')
ax.set_aspect('equal', adjustable='box')

plt.tight_layout()
plt.savefig(os.path.join(output_dir, '06_spatial_speed_map.png'), dpi=300, bbox_inches='tight')
plt.close()
print(f"  Saved: 06_spatial_speed_map.png")

#%%
# ==========================
# PLOT 7: CONGESTION ANALYSIS
# ==========================
print("\nGenerating Plot 7: Congestion analysis...")

# Define congestion thresholds
free_flow_threshold = 80  # km/h
moderate_threshold = 50
congestion_threshold = 30

# Calculate percentage of vehicles in each state over time
free_flow_pct = []
moderate_pct = []
congested_pct = []
severe_pct = []

for t in times:
    speeds = time_data[t]['speeds']
    if speeds:
        total = len(speeds)
        free_flow_pct.append(100 * sum(s >= free_flow_threshold for s in speeds) / total)
        moderate_pct.append(100 * sum(moderate_threshold <= s < free_flow_threshold for s in speeds) / total)
        congested_pct.append(100 * sum(congestion_threshold <= s < moderate_threshold for s in speeds) / total)
        severe_pct.append(100 * sum(s < congestion_threshold for s in speeds) / total)
    else:
        free_flow_pct.append(0)
        moderate_pct.append(0)
        congested_pct.append(0)
        severe_pct.append(0)

fig, ax = plt.subplots(figsize=(14, 6))

ax.fill_between(times, 0, severe_pct, color='darkred', alpha=0.7, label='Severe Congestion (<30 km/h)')
ax.fill_between(times, severe_pct, [s+c for s,c in zip(severe_pct, congested_pct)], 
                color='orange', alpha=0.7, label='Congested (30-50 km/h)')
ax.fill_between(times, [s+c for s,c in zip(severe_pct, congested_pct)], 
                [s+c+m for s,c,m in zip(severe_pct, congested_pct, moderate_pct)],
                color='yellow', alpha=0.7, label='Moderate (50-80 km/h)')
ax.fill_between(times, [s+c+m for s,c,m in zip(severe_pct, congested_pct, moderate_pct)],
                100, color='green', alpha=0.7, label='Free Flow (≥80 km/h)')

ax.set_xlabel('Time (seconds)', fontsize=12)
ax.set_ylabel('Percentage of Vehicles (%)', fontsize=12)
ax.set_title(f'Traffic Congestion Levels Over Time - {situation_name}', fontsize=14, fontweight='bold')
ax.legend(loc='best')
ax.grid(True, alpha=0.3, axis='y')
ax.set_xlim([0, max(times)])
ax.set_ylim([0, 100])

plt.tight_layout()
plt.savefig(os.path.join(output_dir, '07_congestion_analysis.png'), dpi=300, bbox_inches='tight')
plt.close()
print(f"  Saved: 07_congestion_analysis.png")

#%%
# ==========================
# SUMMARY STATISTICS
# ==========================
print("\n" + "="*60)
print(f"SUMMARY STATISTICS - {situation_name.upper()}")
print("="*60)

print(f"\nOverall Statistics:")
print(f"  Total vehicles simulated: {len(vehicle_data)}")
print(f"  Simulation duration: {max(times):.0f} seconds ({max(times)/60:.1f} minutes)")
print(f"  Average speed (network-wide): {np.nanmean(avg_speeds):.2f} km/h")
print(f"  Median speed (network-wide): {np.nanmedian(avg_speeds):.2f} km/h")
print(f"  Speed std deviation: {np.nanmean(speed_std):.2f} km/h")
print(f"  Peak vehicle count: {max(vehicle_counts)} vehicles")
print(f"  Average vehicle count: {np.mean(vehicle_counts):.1f} vehicles")

print(f"\nCongestion Metrics:")
avg_free_flow = np.mean(free_flow_pct)
avg_moderate = np.mean(moderate_pct)
avg_congested = np.mean(congested_pct)
avg_severe = np.mean(severe_pct)

print(f"  Free flow (≥80 km/h): {avg_free_flow:.1f}% of time")
print(f"  Moderate (50-80 km/h): {avg_moderate:.1f}% of time")
print(f"  Congested (30-50 km/h): {avg_congested:.1f}% of time")
print(f"  Severe (<30 km/h): {avg_severe:.1f}% of time")

print(f"\nSpeed Statistics (all measurements):")
print(f"  Min speed: {min(all_speeds):.2f} km/h")
print(f"  Max speed: {max(all_speeds):.2f} km/h")
print(f"  Mean speed: {np.mean(all_speeds):.2f} km/h")
print(f"  Median speed: {np.median(all_speeds):.2f} km/h")
print(f"  Std deviation: {np.std(all_speeds):.2f} km/h")

print("\n" + "="*60)
print(f"All plots saved to: {output_dir}")
print("="*60)

# %%
