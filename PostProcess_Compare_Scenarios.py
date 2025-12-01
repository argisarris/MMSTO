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
# TIME RANGE FOR ANALYSIS (exclude warm-up period)
TIME_START = 900  # seconds
TIME_END = 4500   # seconds

# RAMP EDGES TO EXCLUDE FROM SPEED CALCULATIONS
# These are the ramp edges - exclude vehicles on these from network speed stats
RAMP_EDGES = [
    'A36_WAED', 'E35_HOR', 'A35_HOR',
    'E34_THA', 'A34_THA', 'E36_WAED', 
    'E36_WAED_ACC', 'E35_HOR_ACC', 'E34_THA_ACC'
]

# Get the directory where this script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

# Define all scenarios
scenarios = {
    'sit0': {
        'name': 'No Control',
        'fcd_file': os.path.join(script_dir, 'simulation_output', 'scenario_0_Base', 'output_fcd_sit0.xml'),
        'detector_path': os.path.join(script_dir, 'simulation_output', 'scenario_0_Base', 'output_detectors'),
        'color': 'red'
    },
    'sit1': {
        'name': 'ALINEA',
        'fcd_file': os.path.join(script_dir, 'simulation_output', 'scenario_1_ALINEA', 'output_fcd_sit1.xml'),
        'detector_path': os.path.join(script_dir, 'simulation_output', 'scenario_1_ALINEA', 'output_detectors'),
        'color': 'blue'
    },
    'sit2': {
        'name': 'ALINEA+HERO',
        'fcd_file': os.path.join(script_dir, 'simulation_output', 'scenario_2_ALINEA+HERO', 'output_fcd_sit2.xml'),
        'detector_path': os.path.join(script_dir, 'simulation_output', 'scenario_2_ALINEA+HERO', 'output_detectors'),
        'color': 'green'
    },
    'sit3': {
        'name': 'ALINEA_long',
        'fcd_file': os.path.join(script_dir, 'simulation_output', 'scenario_3_ALINEA_long', 'output_fcd_sit3.xml'),
        'detector_path': os.path.join(script_dir, 'simulation_output', 'scenario_3_ALINEA_long', 'output_detectors'),
        'color': 'purple'
    }
}

output_dir = os.path.join(script_dir, 'simulation_output', 'comparison_plots')
os.makedirs(output_dir, exist_ok=True)

print("="*60)
print("SCENARIO COMPARISON ANALYSIS")
print("="*60)
print(f"Time range: {TIME_START}-{TIME_END} seconds")
print(f"Output directory: {output_dir}\n")

#%%
# ==========================
# PARSE FCD DATA FOR ALL SCENARIOS
# ==========================
print("Parsing FCD data for all scenarios...")
print(f"  Excluding vehicles on ramp edges: {RAMP_EDGES}")

fcd_data = {}

for sit_id, sit_info in scenarios.items():
    print(f"\n  Processing {sit_info['name']}...")
    
    time_data = defaultdict(lambda: {'speeds': [], 'speeds_mainline': [], 'count': 0, 'count_mainline': 0})
    
    if not os.path.exists(sit_info['fcd_file']):
        print(f"    WARNING: FCD file not found: {sit_info['fcd_file']}")
        continue
    
    context = ET.iterparse(sit_info['fcd_file'], events=('start', 'end'))
    context = iter(context)
    event, root = next(context)
    
    for event, elem in context:
        if event == 'end' and elem.tag == 'timestep':
            time = float(elem.get('time'))
            
            if TIME_START <= time <= TIME_END:
                for vehicle in elem.findall('vehicle'):
                    speed = float(vehicle.get('speed', 0))
                    lane = vehicle.get('lane', '')
                    
                    # Extract edge from lane (format: edgeID_laneIndex)
                    edge = lane.rsplit('_', 1)[0] if '_' in lane else lane
                    
                    speed_kmh = speed * 3.6
                    time_data[time]['speeds'].append(speed_kmh)
                    time_data[time]['count'] += 1
                    
                    # Only add to mainline data if not on a ramp edge
                    if edge not in RAMP_EDGES:
                        time_data[time]['speeds_mainline'].append(speed_kmh)
                        time_data[time]['count_mainline'] += 1
            
            elem.clear()
            root.clear()
    
    # Compute aggregate statistics (using mainline data for speed metrics)
    times = sorted(time_data.keys())
    avg_speeds = []
    vehicle_counts = []
    speed_std = []
    vehicle_counts_mainline = []
    
    for t in times:
        speeds_mainline = time_data[t]['speeds_mainline']
        if speeds_mainline:
            avg_speeds.append(np.mean(speeds_mainline))
            speed_std.append(np.std(speeds_mainline))
        else:
            avg_speeds.append(np.nan)
            speed_std.append(np.nan)
        vehicle_counts.append(time_data[t]['count'])
        vehicle_counts_mainline.append(time_data[t]['count_mainline'])
    
    fcd_data[sit_id] = {
        'times': np.array(times),
        'avg_speeds': np.array(avg_speeds),
        'speed_std': np.array(speed_std),
        'vehicle_counts': np.array(vehicle_counts),
        'vehicle_counts_mainline': np.array(vehicle_counts_mainline),
        'time_data': time_data
    }
    
    print(f"    Found data for {len(times)} timesteps")
    print(f"    Average mainline vehicles per timestep: {np.mean(vehicle_counts_mainline):.1f}")

#%%
# ==========================
# PARSE DETECTOR DATA FOR ALL SCENARIOS
# ==========================
print("\nParsing detector data for all scenarios...")

detector_data = {}

for sit_id, sit_info in scenarios.items():
    print(f"\n  Processing {sit_info['name']} detectors...")
    
    det_path = sit_info['detector_path']
    if not os.path.isdir(det_path):
        print(f"    WARNING: Detector directory not found: {det_path}")
        continue
    
    xml_files = [f for f in os.listdir(det_path) if f.endswith('.xml')]
    
    scenario_detectors = defaultdict(lambda: {
        'time': [], 'speed': [], 'occupancy': [], 'flow': []
    })
    
    for xml_file in xml_files:
        try:
            tree = ET.parse(os.path.join(det_path, xml_file))
            root = tree.getroot()
            
            for interval in root.findall('interval'):
                det_id = interval.get('id')
                time_begin = float(interval.get('begin', 0))
                time_end = float(interval.get('end', 0))
                time_mid = (time_begin + time_end) / 2
                
                if TIME_START <= time_mid <= TIME_END:
                    speed = float(interval.get('speed', -1))
                    occupancy = float(interval.get('occupancy', 0))
                    flow = float(interval.get('flow', 0))
                    
                    scenario_detectors[det_id]['time'].append(time_mid)
                    scenario_detectors[det_id]['speed'].append(speed * 3.6 if speed >= 0 else np.nan)
                    scenario_detectors[det_id]['occupancy'].append(occupancy)
                    scenario_detectors[det_id]['flow'].append(flow)
        except:
            continue
    
    detector_data[sit_id] = scenario_detectors
    print(f"    Found {len(scenario_detectors)} detectors")

#%%
# ==========================
# PLOT 1: NETWORK-WIDE AVERAGE SPEED COMPARISON
# ==========================
print("\nGenerating Plot 1: Network-wide speed comparison...")

fig, ax = plt.subplots(figsize=(16, 6))

for sit_id, data in fcd_data.items():
    sit_info = scenarios[sit_id]
    ax.plot(data['times'], data['avg_speeds'], 
            label=sit_info['name'], color=sit_info['color'], linewidth=2, alpha=0.8)

ax.axhline(y=80, color='green', linestyle='--', linewidth=1, alpha=0.5, label='Free flow (80 km/h)')
ax.axhline(y=50, color='orange', linestyle='--', linewidth=1, alpha=0.5, label='Moderate (50 km/h)')
ax.axhline(y=30, color='red', linestyle='--', linewidth=1, alpha=0.5, label='Congestion (30 km/h)')

ax.set_xlabel('Time (seconds)', fontsize=12)
ax.set_ylabel('Average Speed (km/h)', fontsize=12)
ax.set_title('Network-Wide Average Speed Comparison', fontsize=14, fontweight='bold')
ax.legend(loc='best', fontsize=11)
ax.grid(True, alpha=0.3)
ax.set_xlim([TIME_START, TIME_END])
ax.set_ylim([0, 120])

plt.tight_layout()
plt.savefig(os.path.join(output_dir, '01_speed_comparison.png'), dpi=300, bbox_inches='tight')
plt.close()
print("  Saved: 01_speed_comparison.png")

#%%
# ==========================
# PLOT 2: VEHICLE COUNT COMPARISON
# ==========================
print("\nGenerating Plot 2: Vehicle count comparison...")

fig, ax = plt.subplots(figsize=(16, 6))

for sit_id, data in fcd_data.items():
    sit_info = scenarios[sit_id]
    ax.plot(data['times'], data['vehicle_counts_mainline'], 
            label=f"{sit_info['name']} (Mainline)", color=sit_info['color'], linewidth=2, alpha=0.8)

ax.set_xlabel('Time (seconds)', fontsize=12)
ax.set_ylabel('Number of Vehicles', fontsize=12)
ax.set_title('Mainline Vehicle Count in Network Comparison', fontsize=14, fontweight='bold')
ax.legend(loc='best', fontsize=11)
ax.grid(True, alpha=0.3)
ax.set_xlim([TIME_START, TIME_END])

plt.tight_layout()
plt.savefig(os.path.join(output_dir, '02_vehicle_count_comparison.png'), dpi=300, bbox_inches='tight')
plt.close()
print("  Saved: 02_vehicle_count_comparison.png")

#%%
# ==========================
# PLOT 3: RAMP-SPECIFIC COMPARISONS (THALWIL)
# ==========================
print("\nGenerating Plot 3: Thalwil ramp comparison...")

tha_mainline_detectors = ['SENS_A3_THA_MID0', 'SENS_A3_THA_MID1']
tha_ramp_detector = 'SENS_E_THA'

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 10))

# Mainline speed comparison
for sit_id, det_data in detector_data.items():
    sit_info = scenarios[sit_id]
    if all(det in det_data for det in tha_mainline_detectors):
        times = det_data[tha_mainline_detectors[0]]['time']
        speeds = []
        for i in range(len(times)):
            speed_vals = [det_data[det]['speed'][i] for det in tha_mainline_detectors]
            speeds.append(np.nanmean(speed_vals))
        ax1.plot(times, speeds, label=f'{sit_info["name"]} - Mainline', 
                color=sit_info['color'], linewidth=2, alpha=0.8)

ax1.set_xlabel('Time (seconds)', fontsize=12)
ax1.set_ylabel('Speed (km/h)', fontsize=12)
ax1.set_title('Thalwil Ramp - Mainline Speed Comparison', fontsize=13, fontweight='bold')
ax1.legend(loc='best', fontsize=10)
ax1.grid(True, alpha=0.3)
ax1.set_xlim([TIME_START, TIME_END])
ax1.set_ylim([0, 120])

# Ramp flow comparison
for sit_id, det_data in detector_data.items():
    sit_info = scenarios[sit_id]
    if tha_ramp_detector in det_data:
        times = det_data[tha_ramp_detector]['time']
        flows = det_data[tha_ramp_detector]['flow']
        ax2.plot(times, flows, label=f'{sit_info["name"]} - Ramp Flow', 
                color=sit_info['color'], linewidth=2, alpha=0.8)

ax2.set_xlabel('Time (seconds)', fontsize=12)
ax2.set_ylabel('Flow (veh/h)', fontsize=12)
ax2.set_title('Thalwil Ramp - Flow Comparison', fontsize=13, fontweight='bold')
ax2.legend(loc='best', fontsize=10)
ax2.grid(True, alpha=0.3)
ax2.set_xlim([TIME_START, TIME_END])

plt.tight_layout()
plt.savefig(os.path.join(output_dir, '03_THA_comparison.png'), dpi=300, bbox_inches='tight')
plt.close()
print("  Saved: 03_THA_comparison.png")

#%%
# ==========================
# PLOT 4: RAMP-SPECIFIC COMPARISONS (HORGEN)
# ==========================
print("\nGenerating Plot 4: Horgen ramp comparison...")

hor_mainline_detectors = ['SENS_A3_HOR_MID0', 'SENS_A3_HOR_MID1']
hor_ramp_detector = 'SENS_E_HOR'

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 10))

for sit_id, det_data in detector_data.items():
    sit_info = scenarios[sit_id]
    if all(det in det_data for det in hor_mainline_detectors):
        times = det_data[hor_mainline_detectors[0]]['time']
        speeds = []
        for i in range(len(times)):
            speed_vals = [det_data[det]['speed'][i] for det in hor_mainline_detectors]
            speeds.append(np.nanmean(speed_vals))
        ax1.plot(times, speeds, label=f'{sit_info["name"]} - Mainline', 
                color=sit_info['color'], linewidth=2, alpha=0.8)

ax1.set_xlabel('Time (seconds)', fontsize=12)
ax1.set_ylabel('Speed (km/h)', fontsize=12)
ax1.set_title('Horgen Ramp - Mainline Speed Comparison', fontsize=13, fontweight='bold')
ax1.legend(loc='best', fontsize=10)
ax1.grid(True, alpha=0.3)
ax1.set_xlim([TIME_START, TIME_END])
ax1.set_ylim([0, 120])

for sit_id, det_data in detector_data.items():
    sit_info = scenarios[sit_id]
    if hor_ramp_detector in det_data:
        times = det_data[hor_ramp_detector]['time']
        flows = det_data[hor_ramp_detector]['flow']
        ax2.plot(times, flows, label=f'{sit_info["name"]} - Ramp Flow', 
                color=sit_info['color'], linewidth=2, alpha=0.8)

ax2.set_xlabel('Time (seconds)', fontsize=12)
ax2.set_ylabel('Flow (veh/h)', fontsize=12)
ax2.set_title('Horgen Ramp - Flow Comparison', fontsize=13, fontweight='bold')
ax2.legend(loc='best', fontsize=10)
ax2.grid(True, alpha=0.3)
ax2.set_xlim([TIME_START, TIME_END])

plt.tight_layout()
plt.savefig(os.path.join(output_dir, '04_HOR_comparison.png'), dpi=300, bbox_inches='tight')
plt.close()
print("  Saved: 04_HOR_comparison.png")

#%%
# ==========================
# PLOT 5: RAMP-SPECIFIC COMPARISONS (WÄDENSWIL)
# ==========================
print("\nGenerating Plot 5: Wädenswil ramp comparison...")

wae_mainline_detectors = ['SENS_A3_WAE_MID0', 'SENS_A3_WAE_MID1']
wae_ramp_detector = 'SENS_E_WAE'

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 10))

for sit_id, det_data in detector_data.items():
    sit_info = scenarios[sit_id]
    if all(det in det_data for det in wae_mainline_detectors):
        times = det_data[wae_mainline_detectors[0]]['time']
        speeds = []
        for i in range(len(times)):
            speed_vals = [det_data[det]['speed'][i] for det in wae_mainline_detectors]
            speeds.append(np.nanmean(speed_vals))
        ax1.plot(times, speeds, label=f'{sit_info["name"]} - Mainline', 
                color=sit_info['color'], linewidth=2, alpha=0.8)

ax1.set_xlabel('Time (seconds)', fontsize=12)
ax1.set_ylabel('Speed (km/h)', fontsize=12)
ax1.set_title('Wädenswil Ramp - Mainline Speed Comparison', fontsize=13, fontweight='bold')
ax1.legend(loc='best', fontsize=10)
ax1.grid(True, alpha=0.3)
ax1.set_xlim([TIME_START, TIME_END])
ax1.set_ylim([0, 120])

for sit_id, det_data in detector_data.items():
    sit_info = scenarios[sit_id]
    if wae_ramp_detector in det_data:
        times = det_data[wae_ramp_detector]['time']
        flows = det_data[wae_ramp_detector]['flow']
        ax2.plot(times, flows, label=f'{sit_info["name"]} - Ramp Flow', 
                color=sit_info['color'], linewidth=2, alpha=0.8)

ax2.set_xlabel('Time (seconds)', fontsize=12)
ax2.set_ylabel('Flow (veh/h)', fontsize=12)
ax2.set_title('Wädenswil Ramp - Flow Comparison', fontsize=13, fontweight='bold')
ax2.legend(loc='best', fontsize=10)
ax2.grid(True, alpha=0.3)
ax2.set_xlim([TIME_START, TIME_END])

plt.tight_layout()
plt.savefig(os.path.join(output_dir, '05_WAE_comparison.png'), dpi=300, bbox_inches='tight')
plt.close()
print("  Saved: 05_WAE_comparison.png")

#%%
# ==========================
# PLOT 6: SUMMARY BAR CHARTS
# ==========================
print("\nGenerating Plot 6: Summary statistics comparison...")

fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))

scenario_names = [scenarios[sit]['name'] for sit in ['sit0', 'sit1', 'sit2', 'sit3']]
colors = [scenarios[sit]['color'] for sit in ['sit0', 'sit1', 'sit2', 'sit3']]

# Average speed (mainline only)
avg_speeds_summary = [np.nanmean(fcd_data[sit]['avg_speeds']) for sit in ['sit0', 'sit1', 'sit2', 'sit3'] if sit in fcd_data]
ax1.bar(scenario_names[:len(avg_speeds_summary)], avg_speeds_summary, color=colors[:len(avg_speeds_summary)], alpha=0.7)
ax1.set_ylabel('Average Speed (km/h)', fontsize=11)
ax1.set_title('Network Average Speed (Mainline Only)', fontsize=12, fontweight='bold')
ax1.grid(True, alpha=0.3, axis='y')

# Average vehicle count (mainline)
avg_counts = [np.mean(fcd_data[sit]['vehicle_counts_mainline']) for sit in ['sit0', 'sit1', 'sit2', 'sit3'] if sit in fcd_data]
ax2.bar(scenario_names[:len(avg_counts)], avg_counts, color=colors[:len(avg_counts)], alpha=0.7)
ax2.set_ylabel('Average Vehicle Count', fontsize=11)
ax2.set_title('Average Vehicles in Network (Mainline)', fontsize=12, fontweight='bold')
ax2.grid(True, alpha=0.3, axis='y')

# Speed standard deviation (mainline only)
speed_std_summary = [np.nanmean(fcd_data[sit]['speed_std']) for sit in ['sit0', 'sit1', 'sit2', 'sit3'] if sit in fcd_data]
ax3.bar(scenario_names[:len(speed_std_summary)], speed_std_summary, color=colors[:len(speed_std_summary)], alpha=0.7)
ax3.set_ylabel('Speed Std Dev (km/h)', fontsize=11)
ax3.set_title('Speed Variability (Mainline Only)', fontsize=12, fontweight='bold')
ax3.grid(True, alpha=0.3, axis='y')

# Congestion percentage (speed < 50 km/h, mainline only)
congestion_pct = []
for sit in ['sit0', 'sit1', 'sit2', 'sit3']:
    if sit in fcd_data:
        all_speeds_mainline = []
        for t in fcd_data[sit]['time_data'].values():
            all_speeds_mainline.extend(t['speeds_mainline'])
        if all_speeds_mainline:
            pct = 100 * sum(s < 50 for s in all_speeds_mainline) / len(all_speeds_mainline)
            congestion_pct.append(pct)

ax4.bar(scenario_names[:len(congestion_pct)], congestion_pct, color=colors[:len(congestion_pct)], alpha=0.7)
ax4.set_ylabel('Congestion (%)', fontsize=11)
ax4.set_title('Percentage of Speeds < 50 km/h (Mainline Only)', fontsize=12, fontweight='bold')
ax4.grid(True, alpha=0.3, axis='y')

plt.suptitle('Scenario Comparison - Summary Statistics (Mainline Only)', fontsize=14, fontweight='bold', y=0.995)
plt.tight_layout()
plt.savefig(os.path.join(output_dir, '06_summary_comparison.png'), dpi=300, bbox_inches='tight')
plt.close()
print("  Saved: 06_summary_comparison.png")

#%%
# ==========================
# PRINT SUMMARY STATISTICS
# ==========================
print("\n" + "="*60)
print("COMPARATIVE SUMMARY STATISTICS (MAINLINE ONLY)")
print("="*60)

for sit_id in ['sit0', 'sit1', 'sit2', 'sit3']:
    if sit_id not in fcd_data:
        continue
    
    sit_info = scenarios[sit_id]
    data = fcd_data[sit_id]
    
    print(f"\n{sit_info['name']}:")
    print(f"  Average speed (mainline): {np.nanmean(data['avg_speeds']):.2f} km/h")
    print(f"  Average vehicle count (all): {np.mean(data['vehicle_counts']):.1f} vehicles")
    print(f"  Average vehicle count (mainline): {np.mean(data['vehicle_counts_mainline']):.1f} vehicles")
    print(f"  Speed std deviation (mainline): {np.nanmean(data['speed_std']):.2f} km/h")
    
    all_speeds_mainline = []
    for t in data['time_data'].values():
        all_speeds_mainline.extend(t['speeds_mainline'])
    
    if all_speeds_mainline:
        print(f"  % Free flow (≥80 km/h, mainline): {100 * sum(s >= 80 for s in all_speeds_mainline) / len(all_speeds_mainline):.1f}%")
        print(f"  % Congested (<50 km/h, mainline): {100 * sum(s < 50 for s in all_speeds_mainline) / len(all_speeds_mainline):.1f}%")

print("\n" + "="*60)
print(f"All comparison plots saved to: {output_dir}")
print("="*60)

# %%
