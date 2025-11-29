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

# TIME RANGE FOR ANALYSIS (exclude warm-up period)
TIME_START = 900  # seconds
TIME_END = 4500   # seconds

# Get the directory where this script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

# Define paths based on situation
if SITUATION == "sit0":
    detector_path = os.path.join(script_dir, 'simulation_output', 'scenario_0_Base', 'output_detectors')
    output_dir = os.path.join(script_dir, 'simulation_output', 'scenario_0_Base', f'plots_{SITUATION}')
    situation_name = "Situation 0 (No Control)"
elif SITUATION == "sit1":
    detector_path = os.path.join(script_dir, 'simulation_output', 'scenario_1_ALINEA', 'output_detectors')
    output_dir = os.path.join(script_dir, 'simulation_output', 'scenario_1_ALINEA', f'plots_{SITUATION}')
    situation_name = "Situation 1 (ALINEA)"
elif SITUATION == "sit2":
    detector_path = os.path.join(script_dir, 'simulation_output', 'scenario_2_ALINEA+HERO', 'output_detectors')
    output_dir = os.path.join(script_dir, 'simulation_output', 'scenario_2_ALINEA+HERO', f'plots_{SITUATION}')
    situation_name = "Situation 2 (HERO)"
else:
    print(f"ERROR: Unknown situation '{SITUATION}'")
    sys.exit(1)

os.makedirs(output_dir, exist_ok=True)

print(f"Processing: {situation_name}")
print(f"Time range: {TIME_START}-{TIME_END} seconds (excluding {TIME_START}s warm-up)")

# Check if it's a directory and find all XML files inside
detector_files = []
if os.path.isdir(detector_path):
    print(f"Found detector directory: {detector_path}")
    # Look for XML files in the directory
    xml_files = [f for f in os.listdir(detector_path) if f.endswith('.xml')]
    if xml_files:
        detector_files = [os.path.join(detector_path, f) for f in xml_files]
        print(f"Found {len(detector_files)} XML files:")
        for f in xml_files:
            print(f"  - {f}")
    else:
        print(f"ERROR: No XML files found in {detector_path}")
        print("Available files:")
        for f in os.listdir(detector_path):
            print(f"  - {f}")
        sys.exit(1)
elif os.path.isfile(detector_path):
    detector_files = [detector_path]
elif os.path.isfile(detector_path + '.xml'):
    detector_files = [detector_path + '.xml']
else:
    print(f"ERROR: Detector file/directory not found at: {detector_path}")
    print(f"Absolute path: {os.path.abspath(detector_path)}")
    sys.exit(1)

print(f"Saving plots to: {output_dir}")

#%%
# ==========================
# PARSE DETECTOR DATA
# ==========================
print("\nParsing detector XML files...")

# Data storage by detector location
detector_data = defaultdict(lambda: {
    'time': [],
    'speed': [],
    'occupancy': [],
    'nVehContrib': [],
    'flow': []
})

# Parse all detector output XML files
for detector_file in detector_files:
    print(f"  Processing: {os.path.basename(detector_file)}")
    try:
        tree = ET.parse(detector_file)
        root = tree.getroot()
        
        # In SUMO detector output, intervals are directly under root
        # Each interval has the detector ID as an attribute
        for interval in root.findall('interval'):
            det_id = interval.get('id')  # Detector ID is in the interval tag
            time_begin = float(interval.get('begin', 0))
            time_end = float(interval.get('end', 0))
            time_mid = (time_begin + time_end) / 2
            
            # Only process data within the specified time range
            if TIME_START <= time_mid <= TIME_END:
                # Get metrics from interval attributes
                speed = float(interval.get('speed', -1))
                occupancy = float(interval.get('occupancy', 0))
                nVehContrib = int(interval.get('nVehContrib', 0))
                flow = float(interval.get('flow', 0))  # Flow is already calculated by SUMO
                
                # Store data
                detector_data[det_id]['time'].append(time_mid)
                detector_data[det_id]['speed'].append(speed * 3.6 if speed >= 0 else np.nan)  # Convert m/s to km/h
                detector_data[det_id]['occupancy'].append(occupancy)
                detector_data[det_id]['nVehContrib'].append(nVehContrib)
                detector_data[det_id]['flow'].append(flow)
        
    except Exception as e:
        print(f"    Warning: Could not parse {os.path.basename(detector_file)}: {e}")
        continue

print(f"\nParsing complete. Found {len(detector_data)} detectors with data in analysis period.")

if len(detector_data) == 0:
    print("\nERROR: No detector data found in any of the XML files.")
    print("This could mean:")
    print("  1. The detector definitions in detectors.add.xml don't match the network")
    print("  2. The simulation didn't run long enough to collect data")
    print("  3. The detector output wasn't properly generated")
    sys.exit(1)

#%%
# ==========================
# PLOT 1: THALWIL (THA) RAMP ANALYSIS
# ==========================
print("\nGenerating Plot 1: Thalwil ramp analysis...")

# Define detector groups for THA
tha_mainline_detectors = ['SENS_A3_THA_MID0', 'SENS_A3_THA_MID1']
tha_after_merge_detectors = ['SENS_A3_THA_N0', 'SENS_A3_THA_N1']
tha_ramp_detector = 'SENS_E_THA'

# Aggregate mainline data (before merge)
if all(det in detector_data for det in tha_mainline_detectors):
    times = detector_data[tha_mainline_detectors[0]]['time']
    
    tha_mainline_speed = []
    tha_mainline_occ = []
    tha_mainline_flow = []
    
    for i in range(len(times)):
        speeds = [detector_data[det]['speed'][i] for det in tha_mainline_detectors]
        occs = [detector_data[det]['occupancy'][i] for det in tha_mainline_detectors]
        flows = [detector_data[det]['flow'][i] for det in tha_mainline_detectors]
        
        tha_mainline_speed.append(np.nanmean(speeds))
        tha_mainline_occ.append(np.mean(occs))
        tha_mainline_flow.append(np.sum(flows))
    
    # Get after-merge data
    tha_after_speed = []
    tha_after_occ = []
    
    for i in range(len(times)):
        speeds = [detector_data[det]['speed'][i] for det in tha_after_merge_detectors]
        occs = [detector_data[det]['occupancy'][i] for det in tha_after_merge_detectors]
        
        tha_after_speed.append(np.nanmean(speeds))
        tha_after_occ.append(np.mean(occs))
    
    # Get ramp data
    tha_ramp_speed = detector_data[tha_ramp_detector]['speed'] if tha_ramp_detector in detector_data else []
    tha_ramp_flow = detector_data[tha_ramp_detector]['flow'] if tha_ramp_detector in detector_data else []
    
    # Create plots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10))
    
    # Top plot: Occupancy and flow
    ax1_twin = ax1.twinx()
    ax1.plot(times, tha_mainline_occ, label='Occupancy Before Merge (%)', color='cyan', linewidth=2)
    ax1.plot(times, tha_after_occ, label='Occupancy After Merge (%)', color='blue', linewidth=2)
    ax1_twin.plot(times, tha_mainline_flow, label='Mainline Flow (veh/h)', color='red', linewidth=2, linestyle='--')
    if tha_ramp_flow:
        ax1_twin.plot(times, tha_ramp_flow, label='Ramp Flow (veh/h)', color='purple', linewidth=2, linestyle='--')
    
    ax1.set_xlabel('Time (seconds)', fontsize=12)
    ax1.set_ylabel('Occupancy (%)', fontsize=12, color='blue')
    ax1_twin.set_ylabel('Flow (veh/h)', fontsize=12, color='red')
    ax1.tick_params(axis='y', labelcolor='blue')
    ax1_twin.tick_params(axis='y', labelcolor='red')
    ax1.set_title(f'Thalwil Ramp - Occupancy and Flow Analysis ({situation_name})', fontsize=14, fontweight='bold')
    ax1.legend(loc='upper left')
    ax1_twin.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim([TIME_START, TIME_END])
    
    # Bottom plot: Speed comparison
    ax2.plot(times, tha_mainline_speed, label='Speed Before Merge (km/h)', color='green', linewidth=2)
    ax2.plot(times, tha_after_speed, label='Speed After Merge (km/h)', color='darkgreen', linewidth=2)
    if tha_ramp_speed:
        ax2.plot(times, tha_ramp_speed, label='Ramp Speed (km/h)', color='orange', linewidth=2)
    
    ax2.axhline(y=80, color='green', linestyle='--', linewidth=1, alpha=0.5)
    ax2.axhline(y=50, color='orange', linestyle='--', linewidth=1, alpha=0.5)
    ax2.axhline(y=30, color='red', linestyle='--', linewidth=1, alpha=0.5)
    
    ax2.set_xlabel('Time (seconds)', fontsize=12)
    ax2.set_ylabel('Speed (km/h)', fontsize=12)
    ax2.set_title('Speed Comparison: Mainline vs Ramp', fontsize=12, fontweight='bold')
    ax2.legend(loc='best')
    ax2.grid(True, alpha=0.3)
    ax2.set_ylim([0, 120])
    ax2.set_xlim([TIME_START, TIME_END])
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, '08_THA_detector_analysis.png'), dpi=300, bbox_inches='tight')
    plt.close()
    print(f"  Saved: 08_THA_detector_analysis.png")

#%%
# ==========================
# PLOT 2: HORGEN (HOR) RAMP ANALYSIS
# ==========================
print("\nGenerating Plot 2: Horgen ramp analysis...")

hor_mainline_detectors = ['SENS_A3_HOR_MID0', 'SENS_A3_HOR_MID1']
hor_after_merge_detectors = ['SENS_A3_HOR_N0', 'SENS_A3_HOR_N1']
hor_ramp_detector = 'SENS_E_HOR'

if all(det in detector_data for det in hor_mainline_detectors):
    times = detector_data[hor_mainline_detectors[0]]['time']
    
    # Similar processing as THA
    hor_mainline_speed = []
    hor_mainline_occ = []
    hor_mainline_flow = []
    
    for i in range(len(times)):
        speeds = [detector_data[det]['speed'][i] for det in hor_mainline_detectors]
        occs = [detector_data[det]['occupancy'][i] for det in hor_mainline_detectors]
        flows = [detector_data[det]['flow'][i] for det in hor_mainline_detectors]
        
        hor_mainline_speed.append(np.nanmean(speeds))
        hor_mainline_occ.append(np.mean(occs))
        hor_mainline_flow.append(np.sum(flows))
    
    hor_after_speed = []
    hor_after_occ = []
    
    for i in range(len(times)):
        speeds = [detector_data[det]['speed'][i] for det in hor_after_merge_detectors]
        occs = [detector_data[det]['occupancy'][i] for det in hor_after_merge_detectors]
        
        hor_after_speed.append(np.nanmean(speeds))
        hor_after_occ.append(np.mean(occs))
    
    hor_ramp_speed = detector_data[hor_ramp_detector]['speed'] if hor_ramp_detector in detector_data else []
    hor_ramp_flow = detector_data[hor_ramp_detector]['flow'] if hor_ramp_detector in detector_data else []
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10))
    
    ax1_twin = ax1.twinx()
    ax1.plot(times, hor_mainline_occ, label='Occupancy Before Merge (%)', color='cyan', linewidth=2)
    ax1.plot(times, hor_after_occ, label='Occupancy After Merge (%)', color='blue', linewidth=2)
    ax1_twin.plot(times, hor_mainline_flow, label='Mainline Flow (veh/h)', color='red', linewidth=2, linestyle='--')
    if hor_ramp_flow:
        ax1_twin.plot(times, hor_ramp_flow, label='Ramp Flow (veh/h)', color='purple', linewidth=2, linestyle='--')
    
    ax1.set_xlabel('Time (seconds)', fontsize=12)
    ax1.set_ylabel('Occupancy (%)', fontsize=12, color='blue')
    ax1_twin.set_ylabel('Flow (veh/h)', fontsize=12, color='red')
    ax1.tick_params(axis='y', labelcolor='blue')
    ax1_twin.tick_params(axis='y', labelcolor='red')
    ax1.set_title(f'Horgen Ramp - Occupancy and Flow Analysis ({situation_name})', fontsize=14, fontweight='bold')
    ax1.legend(loc='upper left')
    ax1_twin.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim([TIME_START, TIME_END])
    
    ax2.plot(times, hor_mainline_speed, label='Speed Before Merge (km/h)', color='green', linewidth=2)
    ax2.plot(times, hor_after_speed, label='Speed After Merge (km/h)', color='darkgreen', linewidth=2)
    if hor_ramp_speed:
        ax2.plot(times, hor_ramp_speed, label='Ramp Speed (km/h)', color='orange', linewidth=2)
    
    ax2.axhline(y=80, color='green', linestyle='--', linewidth=1, alpha=0.5)
    ax2.axhline(y=50, color='orange', linestyle='--', linewidth=1, alpha=0.5)
    ax2.axhline(y=30, color='red', linestyle='--', linewidth=1, alpha=0.5)
    
    ax2.set_xlabel('Time (seconds)', fontsize=12)
    ax2.set_ylabel('Speed (km/h)', fontsize=12)
    ax2.set_title('Speed Comparison: Mainline vs Ramp', fontsize=12, fontweight='bold')
    ax2.legend(loc='best')
    ax2.grid(True, alpha=0.3)
    ax2.set_ylim([0, 120])
    ax2.set_xlim([TIME_START, TIME_END])
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, '09_HOR_detector_analysis.png'), dpi=300, bbox_inches='tight')
    plt.close()
    print(f"  Saved: 09_HOR_detector_analysis.png")

#%%
# ==========================
# PLOT 3: WÄDENSWIL (WAE) RAMP ANALYSIS
# ==========================
print("\nGenerating Plot 3: Wädenswil ramp analysis...")

wae_mainline_detectors = ['SENS_A3_WAE_MID0', 'SENS_A3_WAE_MID1']
wae_after_merge_detectors = ['SENS_A3_WAE_N0', 'SENS_A3_WAE_N1']
wae_ramp_detector = 'SENS_E_WAE'

if all(det in detector_data for det in wae_mainline_detectors):
    times = detector_data[wae_mainline_detectors[0]]['time']
    
    wae_mainline_speed = []
    wae_mainline_occ = []
    wae_mainline_flow = []
    
    for i in range(len(times)):
        speeds = [detector_data[det]['speed'][i] for det in wae_mainline_detectors]
        occs = [detector_data[det]['occupancy'][i] for det in wae_mainline_detectors]
        flows = [detector_data[det]['flow'][i] for det in wae_mainline_detectors]
        
        wae_mainline_speed.append(np.nanmean(speeds))
        wae_mainline_occ.append(np.mean(occs))
        wae_mainline_flow.append(np.sum(flows))
    
    wae_after_speed = []
    wae_after_occ = []
    
    for i in range(len(times)):
        speeds = [detector_data[det]['speed'][i] for det in wae_after_merge_detectors]
        occs = [detector_data[det]['occupancy'][i] for det in wae_after_merge_detectors]
        
        wae_after_speed.append(np.nanmean(speeds))
        wae_after_occ.append(np.mean(occs))
    
    wae_ramp_speed = detector_data[wae_ramp_detector]['speed'] if wae_ramp_detector in detector_data else []
    wae_ramp_flow = detector_data[wae_ramp_detector]['flow'] if wae_ramp_detector in detector_data else []
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10))
    
    ax1_twin = ax1.twinx()
    ax1.plot(times, wae_mainline_occ, label='Occupancy Before Merge (%)', color='cyan', linewidth=2)
    ax1.plot(times, wae_after_occ, label='Occupancy After Merge (%)', color='blue', linewidth=2)
    ax1_twin.plot(times, wae_mainline_flow, label='Mainline Flow (veh/h)', color='red', linewidth=2, linestyle='--')
    if wae_ramp_flow:
        ax1_twin.plot(times, wae_ramp_flow, label='Ramp Flow (veh/h)', color='purple', linewidth=2, linestyle='--')
    
    ax1.set_xlabel('Time (seconds)', fontsize=12)
    ax1.set_ylabel('Occupancy (%)', fontsize=12, color='blue')
    ax1_twin.set_ylabel('Flow (veh/h)', fontsize=12, color='red')
    ax1.tick_params(axis='y', labelcolor='blue')
    ax1_twin.tick_params(axis='y', labelcolor='red')
    ax1.set_title(f'Wädenswil Ramp - Occupancy and Flow Analysis ({situation_name})', fontsize=14, fontweight='bold')
    ax1.legend(loc='upper left')
    ax1_twin.legend(loc='upper right')
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim([TIME_START, TIME_END])
    
    ax2.plot(times, wae_mainline_speed, label='Speed Before Merge (km/h)', color='green', linewidth=2)
    ax2.plot(times, wae_after_speed, label='Speed After Merge (km/h)', color='darkgreen', linewidth=2)
    if wae_ramp_speed:
        ax2.plot(times, wae_ramp_speed, label='Ramp Speed (km/h)', color='orange', linewidth=2)
    
    ax2.axhline(y=80, color='green', linestyle='--', linewidth=1, alpha=0.5)
    ax2.axhline(y=50, color='orange', linestyle='--', linewidth=1, alpha=0.5)
    ax2.axhline(y=30, color='red', linestyle='--', linewidth=1, alpha=0.5)
    
    ax2.set_xlabel('Time (seconds)', fontsize=12)
    ax2.set_ylabel('Speed (km/h)', fontsize=12)
    ax2.set_title('Speed Comparison: Mainline vs Ramp', fontsize=12, fontweight='bold')
    ax2.legend(loc='best')
    ax2.grid(True, alpha=0.3)
    ax2.set_ylim([0, 120])
    ax2.set_xlim([TIME_START, TIME_END])
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, '10_WAE_detector_analysis.png'), dpi=300, bbox_inches='tight')
    plt.close()
    print(f"  Saved: 10_WAE_detector_analysis.png")

#%%
# ==========================
# SUMMARY STATISTICS FROM DETECTORS
# ==========================
print("\n" + "="*60)
print(f"DETECTOR-BASED SUMMARY STATISTICS - {situation_name.upper()}")
print("="*60)

print(f"\nAnalysis Period: {TIME_START}-{TIME_END} seconds ({(TIME_END-TIME_START)/60:.1f} minutes)")
print("\nAvailable detectors:")
for det_id in sorted(detector_data.keys()):
    print(f"  - {det_id}")

print("\n" + "="*60)
print(f"All detector plots saved to: {output_dir}")
print("="*60)

# %%