#%%
# ==========================
# PYTHON IMPORTS
# ==========================
import os
import sys
import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt
from collections import defaultdict

#%%
# ==========================
# RAMP METERING VERIFICATION SCRIPT
# ==========================
print("="*60)
print("RAMP METERING VERIFICATION ANALYSIS")
print("="*60)

script_dir = os.path.dirname(os.path.abspath(__file__))

scenarios = {
    'sit0': {
        'name': 'No Control',
        'detector_path': os.path.join(script_dir, 'simulation_output', 'scenario_0_Base', 'output_detectors'),
    },
    'sit1': {
        'name': 'ALINEA',
        'detector_path': os.path.join(script_dir, 'simulation_output', 'scenario_1_ALINEA', 'output_detectors'),
    },
    'sit2': {
        'name': 'ALINEA+HERO',
        'detector_path': os.path.join(script_dir, 'simulation_output', 'scenario_2_ALINEA+HERO', 'output_detectors'),
    }
}

# Ramp detectors to check
ramp_detectors = ['SENS_E_WAE', 'SENS_E_HOR', 'SENS_E_THA']
ramp_names = {'SENS_E_WAE': 'Wädenswil', 'SENS_E_HOR': 'Horgen', 'SENS_E_THA': 'Thalwil'}

#%%
# ==========================
# PARSE RAMP DETECTOR DATA
# ==========================
print("\nAnalyzing ramp detector data...\n")

ramp_data = {}

for sit_id, sit_info in scenarios.items():
    print(f"Processing {sit_info['name']}...")
    
    det_path = sit_info['detector_path']
    if not os.path.isdir(det_path):
        print(f"  WARNING: Detector directory not found")
        continue
    
    scenario_ramps = {}
    
    for ramp_det in ramp_detectors:
        xml_file = os.path.join(det_path, f'{ramp_det}.xml')
        
        if not os.path.exists(xml_file):
            print(f"  WARNING: {ramp_det}.xml not found")
            continue
        
        try:
            tree = ET.parse(xml_file)
            root = tree.getroot()
            
            times = []
            speeds = []
            occupancies = []
            flows = []
            
            for interval in root.findall('interval'):
                time_begin = float(interval.get('begin', 0))
                time_end = float(interval.get('end', 0))
                time_mid = (time_begin + time_end) / 2
                
                if 900 <= time_mid <= 4500:  # Analysis period
                    speed = float(interval.get('speed', -1))
                    occupancy = float(interval.get('occupancy', 0))
                    flow = float(interval.get('flow', 0))
                    
                    times.append(time_mid)
                    speeds.append(speed * 3.6 if speed >= 0 else np.nan)
                    occupancies.append(occupancy)
                    flows.append(flow)
            
            scenario_ramps[ramp_det] = {
                'times': times,
                'speeds': speeds,
                'occupancies': occupancies,
                'flows': flows
            }
            
        except Exception as e:
            print(f"  ERROR parsing {ramp_det}: {e}")
    
    ramp_data[sit_id] = scenario_ramps
    print(f"  Found data for {len(scenario_ramps)} ramps")

#%%
# ==========================
# ANALYZE RAMP METERING INDICATORS
# ==========================
print("\n" + "="*60)
print("RAMP METERING INDICATORS")
print("="*60)

for ramp_det in ramp_detectors:
    print(f"\n{ramp_names[ramp_det]} Ramp ({ramp_det}):")
    print("-" * 50)
    
    for sit_id in ['sit0', 'sit1', 'sit2']:
        sit_name = scenarios[sit_id]['name']
        
        if sit_id not in ramp_data or ramp_det not in ramp_data[sit_id]:
            print(f"  {sit_name}: No data")
            continue
        
        data = ramp_data[sit_id][ramp_det]
        
        if not data['occupancies']:
            print(f"  {sit_name}: No occupancy data")
            continue
        
        avg_occ = np.mean(data['occupancies'])
        max_occ = np.max(data['occupancies'])
        avg_flow = np.mean(data['flows'])
        
        # High occupancy on ramp indicates queuing (ramp metering working)
        print(f"  {sit_name}:")
        print(f"    Avg Occupancy: {avg_occ:.1f}%")
        print(f"    Max Occupancy: {max_occ:.1f}%")
        print(f"    Avg Flow: {avg_flow:.1f} veh/h")
        
        # Indicators of ramp metering:
        # - Higher occupancy in sit1/sit2 vs sit0 (vehicles queuing on ramp)
        # - Lower flow in sit1/sit2 vs sit0 (metering restricting flow)

#%%
# ==========================
# VISUAL COMPARISON: RAMP OCCUPANCY
# ==========================
print("\nGenerating ramp occupancy comparison plots...")

output_dir = os.path.join(script_dir, 'simulation_output', 'ramp_metering_check')
os.makedirs(output_dir, exist_ok=True)

for ramp_det in ramp_detectors:
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(16, 10))
    
    ramp_name = ramp_names[ramp_det]
    
    # Plot occupancy
    for sit_id in ['sit0', 'sit1', 'sit2']:
        if sit_id not in ramp_data or ramp_det not in ramp_data[sit_id]:
            continue
        
        data = ramp_data[sit_id][ramp_det]
        sit_info = scenarios[sit_id]
        
        ax1.plot(data['times'], data['occupancies'], 
                label=sit_info['name'], linewidth=2, alpha=0.8,
                color=sit_info.get('color', 'gray'))
    
    ax1.set_xlabel('Time (seconds)', fontsize=12)
    ax1.set_ylabel('Occupancy (%)', fontsize=12)
    ax1.set_title(f'{ramp_name} Ramp - Occupancy Comparison\n(Higher occupancy indicates queuing/metering)', 
                  fontsize=13, fontweight='bold')
    ax1.legend(loc='best', fontsize=11)
    ax1.grid(True, alpha=0.3)
    ax1.set_xlim([900, 4500])
    
    # Plot flow
    for sit_id in ['sit0', 'sit1', 'sit2']:
        if sit_id not in ramp_data or ramp_det not in ramp_data[sit_id]:
            continue
        
        data = ramp_data[sit_id][ramp_det]
        sit_info = scenarios[sit_id]
        
        ax2.plot(data['times'], data['flows'], 
                label=sit_info['name'], linewidth=2, alpha=0.8,
                color=sit_info.get('color', 'gray'))
    
    ax2.set_xlabel('Time (seconds)', fontsize=12)
    ax2.set_ylabel('Flow (veh/h)', fontsize=12)
    ax2.set_title(f'{ramp_name} Ramp - Flow Comparison\n(Lower flow indicates metering restriction)', 
                  fontsize=13, fontweight='bold')
    ax2.legend(loc='best', fontsize=11)
    ax2.grid(True, alpha=0.3)
    ax2.set_xlim([900, 4500])
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, f'{ramp_det}_metering_check.png'), 
                dpi=300, bbox_inches='tight')
    plt.close()
    print(f"  Saved: {ramp_det}_metering_check.png")

#%%
# ==========================
# SUMMARY ASSESSMENT
# ==========================
print("\n" + "="*60)
print("RAMP METERING ASSESSMENT")
print("="*60)

print("\nTo confirm ramp metering is working, look for:")
print("1. HIGHER occupancy on ramps in sit1/sit2 vs sit0")
print("   → Vehicles queuing due to metering")
print("2. LOWER or more controlled flow in sit1/sit2 vs sit0")
print("   → Metering restricting ramp inflow")
print("3. SMOOTHER mainline speeds in sit1/sit2")
print("   → Better traffic flow due to metering")

print("\nIf you see similar occupancy/flow across all scenarios:")
print("→ Ramp metering may NOT be active")
print("→ Check if traffic light programs are properly configured")
print("→ Check simulation logs for errors")

print("\n" + "="*60)
print(f"Analysis complete. Plots saved to: {output_dir}")
print("="*60)

# %%
