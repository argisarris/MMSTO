#%%
# ==========================
# PYTHON IMPORTS
# ==========================
import os
import sys
import traci
import time
import numpy as np
import math
import matplotlib.pyplot as plt

# Create plots directory if it doesn't exist
os.makedirs('../../plots/scenario_4_2040-Peace', exist_ok=True)

#%%
# ==========================
# START SUMO
# ==========================
if 'SUMO_HOME' in os.environ:
	sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
	print(os.environ["SUMO_HOME"])
sumoBinary = r"C:\Program Files (x86)\Eclipse\Sumo\bin\sumo-gui.exe"
sumoConfigFile = r"Configuration.sumocfg"
sumoCmd = [sumoBinary, "-c", sumoConfigFile, "--start", "--quit-on-end", "--time-to-teleport", "-1",
           "--message-log", "sumo_messages.log", "--error-log", "sumo_errors.log", "-v"]
traci.start(sumoCmd)

# ==========================
# TIME-RELEVANT PARAMETERS
# ==========================
STEPS_PER_SECOND = 1  # steps/sec
RECORDING_CONTROL_STATS_START_TIME = 240.0

# ==========================
# Simulation (Scenario 4: 2040-Peace)
# ==========================
print("Simulation step length (DeltaT):", traci.simulation.getDeltaT(), "s")
STEP_INTERVAL = 30  # update every 30 simulation steps
occList_THA, occList_HOR, occList_WAE = [], [], []
numVEHList_THA, numVEHList_HOR, numVEHList_WAE = [], [], []
QUEUEList_THA, QUEUEList_HOR, QUEUEList_WAE = [], [], []

# Lists for mainline/merging section monitoring (to detect traffic jams)
speedList_THA_MID, speedList_HOR_MID, speedList_WAE_MID = [], [], []
speedList_THA_RAMP, speedList_HOR_RAMP, speedList_WAE_RAMP = [], [], []
densityList_THA_MID, densityList_HOR_MID, densityList_WAE_MID = [], [], []

# Lists for before-merging (upstream) occupancy
occList_THA_BEFORE, occList_HOR_BEFORE, occList_WAE_BEFORE = [], [], []

for step in range(4500):
	traci.simulationStep()

	if step > 1000 and step % STEP_INTERVAL == 0:
		# Get occupancies AFTER merging (downstream sensors)
		occ_THA_N0 = traci.inductionloop.getLastIntervalOccupancy("SENS_A3_THA_N0")
		occ_THA_N1 = traci.inductionloop.getLastIntervalOccupancy("SENS_A3_THA_N1")
		occ_HOR_N0 = traci.inductionloop.getLastIntervalOccupancy("SENS_A3_HOR_N0")
		occ_HOR_N1 = traci.inductionloop.getLastIntervalOccupancy("SENS_A3_HOR_N1")
		occ_WAE_N0 = traci.inductionloop.getLastIntervalOccupancy("SENS_A3_WAE_N0")
		occ_WAE_N1 = traci.inductionloop.getLastIntervalOccupancy("SENS_A3_WAE_N1")
		occ_THA_before = occ_THA_N0 + occ_THA_N1
		occ_HOR_before = occ_HOR_N0 + occ_HOR_N1
		occ_WAE_before = occ_WAE_N0 + occ_WAE_N1
		occList_THA_BEFORE.append(occ_THA_before)
		occList_HOR_BEFORE.append(occ_HOR_before)
		occList_WAE_BEFORE.append(occ_WAE_before)

		# Get occupancies BEFORE merging (upstream sections before each ramp)
		occ_THA_0 = traci.inductionloop.getLastIntervalOccupancy("SENS_A3_THA_MID0")
		occ_THA_1 = traci.inductionloop.getLastIntervalOccupancy("SENS_A3_THA_MID1")
		occ_HOR_0 = traci.inductionloop.getLastIntervalOccupancy("SENS_A3_HOR_MID0")
		occ_HOR_1 = traci.inductionloop.getLastIntervalOccupancy("SENS_A3_HOR_MID1")
		occ_WAE_0 = traci.inductionloop.getLastIntervalOccupancy("SENS_A3_WAE_MID0")
		occ_WAE_1 = traci.inductionloop.getLastIntervalOccupancy("SENS_A3_WAE_MID1")
		occ_THA = occ_THA_0 + occ_THA_1
		occ_HOR = occ_HOR_0 + occ_HOR_1
		occ_WAE = occ_WAE_0 + occ_WAE_1
		occList_THA.append(occ_THA)
		occList_HOR.append(occ_HOR)
		occList_WAE.append(occ_WAE)

		# Get average speed on mainline before merge (indicator of congestion)
		speed_THA_0 = traci.inductionloop.getLastIntervalMeanSpeed("SENS_A3_THA_MID0")
		speed_THA_1 = traci.inductionloop.getLastIntervalMeanSpeed("SENS_A3_THA_MID1")
		speed_HOR_0 = traci.inductionloop.getLastIntervalMeanSpeed("SENS_A3_HOR_MID0")
		speed_HOR_1 = traci.inductionloop.getLastIntervalMeanSpeed("SENS_A3_HOR_MID1")
		speed_WAE_0 = traci.inductionloop.getLastIntervalMeanSpeed("SENS_A3_WAE_MID0")
		speed_WAE_1 = traci.inductionloop.getLastIntervalMeanSpeed("SENS_A3_WAE_MID1")
		avg_speed_THA = (speed_THA_0 + speed_THA_1) / 2
		avg_speed_HOR = (speed_HOR_0 + speed_HOR_1) / 2
		avg_speed_WAE = (speed_WAE_0 + speed_WAE_1) / 2
		speedList_THA_MID.append(avg_speed_THA)
		speedList_HOR_MID.append(avg_speed_HOR)
		speedList_WAE_MID.append(avg_speed_WAE)

		# Get average speed on ramps (calculate from individual vehicles)
		VEH_THA = traci.lanearea.getLastStepVehicleIDs("SENS_E_THA")
		VEH_HOR = traci.lanearea.getLastStepVehicleIDs("SENS_E_HOR")
		VEH_WAE = traci.lanearea.getLastStepVehicleIDs("SENS_E_WAE")

		# Calculate average speed for vehicles on ramp (avoid division by zero)
		avg_speed_ramp_THA = np.mean([traci.vehicle.getSpeed(veh_id) for veh_id in VEH_THA]) if len(VEH_THA) > 0 else 0
		avg_speed_ramp_HOR = np.mean([traci.vehicle.getSpeed(veh_id) for veh_id in VEH_HOR]) if len(VEH_HOR) > 0 else 0
		avg_speed_ramp_WAE = np.mean([traci.vehicle.getSpeed(veh_id) for veh_id in VEH_WAE]) if len(VEH_WAE) > 0 else 0

		speedList_THA_RAMP.append(avg_speed_ramp_THA)
		speedList_HOR_RAMP.append(avg_speed_ramp_HOR)
		speedList_WAE_RAMP.append(avg_speed_ramp_WAE)

		# Get number of vehicles on the ramp (from already-collected vehicle IDs)
		numVEH_THA = len(VEH_THA)
		numVEH_HOR = len(VEH_HOR)
		numVEH_WAE = len(VEH_WAE)
		numVEHList_THA.append(numVEH_THA)
		numVEHList_HOR.append(numVEH_HOR)
		numVEHList_WAE.append(numVEH_WAE)

		# Get number of vehicles standing on the ramp (queue length)
		QUEUEstep_THA = sum(1 for veh_id in VEH_THA if traci.vehicle.getSpeed(veh_id) < 0.01)
		QUEUEstep_HOR = sum(1 for veh_id in VEH_HOR if traci.vehicle.getSpeed(veh_id) < 0.01)
		QUEUEstep_WAE = sum(1 for veh_id in VEH_WAE if traci.vehicle.getSpeed(veh_id) < 0.01)
		QUEUEList_THA.append(QUEUEstep_THA)
		QUEUEList_HOR.append(QUEUEstep_HOR)
		QUEUEList_WAE.append(QUEUEstep_WAE)

	time.sleep(0)

traci.close()

#%%
# ==========================
# PLOTS FOR THALWIL (THA)
# ==========================
occPLOT_THA_BEFORE = np.array(occList_THA_BEFORE)
occPLOT_THA_AFTER = np.array(occList_THA)
# Create time axis in seconds matching the actual number of data points collected
time_steps = list(range(1000 + STEP_INTERVAL, 1000 + STEP_INTERVAL + len(occPLOT_THA_AFTER) * STEP_INTERVAL, STEP_INTERVAL))
num_vehPLOT_THA = np.array(numVEHList_THA)
queuePLOT_THA = np.array(QUEUEList_THA)
speedPLOT_THA_MAINLINE = np.array(speedList_THA_MID) * 3.6  # Convert to km/h
speedPLOT_THA_RAMP = np.array(speedList_THA_RAMP) * 3.6  # Convert to km/h

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))

# Top plot: Occupancy (before/after merge), queue, vehicles on ramp
ax1.plot(time_steps, occPLOT_THA_BEFORE, label='Occupancy in A3_THA_N (after THA merge) (%)', color='blue', linewidth=2, linestyle='-')
ax1.plot(time_steps, occPLOT_THA_AFTER, label='Occupancy in A3_THA_MID (before THA merge) (%)', color='cyan', linewidth=2, linestyle='-')
ax1.plot(time_steps, num_vehPLOT_THA, label='Number of vehicles on ramp (# vehicles)', color='red', linewidth=2)
ax1.plot(time_steps, queuePLOT_THA, label='Length of standing queue (# vehicles)', color='purple', linewidth=2)
ax1.set_xlabel('Time (seconds)')
ax1.set_ylabel('Occupancy / Queue / Vehicles')
ax1.legend(loc='upper left')
ax1.grid(True, alpha=0.3)
ax1.set_title('Ramp and Mainline Metrics - Thalwil (Scenario 4: 2040-Peace)')

# Bottom plot: Speed on mainline and ramp
ax2.plot(time_steps, speedPLOT_THA_MAINLINE, label='Speed on Mainline (km/h)', color='green', linewidth=2)
ax2.plot(time_steps, speedPLOT_THA_RAMP, label='Speed on Ramp (km/h)', color='orange', linewidth=2)
ax2.axhline(y=50, color='orange', linestyle='--', linewidth=1, alpha=0.5, label='50 km/h threshold')
ax2.axhline(y=30, color='red', linestyle='--', linewidth=1, alpha=0.5, label='30 km/h threshold (congestion)')
ax2.set_xlabel('Time (seconds)')
ax2.set_ylabel('Speed (km/h)')
ax2.legend(loc='upper left')
ax2.grid(True, alpha=0.3)
ax2.set_title('Speed Comparison: Mainline vs Ramp')

plt.tight_layout()
plot_path = os.path.abspath('../../plots/scenario_4_2040-Peace/THA_peace.png')
if os.path.exists(plot_path):
    os.remove(plot_path)
plt.savefig(plot_path, dpi=300, bbox_inches='tight')
print(f"Saved plot: {plot_path}")
plt.close()

#%%
# ==========================
# PLOTS FOR HORGEN (HOR)
# ==========================
occPLOT_HOR_BEFORE = np.array(occList_HOR_BEFORE)
occPLOT_HOR_AFTER = np.array(occList_HOR)
num_vehPLOT_HOR = np.array(numVEHList_HOR)
queuePLOT_HOR = np.array(QUEUEList_HOR)
speedPLOT_HOR_MAINLINE = np.array(speedList_HOR_MID) * 3.6  # Convert to km/h
speedPLOT_HOR_RAMP = np.array(speedList_HOR_RAMP) * 3.6  # Convert to km/h

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))

# Top plot: Occupancy (before/after merge), queue, vehicles on ramp
ax1.plot(time_steps, occPLOT_HOR_BEFORE, label='Occupancy in A3_THA_S (after HOR merge) (%)', color='blue', linewidth=2, linestyle='-')
ax1.plot(time_steps, occPLOT_HOR_AFTER, label='Occupancy in A3_HOR_MID (before HOR merge) (%)', color='cyan', linewidth=2, linestyle='-')
ax1.plot(time_steps, num_vehPLOT_HOR, label='Number of vehicles on ramp (# vehicles)', color='red', linewidth=2)
ax1.plot(time_steps, queuePLOT_HOR, label='Length of standing queue (# vehicles)', color='purple', linewidth=2)
ax1.set_xlabel('Time (seconds)')
ax1.set_ylabel('Occupancy / Queue / Vehicles')
ax1.legend(loc='upper left')
ax1.grid(True, alpha=0.3)
ax1.set_title('Ramp and Mainline Metrics - Horgen (Scenario 4: 2040-Peace)')

# Bottom plot: Speed on mainline and ramp
ax2.plot(time_steps, speedPLOT_HOR_MAINLINE, label='Speed on Mainline (km/h)', color='green', linewidth=2)
ax2.plot(time_steps, speedPLOT_HOR_RAMP, label='Speed on Ramp (km/h)', color='orange', linewidth=2)
ax2.axhline(y=50, color='orange', linestyle='--', linewidth=1, alpha=0.5, label='50 km/h threshold')
ax2.axhline(y=30, color='red', linestyle='--', linewidth=1, alpha=0.5, label='30 km/h threshold (congestion)')
ax2.set_xlabel('Time (seconds)')
ax2.set_ylabel('Speed (km/h)')
ax2.legend(loc='upper left')
ax2.grid(True, alpha=0.3)
ax2.set_title('Speed Comparison: Mainline vs Ramp')

plt.tight_layout()
plot_path = os.path.abspath('../../plots/scenario_4_2040-Peace/HOR_peace.png')
if os.path.exists(plot_path):
    os.remove(plot_path)
plt.savefig(plot_path, dpi=300, bbox_inches='tight')
print(f"Saved plot: {plot_path}")
plt.close()

#%%
# ==========================
# PLOTS FOR WÄDENSWIL (WAE)
# ==========================
occPLOT_WAE_BEFORE = np.array(occList_WAE_BEFORE)
occPLOT_WAE_AFTER = np.array(occList_WAE)
num_vehPLOT_WAE = np.array(numVEHList_WAE)
queuePLOT_WAE = np.array(QUEUEList_WAE)
speedPLOT_WAE_MAINLINE = np.array(speedList_WAE_MID) * 3.6  # Convert to km/h
speedPLOT_WAE_RAMP = np.array(speedList_WAE_RAMP) * 3.6  # Convert to km/h

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))

# Top plot: Occupancy (before/after merge), queue, vehicles on ramp
ax1.plot(time_steps, occPLOT_WAE_BEFORE, label='Occupancy in A3_HOR_S (after WAE merge) (%)', color='blue', linewidth=2, linestyle='-')
ax1.plot(time_steps, occPLOT_WAE_AFTER, label='Occupancy in A3_WAED_MID (before WAE merge) (%)', color='cyan', linewidth=2, linestyle='-')
ax1.plot(time_steps, num_vehPLOT_WAE, label='Number of vehicles on ramp (# vehicles)', color='red', linewidth=2)
ax1.plot(time_steps, queuePLOT_WAE, label='Length of standing queue (# vehicles)', color='purple', linewidth=2)
ax1.set_xlabel('Time (seconds)')
ax1.set_ylabel('Occupancy / Queue / Vehicles')
ax1.legend(loc='upper left')
ax1.grid(True, alpha=0.3)
ax1.set_title('Ramp and Mainline Metrics - Wädenswil (Scenario 4: 2040-Peace)')

# Bottom plot: Speed on mainline and ramp
ax2.plot(time_steps, speedPLOT_WAE_MAINLINE, label='Speed on Mainline (km/h)', color='green', linewidth=2)
ax2.plot(time_steps, speedPLOT_WAE_RAMP, label='Speed on Ramp (km/h)', color='orange', linewidth=2)
ax2.axhline(y=50, color='orange', linestyle='--', linewidth=1, alpha=0.5, label='50 km/h threshold')
ax2.axhline(y=30, color='red', linestyle='--', linewidth=1, alpha=0.5, label='30 km/h threshold (congestion)')
ax2.set_xlabel('Time (seconds)')
ax2.set_ylabel('Speed (km/h)')
ax2.legend(loc='upper left')
ax2.grid(True, alpha=0.3)
ax2.set_title('Speed Comparison: Mainline vs Ramp')

plt.tight_layout()
plot_path = os.path.abspath('../../plots/scenario_4_2040-Peace/WAE_peace.png')
if os.path.exists(plot_path):
    os.remove(plot_path)
plt.savefig(plot_path, dpi=300, bbox_inches='tight')
print(f"Saved plot: {plot_path}")
plt.close()

#%%
# ==========================
# SUMMARY STATISTICS
# ==========================
print("=" * 60)
print("SCENARIO 4 SIMULATION SUMMARY (2040-Peace)")
print("=" * 60)
print(f"\nThalwil (THA):")
print(f"  Average Occupancy (A3_THA_N - after THA merge): {np.mean(occPLOT_THA_BEFORE):.2f}%")
print(f"  Average Occupancy (A3_THA_MID - before THA merge): {np.mean(occPLOT_THA_AFTER):.2f}%")
print(f"  Average Mainline Speed: {np.mean(speedPLOT_THA_MAINLINE):.2f} km/h")
print(f"  Average Ramp Speed: {np.mean(speedPLOT_THA_RAMP):.2f} km/h")
print(f"  Max Queue Length: {np.max(queuePLOT_THA):.0f} vehicles")
print(f"  Average Queue Length: {np.mean(queuePLOT_THA):.2f} vehicles")

print(f"\nHorgen (HOR):")
print(f"  Average Occupancy (A3_THA_S - after HOR merge): {np.mean(occPLOT_HOR_BEFORE):.2f}%")
print(f"  Average Occupancy (A3_HOR_MID - before HOR merge): {np.mean(occPLOT_HOR_AFTER):.2f}%")
print(f"  Average Mainline Speed: {np.mean(speedPLOT_HOR_MAINLINE):.2f} km/h")
print(f"  Average Ramp Speed: {np.mean(speedPLOT_HOR_RAMP):.2f} km/h")
print(f"  Max Queue Length: {np.max(queuePLOT_HOR):.0f} vehicles")
print(f"  Average Queue Length: {np.mean(queuePLOT_HOR):.2f} vehicles")

print(f"\nWädenswil (WAE):")
print(f"  Average Occupancy (A3_HOR_S - after WAE merge): {np.mean(occPLOT_WAE_BEFORE):.2f}%")
print(f"  Average Occupancy (A3_WAED_MID - before WAE merge): {np.mean(occPLOT_WAE_AFTER):.2f}%")
print(f"  Average Mainline Speed: {np.mean(speedPLOT_WAE_MAINLINE):.2f} km/h")
print(f"  Average Ramp Speed: {np.mean(speedPLOT_WAE_RAMP):.2f} km/h")
print(f"  Max Queue Length: {np.max(queuePLOT_WAE):.0f} vehicles")
print(f"  Average Queue Length: {np.mean(queuePLOT_WAE):.2f} vehicles")

print("\n" + "=" * 60)
print("OVERALL NETWORK PERFORMANCE")
print("=" * 60)

# Calculate overall average speed across all 3 measurement points
all_speeds = np.concatenate([speedPLOT_THA_MAINLINE, speedPLOT_HOR_MAINLINE, speedPLOT_WAE_MAINLINE])
overall_avg_speed = np.mean(all_speeds)
print(f"  Overall Average Mainline Speed: {overall_avg_speed:.2f} km/h")

# Calculate congestion metrics (speed < 30 km/h)
congestion_threshold = 30  # km/h
congested_THA = np.sum(speedPLOT_THA_MAINLINE < congestion_threshold)
congested_HOR = np.sum(speedPLOT_HOR_MAINLINE < congestion_threshold)
congested_WAE = np.sum(speedPLOT_WAE_MAINLINE < congestion_threshold)
total_measurements = len(speedPLOT_THA_MAINLINE)

pct_congested_THA = (congested_THA / total_measurements) * 100
pct_congested_HOR = (congested_HOR / total_measurements) * 100
pct_congested_WAE = (congested_WAE / total_measurements) * 100

print(f"\nCONGESTION ANALYSIS (Speed < 30 km/h):")
print(f"  THA: {pct_congested_THA:.1f}% of time congested ({congested_THA}/{total_measurements} measurements)")
print(f"  HOR: {pct_congested_HOR:.1f}% of time congested ({congested_HOR}/{total_measurements} measurements)")
print(f"  WAE: {pct_congested_WAE:.1f}% of time congested ({congested_WAE}/{total_measurements} measurements)")

# Calculate min speeds (worst congestion)
min_speed_THA = np.min(speedPLOT_THA_MAINLINE)
min_speed_HOR = np.min(speedPLOT_HOR_MAINLINE)
min_speed_WAE = np.min(speedPLOT_WAE_MAINLINE)
print(f"\nMINIMUM SPEEDS (Worst Congestion):")
print(f"  THA: {min_speed_THA:.2f} km/h")
print(f"  HOR: {min_speed_HOR:.2f} km/h")
print(f"  WAE: {min_speed_WAE:.2f} km/h")

print("\n" + "=" * 60)
print("NOTE: Highway congestion occurs when speeds drop below:")
print("  - 50 km/h: Moderate slowdown")
print("  - 30 km/h: Severe congestion (stop-and-go traffic)")
print("  - 10 km/h: Near-gridlock conditions")
print("=" * 60)

# %%
