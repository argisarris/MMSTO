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
os.makedirs('plots', exist_ok=True)

#%%
# ==========================
# START SUMO
# ==========================
if 'SUMO_HOME' in os.environ:
	sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
	print(os.environ["SUMO_HOME"])
sumoBinary = r"C:\Program Files (x86)\Eclipse\Sumo\bin\sumo-gui.exe"
sumoConfigFile = r"C:\Users\ETH\Documents\GitHub\MMSTO\00_asarris_Configuration_Sit0.sumocfg"
sumoCmd = [sumoBinary, "-c", sumoConfigFile, "--start", "--quit-on-end", "--time-to-teleport", "-1"]
traci.start(sumoCmd)

# ==========================
# TIME-RELEVANT PARAMETERS
# ==========================
STEPS_PER_SECOND = 1  # steps/sec
RECORDING_CONTROL_STATS_START_TIME = 240.0

# ==========================
# Simulation (Baseline - No Control)
# ==========================
print("Simulation step length (DeltaT):", traci.simulation.getDeltaT(), "s")
STEP_INTERVAL = 30  # update every 30 simulation steps
occList_THA, occList_HOR, occList_WAE = [], [], []
numVEHList_THA, numVEHList_HOR, numVEHList_WAE = [], [], []
QUEUEList_THA, QUEUEList_HOR, QUEUEList_WAE = [], [], []

# Lists for mainline/merging section monitoring (to detect traffic jams)
speedList_THA_MID, speedList_HOR_MID, speedList_WAE_MID = [], [], []
densityList_THA_MID, densityList_HOR_MID, densityList_WAE_MID = [], [], []

for step in range(4500):
	traci.simulationStep()

	if step > 1000 and step % STEP_INTERVAL == 0:
		# Get occupancies on mainline (mid sections after each ramp)
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

		# Get average speed on mainline (indicator of congestion)
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

		# Get number of vehicles on the ramp
		numVEH_THA = traci.lanearea.getLastStepVehicleNumber("SENS_E_THA")
		numVEH_HOR = traci.lanearea.getLastStepVehicleNumber("SENS_E_HOR")
		numVEH_WAE = traci.lanearea.getLastStepVehicleNumber("SENS_E_WAE")
		numVEHList_THA.append(numVEH_THA)
		numVEHList_HOR.append(numVEH_HOR)
		numVEHList_WAE.append(numVEH_WAE)

		# Get number of vehicles standing on the ramp (queue length)
		VEH_THA = traci.lanearea.getLastStepVehicleIDs("SENS_E_THA")
		VEH_HOR = traci.lanearea.getLastStepVehicleIDs("SENS_E_HOR")
		VEH_WAE = traci.lanearea.getLastStepVehicleIDs("SENS_E_WAE")
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
time_steps = range(len(occList_THA))
occPLOT_THA = np.array(occList_THA)
num_vehPLOT_THA = np.array(numVEHList_THA)
queuePLOT_THA = np.array(QUEUEList_THA)
speedPLOT_THA = np.array(speedList_THA_MID)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))

# Top plot: Occupancy, queue, vehicles on ramp
ax1.plot(time_steps, occPLOT_THA, label='Occupancy on main line (%)', color='blue', linewidth=2)
ax1.plot(time_steps, num_vehPLOT_THA, label='Number of vehicles on the ramp (# vehicles)', color='red', linewidth=2)
ax1.plot(time_steps, queuePLOT_THA, label='Length of standing queue (# vehicles)', color='purple', linewidth=2)
ax1.set_xlabel('Simulation Step')
ax1.set_ylabel('Occupancy / Queue / Vehicles')
ax1.legend(loc='upper left')
ax1.grid(True, alpha=0.3)
ax1.set_title('Ramp and Mainline Metrics - Thalwil (Baseline - No Control)')

# Bottom plot: Speed on mainline (traffic jam indicator)
ax2.plot(time_steps, speedPLOT_THA, label='Average Speed on Mainline (m/s)', color='green', linewidth=2)
ax2.axhline(y=13.89, color='orange', linestyle='--', linewidth=1, label='50 km/h threshold')
ax2.axhline(y=8.33, color='red', linestyle='--', linewidth=1, label='30 km/h threshold (congestion)')
ax2.set_xlabel('Simulation Step')
ax2.set_ylabel('Speed (m/s)')
ax2.legend(loc='upper left')
ax2.grid(True, alpha=0.3)
ax2.set_title('Mainline Speed - Traffic Jam Detection')

plt.tight_layout()
plt.savefig('plots/Sit0_THA_baseline.png', dpi=300, bbox_inches='tight')
print("Saved plot: plots/Sit0_THA_baseline.png")
plt.show()

#%%
# ==========================
# PLOTS FOR HORGEN (HOR)
# ==========================
occPLOT_HOR = np.array(occList_HOR)
num_vehPLOT_HOR = np.array(numVEHList_HOR)
queuePLOT_HOR = np.array(QUEUEList_HOR)
speedPLOT_HOR = np.array(speedList_HOR_MID)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))

# Top plot: Occupancy, queue, vehicles on ramp
ax1.plot(time_steps, occPLOT_HOR, label='Occupancy on main line (%)', color='blue', linewidth=2)
ax1.plot(time_steps, num_vehPLOT_HOR, label='Number of vehicles on the ramp (# vehicles)', color='red', linewidth=2)
ax1.plot(time_steps, queuePLOT_HOR, label='Length of standing queue (# vehicles)', color='purple', linewidth=2)
ax1.set_xlabel('Simulation Step')
ax1.set_ylabel('Occupancy / Queue / Vehicles')
ax1.legend(loc='upper left')
ax1.grid(True, alpha=0.3)
ax1.set_title('Ramp and Mainline Metrics - Horgen (Baseline - No Control)')

# Bottom plot: Speed on mainline (traffic jam indicator)
ax2.plot(time_steps, speedPLOT_HOR, label='Average Speed on Mainline (m/s)', color='green', linewidth=2)
ax2.axhline(y=13.89, color='orange', linestyle='--', linewidth=1, label='50 km/h threshold')
ax2.axhline(y=8.33, color='red', linestyle='--', linewidth=1, label='30 km/h threshold (congestion)')
ax2.set_xlabel('Simulation Step')
ax2.set_ylabel('Speed (m/s)')
ax2.legend(loc='upper left')
ax2.grid(True, alpha=0.3)
ax2.set_title('Mainline Speed - Traffic Jam Detection')

plt.tight_layout()
plt.savefig('plots/Sit0_HOR_baseline.png', dpi=300, bbox_inches='tight')
print("Saved plot: plots/Sit0_HOR_baseline.png")
plt.show()

#%%
# ==========================
# PLOTS FOR WÄDENSWIL (WAE)
# ==========================
occPLOT_WAE = np.array(occList_WAE)
num_vehPLOT_WAE = np.array(numVEHList_WAE)
queuePLOT_WAE = np.array(QUEUEList_WAE)
speedPLOT_WAE = np.array(speedList_WAE_MID)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))

# Top plot: Occupancy, queue, vehicles on ramp
ax1.plot(time_steps, occPLOT_WAE, label='Occupancy on main line (%)', color='blue', linewidth=2)
ax1.plot(time_steps, num_vehPLOT_WAE, label='Number of vehicles on the ramp (# vehicles)', color='red', linewidth=2)
ax1.plot(time_steps, queuePLOT_WAE, label='Length of standing queue (# vehicles)', color='purple', linewidth=2)
ax1.set_xlabel('Simulation Step')
ax1.set_ylabel('Occupancy / Queue / Vehicles')
ax1.legend(loc='upper left')
ax1.grid(True, alpha=0.3)
ax1.set_title('Ramp and Mainline Metrics - Wädenswil (Baseline - No Control)')

# Bottom plot: Speed on mainline (traffic jam indicator)
ax2.plot(time_steps, speedPLOT_WAE, label='Average Speed on Mainline (m/s)', color='green', linewidth=2)
ax2.axhline(y=13.89, color='orange', linestyle='--', linewidth=1, label='50 km/h threshold')
ax2.axhline(y=8.33, color='red', linestyle='--', linewidth=1, label='30 km/h threshold (congestion)')
ax2.set_xlabel('Simulation Step')
ax2.set_ylabel('Speed (m/s)')
ax2.legend(loc='upper left')
ax2.grid(True, alpha=0.3)
ax2.set_title('Mainline Speed - Traffic Jam Detection')

plt.tight_layout()
plt.savefig('plots/Sit0_WAE_baseline.png', dpi=300, bbox_inches='tight')
print("Saved plot: plots/Sit0_WAE_baseline.png")
plt.show()

#%%
# ==========================
# SUMMARY STATISTICS
# ==========================
print("=" * 60)
print("BASELINE SIMULATION SUMMARY (No Control)")
print("=" * 60)
print(f"\nThalwil (THA):")
print(f"  Average Mainline Occupancy: {np.mean(occPLOT_THA):.2f}%")
print(f"  Average Mainline Speed: {np.mean(speedPLOT_THA):.2f} m/s ({np.mean(speedPLOT_THA)*3.6:.2f} km/h)")
print(f"  Max Queue Length: {np.max(queuePLOT_THA):.0f} vehicles")
print(f"  Average Queue Length: {np.mean(queuePLOT_THA):.2f} vehicles")

print(f"\nHorgen (HOR):")
print(f"  Average Mainline Occupancy: {np.mean(occPLOT_HOR):.2f}%")
print(f"  Average Mainline Speed: {np.mean(speedPLOT_HOR):.2f} m/s ({np.mean(speedPLOT_HOR)*3.6:.2f} km/h)")
print(f"  Max Queue Length: {np.max(queuePLOT_HOR):.0f} vehicles")
print(f"  Average Queue Length: {np.mean(queuePLOT_HOR):.2f} vehicles")

print(f"\nWädenswil (WAE):")
print(f"  Average Mainline Occupancy: {np.mean(occPLOT_WAE):.2f}%")
print(f"  Average Mainline Speed: {np.mean(speedPLOT_WAE):.2f} m/s ({np.mean(speedPLOT_WAE)*3.6:.2f} km/h)")
print(f"  Max Queue Length: {np.max(queuePLOT_WAE):.0f} vehicles")
print(f"  Average Queue Length: {np.mean(queuePLOT_WAE):.2f} vehicles")

print("\n" + "=" * 60)
print("NOTE: Speeds below 30 km/h (8.33 m/s) indicate congestion")
print("=" * 60)

# %%