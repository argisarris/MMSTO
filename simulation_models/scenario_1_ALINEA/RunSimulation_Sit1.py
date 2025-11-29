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

#%%
# ==========================
# START SUMO
# ==========================
if 'SUMO_HOME' in os.environ:
	sys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
	print(os.environ["SUMO_HOME"])

# Get the directory where this script is located
script_dir = os.path.dirname(os.path.abspath(__file__))

# ==========================
# TRAFFIC SCALING PARAMETER
# ==========================
TRAFFIC_SCALE = 0.75  # Scale traffic (adjust between 0.0 and 1.0)

sumoBinary = r"C:\Program Files (x86)\Eclipse\Sumo\bin\sumo-gui.exe"
# Use relative path to config file in same directory as script
sumoConfigFile = os.path.join(script_dir, "Configuration_Sit1.sumocfg")
sumoCmd = [sumoBinary, "-c", sumoConfigFile, "--start", "--quit-on-end", "--time-to-teleport", "-1", "--scale", str(TRAFFIC_SCALE)]
traci.start(sumoCmd)

# ==========================
# TIME-RELEVANT PARAMETERS (keep unchanged)
# ==========================
STEPS_PER_SECOND = 1  # steps/sec
SIGNAL_CYCLE_DURATION = 30  # sec #fictive number
VEHICLE_AV_ACC_TIME = 2.0  # sec/veh
RECORDING_CONTROL_STATS_START_TIME = 240.0
Q_MIN = 0       # veh/h
Q_MAX = 1800    # veh/h

# ==========================
# CONTROL PARAMETERS
# ==========================
K_R = 5/60 # veh/sec
OCCUPANCY_TARGET = 20.0  # %
QUEUE_MAX_LENGTH_RAMP_THA = 30 # veh 
QUEUE_MAX_LENGTH_RAMP_HOR = 30 # veh 
QUEUE_MAX_LENGTH_RAMP_WAE = 30 # veh 				


# ==========================
# RAMP ALINEA CONTROL PARAMETERS
# ==========================
ramp_THA = "0" 									#What is this needed for???
traffic_light_THA = "RM_THA"
ramp_HOR = "0"									#What is this needed for???
traffic_light_HOR = "RM_HOR"
ramp_WAE = "0"									#What is this needed for???
traffic_light_WAE = "RM_WAED"


# ==========================
# RAMP ALINEA CONTROL FUNCTION
# ==========================
def control_ALINEA(ramp, q_previous_rate, occupancy_measured, queuelength, QUEUE_MAX_LENGTH_RAMP):
	"""
	ALINEA control logic for a single ramp.

	Parameters:
	- ramp: The ramp ID.
	- q_previous_rate: Previous ramp flow rate (veh/h).
	- occupancy_measured: Measured occupancy (%).

	Returns:
	- q_rate: Updated flow rate (veh/h)
	- metering_rate: Green share for ramp (0..1)
	"""
	# Compute new flow rate (ALINEA formula)
	q_rate = q_previous_rate + K_R * (OCCUPANCY_TARGET - occupancy_measured)
	# Bound flow rate
	q_bounded = min(Q_MAX, max(q_rate, Q_MIN))
	# Compute metering rate as fraction of signal cycle
	metering_rate = (q_bounded * VEHICLE_AV_ACC_TIME) / SIGNAL_CYCLE_DURATION
	metering_rate = min(1.0, math.floor(metering_rate * 10) / 10)  # discretize

	if queuelength > QUEUE_MAX_LENGTH_RAMP:
		# Ramp queue too long, increase green 
		metering_rate = 1
	
	return q_rate, metering_rate



# ==========================
# RAMP ASTRA CONTROL FUNCTION
# ==========================
def control_ASTRA(ramp, q_previous_rate, occupancy_measured, queuelength):
	"""
	ASTRA control logic for a single ramp.

	Parameters:
	- ramp: The ramp ID.
	- q_previous_rate: Previous ramp flow rate (veh/h).
	- occupancy_measured: Measured occupancy (%).

	Returns:
	- q_rate: Updated flow rate (veh/h)
	- metering_rate: Green share for ramp (0..1)
	"""
	# Compute new flow rate (ALINEA formula)
	if queuelength > QUEUE_MAX_LENGTH_RAMP:
		# Ramp queue too long, increase green 
		metering = 14
	else:
		q_rate = q_previous_rate + K_R * (OCCUPANCY_TARGET - occupancy_measured)
	
	# Bound flow rate
	q_bounded = min(Q_MAX, max(q_rate, Q_MIN))
	
	# Compute metering rate as fraction of signal cycle
	metering_rate = (q_bounded * VEHICLE_AV_ACC_TIME) / SIGNAL_CYCLE_DURATION
	metering_rate = min(1.0, math.floor(metering_rate * 10) / 10)  # discretize
	
	return q_rate, metering_rate




# ==========================
# Simulation
# ==========================
print("Simulation step length (DeltaT):", traci.simulation.getDeltaT(), "s")
STEP_INTERVAL = 30  # update every 30 simulation steps
q_rate_prev_THA, q_rate_prev_HOR, q_rate_prev_WAE = 0, 0, 0 # Previous flow rate for individual ramps
occList_THA, occList_HOR, occList_WAE = [], [], []
numVEHList_THA, numVEHList_HOR, numVEHList_WAE = [], [], []
QUEUEList_THA, QUEUEList_HOR, QUEUEList_WAE = [], [], []
meteringrateList_THA, meteringrateList_HOR, meteringrateList_WAE = [], [], []
reddurationList_THA, reddurationList_HOR, reddurationList_WAE = [], [], []

standingqueue_ramp1   = [], [], [], [], []
for step in range(4500):            
	traci.simulationStep()
	
	
	if step > RECORDING_CONTROL_STATS_START_TIME and step % STEP_INTERVAL == 0:
		# get occupancies for ALINEA and append to list
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
		# get number of cars on the ramp
		numVEH_THA = traci.lanearea.getLastStepVehicleNumber("SENS_E_THA")
		numVEH_HOR = traci.lanearea.getLastStepVehicleNumber("SENS_E_HOR")
		numVEH_WAE = traci.lanearea.getLastStepVehicleNumber("SENS_E_WAE")
		numVEHList_THA.append(numVEH_THA)
		numVEHList_HOR.append(numVEH_HOR)
		numVEHList_WAE.append(numVEH_WAE)
		# get number of cars standing on the ramp
		VEH_THA = traci.lanearea.getLastStepVehicleIDs("SENS_E_THA")
		VEH_HOR = traci.lanearea.getLastStepVehicleIDs("SENS_E_HOR")
		VEH_WAE = traci.lanearea.getLastStepVehicleIDs("SENS_E_WAE")
		QUEUEstep_THA = sum(1 for veh_id in VEH_THA if traci.vehicle.getSpeed(veh_id) < 0.01)
		QUEUEstep_HOR = sum(1 for veh_id in VEH_HOR if traci.vehicle.getSpeed(veh_id) < 0.01)
		QUEUEstep_WAE = sum(1 for veh_id in VEH_WAE if traci.vehicle.getSpeed(veh_id) < 0.01)
		QUEUEList_THA.append(QUEUEstep_THA)
		QUEUEList_HOR.append(QUEUEstep_HOR)
		QUEUEList_WAE.append(QUEUEstep_WAE)

		# Apply ALINEA control for THA
		# ==============================
		q_rate_prev_THA, metering_rate_THA = control_ALINEA(ramp_THA, q_rate_prev_THA, occ_THA, QUEUEstep_THA, QUEUE_MAX_LENGTH_RAMP_THA)
		meteringrateList_THA.append(metering_rate_THA)
		# Convert metering rate to green duration
		green_duration_THA = int(metering_rate_THA*SIGNAL_CYCLE_DURATION)
		red_duration_THA = SIGNAL_CYCLE_DURATION - green_duration_THA
		reddurationList_THA.append(red_duration_THA)       
		# Get current traffic light logic
		traffic_light_logic_THA = traci.trafficlight.getAllProgramLogics(traffic_light_THA)[0]
		# Apply new green duration to the ramp signal
		for ph_id in range(len(traffic_light_logic_THA.phases)):
			if ph_id == 0:  # green phase
				traffic_light_logic_THA.phases[ph_id].minDur = green_duration_THA
				traffic_light_logic_THA.phases[ph_id].maxDur = green_duration_THA
				traffic_light_logic_THA.phases[ph_id].duration = green_duration_THA
			elif ph_id == 1:  # red phase
				traffic_light_logic_THA.phases[ph_id].minDur = red_duration_THA
				traffic_light_logic_THA.phases[ph_id].maxDur = red_duration_THA
				traffic_light_logic_THA.phases[ph_id].duration = red_duration_THA
		# Apply updated logic to the traffic light
		traci.trafficlight.setProgramLogic(traffic_light_THA, traffic_light_logic_THA)
		# Reset to green phase so new durations take effect immediately
		traci.trafficlight.setPhase(traffic_light_THA, 0)

		# Apply ALINEA control for HOR
		# ==============================
		q_rate_prev_HOR, metering_rate_HOR = control_ALINEA(ramp_HOR, q_rate_prev_HOR, occ_HOR, QUEUEstep_HOR, QUEUE_MAX_LENGTH_RAMP_HOR)
		meteringrateList_HOR.append(metering_rate_HOR)
		print(metering_rate_HOR)
		# Convert metering rate to red duration
		green_duration_HOR = int(metering_rate_HOR*SIGNAL_CYCLE_DURATION)
		red_duration_HOR = SIGNAL_CYCLE_DURATION - green_duration_HOR
		reddurationList_HOR.append(red_duration_HOR)       
		# Get current traffic light logic
		traffic_light_logic_HOR = traci.trafficlight.getAllProgramLogics(traffic_light_HOR)[0]
		# Apply new green duration to the ramp signal
		for ph_id in range(len(traffic_light_logic_HOR.phases)):
			if ph_id == 0:  # green phase
				traffic_light_logic_HOR.phases[ph_id].minDur = green_duration_HOR
				traffic_light_logic_HOR.phases[ph_id].maxDur = green_duration_HOR
				traffic_light_logic_HOR.phases[ph_id].duration = green_duration_HOR
			elif ph_id == 1:  # red phase
				traffic_light_logic_HOR.phases[ph_id].minDur = red_duration_HOR
				traffic_light_logic_HOR.phases[ph_id].maxDur = red_duration_HOR
				traffic_light_logic_HOR.phases[ph_id].duration = red_duration_HOR
		# Apply updated logic to the traffic light
		traci.trafficlight.setProgramLogic(traffic_light_HOR, traffic_light_logic_HOR)
		# Reset to green phase so new durations take effect immediately
		traci.trafficlight.setPhase(traffic_light_HOR, 0)

		# Apply ALINEA control for WAE
		# ==============================
		q_rate_prev_WAE, metering_rate_WAE = control_ALINEA(ramp_WAE, q_rate_prev_WAE, occ_WAE, QUEUEstep_WAE, QUEUE_MAX_LENGTH_RAMP_WAE)
		meteringrateList_WAE.append(metering_rate_WAE)
		# Convert metering rate to red duration
		green_duration_WAE = int(metering_rate_WAE*SIGNAL_CYCLE_DURATION)
		red_duration_WAE = SIGNAL_CYCLE_DURATION - green_duration_WAE
		reddurationList_WAE.append(red_duration_WAE)       
		# Get current traffic light logic
		traffic_light_logic_WAE = traci.trafficlight.getAllProgramLogics(traffic_light_WAE)[0]
		# Apply new green duration to the ramp signal
		for ph_id in range(len(traffic_light_logic_WAE.phases)):
			if ph_id == 0:  # green phase
				traffic_light_logic_WAE.phases[ph_id].minDur = green_duration_WAE
				traffic_light_logic_WAE.phases[ph_id].maxDur = green_duration_WAE
				traffic_light_logic_WAE.phases[ph_id].duration = green_duration_WAE
			elif ph_id == 1:  # red phase
				traffic_light_logic_WAE.phases[ph_id].minDur = red_duration_WAE
				traffic_light_logic_WAE.phases[ph_id].maxDur = red_duration_WAE
				traffic_light_logic_WAE.phases[ph_id].duration = red_duration_WAE
		# Apply updated logic to the traffic light
		traci.trafficlight.setProgramLogic(traffic_light_WAE, traffic_light_logic_WAE)
		# Reset to green phase so new durations take effect immediately
		traci.trafficlight.setPhase(traffic_light_WAE, 0)

	time.sleep(0)
	
traci.close()
#print(f"Collected Occupancies on main line: ", occupancy_main1)

#%%
# ==========================
# PLOTS
# ==========================
time_steps = range(len(occList_THA))
occPLOT_THA = np.array(occList_THA)        
num_vehPLOT_THA = np.array(numVEHList_THA)
reddurationPLOT_THA = np.array(reddurationList_THA)
queuePLOT_THA = np.array(QUEUEList_THA)

fig, ax1 = plt.subplots(figsize=(12, 6))

# Left axis: mainline occupancy, ramp queue, metering rate
ax1.plot(time_steps, occPLOT_THA, label='Occupancy on main line(%)', color='blue', linewidth=2)
ax1.plot(time_steps, num_vehPLOT_THA, label='Number of vehicles on the ramp (# vehicles)', color='red', linewidth=2)
ax1.plot(time_steps, queuePLOT_THA, label='Length of standing queue (# vehicles)', color='purple', linewidth=2)
ax1.set_xlabel('Simulation Step')
ax1.set_ylabel('Occupancy / Queue / Metering Rate')
ax1.legend(loc='upper left')
ax1.grid(True, alpha=0.3)

# Right axis: red duration in seconds
ax2 = ax1.twinx()
ax2.plot(time_steps, reddurationPLOT_THA, label='Red Duration (s)', color='orange', linestyle='--', linewidth=2)
ax2.set_ylabel('Red Duration (s)')
ax2.legend(loc='upper right')

plt.title('Ramp Metering Evolution over Simulation Steps for Thalwil')
plt.tight_layout()
plt.show()

