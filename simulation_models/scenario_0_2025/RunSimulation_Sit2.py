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

sumoBinary = r"C:\Program Files (x86)\Eclipse\Sumo\bin\sumo-gui.exe"
# Use relative path to config file in same directory as script
sumoConfigFile = os.path.join(script_dir, "Configuration_Sit2.sumocfg")
sumoCmd = [sumoBinary, "-c", sumoConfigFile, "--start", "--quit-on-end", "--time-to-teleport", "-1"]
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
QUEUE_MAX_LENGTH_RAMP_THA = 20 # veh 
QUEUE_MAX_LENGTH_RAMP_HOR = 30 # veh 
QUEUE_MAX_LENGTH_RAMP_WAE = 10 # veh 	

# ==========================
# HERO CONTROL PARAMETERS (simple, tunable)
# ==========================
HERO_QUEUE_THRESHOLD1 = 10   # vehicles: l_i > threshold1 -> activate ramp i-1 as slave
HERO_QUEUE_THRESHOLD2 = 20   # vehicles: l_i + l_{i-1} > threshold2 -> activate ramp i-2 as slave
HERO_MIN_RATE = 0.2          # minimum metering rate (0..1) applied to slave ramps

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
		metering_rate = 1.0
	
	return q_rate, metering_rate



# ==========================
# RAMP ASTRA CONTROL FUNCTION
# ==========================
def control_ASTRA(ramp, q_previous_rate, occupancy_measured, queuelength, QUEUE_MAX_LENGTH_RAMP):
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
# HERO COORDINATION FUNCTION
# ==========================
def apply_HERO(
	occ_bottleneck,
	metering_rate_THA,
	QUEUEstep_THA,
	metering_rate_HOR,
	QUEUEstep_HOR,
	metering_rate_WAE,
	QUEUEstep_WAE
):
	"""
	HERO coordination — respecting flush-out condition:
		queuelength > QUEUE_MAX_LENGHT_RAMP  → metering_rate MUST stay 1.0
	"""

	# ---------------------------------------------------
	# 0) If bottleneck not congested → HERO off
	# ---------------------------------------------------
	if occ_bottleneck <= OCCUPANCY_TARGET:
		return metering_rate_THA, metering_rate_HOR, metering_rate_WAE

	# ---------------------------------------------------
	# 1) Flush-out protection — DO NOT override metering=1.0
	# ---------------------------------------------------
	THA_flush = QUEUEstep_THA > QUEUE_MAX_LENGTH_RAMP_THA
	HOR_flush = QUEUEstep_HOR > QUEUE_MAX_LENGTH_RAMP_HOR
	WAE_flush = QUEUEstep_WAE > QUEUE_MAX_LENGTH_RAMP_WAE

	if THA_flush:
		metering_rate_THA = 1.0
	if HOR_flush:
		metering_rate_HOR = 1.0
	if WAE_flush:
		metering_rate_WAE = 1.0

	# ---------------------------------------------------
	# 2) If THA queue → too large → HOR becomes slave
	# ---------------------------------------------------
	if (QUEUEstep_THA > HERO_QUEUE_THRESHOLD1) and not HOR_flush:
		metering_rate_HOR = min(metering_rate_HOR, HERO_MIN_RATE)

	# ---------------------------------------------------
	# 3) If THA + HOR queues too large → WAE becomes slave
	# ---------------------------------------------------
	if ((QUEUEstep_THA + QUEUEstep_HOR) > HERO_QUEUE_THRESHOLD2) and not WAE_flush:
		metering_rate_WAE = min(metering_rate_WAE, HERO_MIN_RATE)

	return metering_rate_THA, metering_rate_HOR, metering_rate_WAE



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
		occ_THA = (occ_THA_0 + occ_THA_1)/2
		occ_HOR = (occ_HOR_0 + occ_HOR_1)/2
		occ_WAE = (occ_WAE_0 + occ_WAE_1)/2
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
		
		# ==============================
		# Apply ALINEA control (local) for each ramp
		# ==============================
		q_rate_prev_THA, metering_rate_THA = control_ALINEA(
			ramp_THA, q_rate_prev_THA, occ_THA, QUEUEstep_THA, QUEUE_MAX_LENGTH_RAMP_THA
		)
		q_rate_prev_HOR, metering_rate_HOR = control_ALINEA(
			ramp_HOR, q_rate_prev_HOR, occ_HOR, QUEUEstep_HOR, QUEUE_MAX_LENGTH_RAMP_HOR
		)
		q_rate_prev_WAE, metering_rate_WAE = control_ALINEA(
			ramp_WAE, q_rate_prev_WAE, occ_WAE, QUEUEstep_WAE, QUEUE_MAX_LENGTH_RAMP_WAE
		)

		# ==============================
		# HERO COORDINATION LAYER
		# THA = master, HOR/WAE = slaves
		# ==============================
		(metering_rate_THA,
		 metering_rate_HOR,
		 metering_rate_WAE) = apply_HERO(
			occ_bottleneck=occ_THA,
			metering_rate_THA=metering_rate_THA,
			QUEUEstep_THA=QUEUEstep_THA,
			metering_rate_HOR=metering_rate_HOR,
			QUEUEstep_HOR=QUEUEstep_HOR,
			metering_rate_WAE=metering_rate_WAE,
			QUEUEstep_WAE=QUEUEstep_WAE
		)

		# store final metering rates (after HERO)
		meteringrateList_THA.append(metering_rate_THA)
		meteringrateList_HOR.append(metering_rate_HOR)
		meteringrateList_WAE.append(metering_rate_WAE)

		# ==============================
		# Convert metering rate to signal timings & apply
		# ==============================

		# --- THA ---
		green_duration_THA = int(metering_rate_THA * SIGNAL_CYCLE_DURATION)
		red_duration_THA = SIGNAL_CYCLE_DURATION - green_duration_THA
		reddurationList_THA.append(red_duration_THA)
		traffic_light_logic_THA = traci.trafficlight.getAllProgramLogics(traffic_light_THA)[0]
		for ph_id in range(len(traffic_light_logic_THA.phases)):
			if ph_id == 0:  # green phase
				traffic_light_logic_THA.phases[ph_id].minDur = green_duration_THA
				traffic_light_logic_THA.phases[ph_id].maxDur = green_duration_THA
				traffic_light_logic_THA.phases[ph_id].duration = green_duration_THA
			elif ph_id == 1:  # red phase
				traffic_light_logic_THA.phases[ph_id].minDur = red_duration_THA
				traffic_light_logic_THA.phases[ph_id].maxDur = red_duration_THA
				traffic_light_logic_THA.phases[ph_id].duration = red_duration_THA
		traci.trafficlight.setProgramLogic(traffic_light_THA, traffic_light_logic_THA)
		traci.trafficlight.setPhase(traffic_light_THA, 0)

		# --- HOR ---
		green_duration_HOR = int(metering_rate_HOR * SIGNAL_CYCLE_DURATION)
		red_duration_HOR = SIGNAL_CYCLE_DURATION - green_duration_HOR
		reddurationList_HOR.append(red_duration_HOR)
		traffic_light_logic_HOR = traci.trafficlight.getAllProgramLogics(traffic_light_HOR)[0]
		for ph_id in range(len(traffic_light_logic_HOR.phases)):
			if ph_id == 0:  # green phase
				traffic_light_logic_HOR.phases[ph_id].minDur = green_duration_HOR
				traffic_light_logic_HOR.phases[ph_id].maxDur = green_duration_HOR
				traffic_light_logic_HOR.phases[ph_id].duration = green_duration_HOR
			elif ph_id == 1:  # red phase
				traffic_light_logic_HOR.phases[ph_id].minDur = red_duration_HOR
				traffic_light_logic_HOR.phases[ph_id].maxDur = red_duration_HOR
				traffic_light_logic_HOR.phases[ph_id].duration = red_duration_HOR
		traci.trafficlight.setProgramLogic(traffic_light_HOR, traffic_light_logic_HOR)
		traci.trafficlight.setPhase(traffic_light_HOR, 0)
		print(metering_rate_HOR)  # print final HOR rate (after HERO)

		# --- WAE ---
		green_duration_WAE = int(metering_rate_WAE * SIGNAL_CYCLE_DURATION)
		red_duration_WAE = SIGNAL_CYCLE_DURATION - green_duration_WAE
		reddurationList_WAE.append(red_duration_WAE)
		traffic_light_logic_WAE = traci.trafficlight.getAllProgramLogics(traffic_light_WAE)[0]
		for ph_id in range(len(traffic_light_logic_WAE.phases)):
			if ph_id == 0:  # green phase
				traffic_light_logic_WAE.phases[ph_id].minDur = green_duration_WAE
				traffic_light_logic_WAE.phases[ph_id].maxDur = green_duration_WAE
				traffic_light_logic_WAE.phases[ph_id].duration = green_duration_WAE
			elif ph_id == 1:  # red phase
				traffic_light_logic_WAE.phases[ph_id].minDur = red_duration_WAE
				traffic_light_logic_WAE.phases[ph_id].maxDur = red_duration_WAE
				traffic_light_logic_WAE.phases[ph_id].duration = red_duration_WAE
		traci.trafficlight.setProgramLogic(traffic_light_WAE, traffic_light_logic_WAE)
		traci.trafficlight.setPhase(traffic_light_WAE, 0)

	time.sleep(0.1)
	
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


# %%
