#!/usr/bin/env python3
"""
Script to generate Sit1 (ALINEA) and Sit2 (ALINEA+HERO) configuration and RunSimulation files
for all 4 demand scenarios in the MMSTO project.

Author: Generated for asarris
Date: 2025-11-27
"""

import os

# Define scenarios
SCENARIOS = [
    {
        "name": "scenario_1_2040-None",
        "description": "2040-None (BASELINE - SUMO Defaults)",
        "vTypes": "vTypes_default.xml"
    },
    {
        "name": "scenario_2_2040-Mixed",
        "description": "2040-Mixed (Current Setup)",
        "vTypes": "vTypes.xml"
    },
    {
        "name": "scenario_3_2040-Mid",
        "description": "2040-Mid (Moderate aggression)",
        "vTypes": "vTypes.xml"
    },
    {
        "name": "scenario_4_2040-Peace",
        "description": "2040-Peace (Peaceful driving)",
        "vTypes": "vTypes.xml"
    }
]

def create_configuration_sit1(scenario_name, scenario_desc, vtypes_file):
    """Create Configuration_Sit1.sumocfg for a scenario"""
    content = f"""<?xml version="1.0" encoding="UTF-8"?>

<!-- SUMO Configuration for A3 Highway Network -->
<!-- Author: asarris -->
<!-- Scenario: {scenario_desc} - Situation 1 (ALINEA Ramp Metering) -->

<sumoConfiguration xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
                   xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/sumoConfiguration.xsd">

    <input>
        <net-file value="..\\..\\shared_simulation_files\\Network_TL.net.xml"/>
        <route-files value="..\\..\\shared_simulation_files\\routes.rou.xml,Demand.rou.xml"/>
        <gui-settings-file value="..\\..\\shared_simulation_files\\viewsettings_asarris_personal.xml"/>
        <additional-files value="..\\..\\shared_simulation_files\\{vtypes_file},..\\..\\shared_simulation_files\\detectors.add.xml"/>
    </input>

    <output>
        <fcd-output value="..\\..\\simulation_output\\{scenario_name}_Sit1\\output_fcd.xml"/>
        <emission-output value="..\\..\\simulation_output\\{scenario_name}_Sit1\\output_emissions.xml"/>
        <summary-output value="..\\..\\simulation_output\\{scenario_name}_Sit1\\output_summary.xml"/>
        <tripinfo-output value="..\\..\\simulation_output\\{scenario_name}_Sit1\\output_tripinfo.xml"/>
    </output>

    <time>
        <begin value="0"/>
        <end value="4500"/>
    </time>

    <processing>
        <collision.check-junctions value="true"/>
        <time-to-teleport value="-1"/>
    </processing>

</sumoConfiguration>
"""
    return content

def create_configuration_sit2(scenario_name, scenario_desc, vtypes_file):
    """Create Configuration_Sit2.sumocfg for a scenario"""
    content = f"""<?xml version="1.0" encoding="UTF-8"?>

<!-- SUMO Configuration for A3 Highway Network -->
<!-- Author: asarris -->
<!-- Scenario: {scenario_desc} - Situation 2 (ALINEA + HERO Coordination) -->

<sumoConfiguration xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
                   xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/sumoConfiguration.xsd">

    <input>
        <net-file value="..\\..\\shared_simulation_files\\Network_TL.net.xml"/>
        <route-files value="..\\..\\shared_simulation_files\\routes.rou.xml,Demand.rou.xml"/>
        <gui-settings-file value="..\\..\\shared_simulation_files\\viewsettings_asarris_personal.xml"/>
        <additional-files value="..\\..\\shared_simulation_files\\{vtypes_file},..\\..\\shared_simulation_files\\detectors.add.xml"/>
    </input>

    <output>
        <fcd-output value="..\\..\\simulation_output\\{scenario_name}_Sit2\\output_fcd.xml"/>
        <emission-output value="..\\..\\simulation_output\\{scenario_name}_Sit2\\output_emissions.xml"/>
        <summary-output value="..\\..\\simulation_output\\{scenario_name}_Sit2\\output_summary.xml"/>
        <tripinfo-output value="..\\..\\simulation_output\\{scenario_name}_Sit2\\output_tripinfo.xml"/>
    </output>

    <time>
        <begin value="0"/>
        <end value="4500"/>
    </time>

    <processing>
        <collision.check-junctions value="true"/>
        <time-to-teleport value="-1"/>
    </processing>

</sumoConfiguration>
"""
    return content

def create_runsimulation_sit1(scenario_name, scenario_desc):
    """Create RunSimulation_Sit1.py for a scenario"""
    content = f'''#%%
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
os.makedirs('../../plots/{scenario_name}_Sit1', exist_ok=True)

#%%
# ==========================
# START SUMO
# ==========================
if 'SUMO_HOME' in os.environ:
\tsys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
\tprint(os.environ["SUMO_HOME"])
sumoBinary = r"C:\\Program Files (x86)\\Eclipse\\Sumo\\bin\\sumo-gui.exe"
sumoConfigFile = r"Configuration_Sit1.sumocfg"
sumoCmd = [sumoBinary, "-c", sumoConfigFile, "--start", "--quit-on-end", "--time-to-teleport", "-1",
           "--message-log", "sumo_messages.log", "--error-log", "sumo_errors.log", "-v"]
traci.start(sumoCmd)

# ==========================
# TIME-RELEVANT PARAMETERS
# ==========================
STEPS_PER_SECOND = 1  # steps/sec
SIGNAL_CYCLE_DURATION = 30  # sec
VEHICLE_AV_ACC_TIME = 2.0  # sec/veh
RECORDING_CONTROL_STATS_START_TIME = 240.0
Q_MIN = 0       # veh/h
Q_MAX = 1800    # veh/h

# ==========================
# CONTROL PARAMETERS (ALINEA)
# ==========================
K_R = 5/60  # veh/sec
OCCUPANCY_TARGET = 20.0  # %
QUEUE_MAX_LENGTH_RAMP_THA = 30  # veh
QUEUE_MAX_LENGTH_RAMP_HOR = 30  # veh
QUEUE_MAX_LENGTH_RAMP_WAE = 30  # veh

# ==========================
# RAMP ALINEA CONTROL PARAMETERS
# ==========================
traffic_light_THA = "RM_THA"
traffic_light_HOR = "RM_HOR"
traffic_light_WAE = "RM_WAED"

# ==========================
# RAMP ALINEA CONTROL FUNCTION
# ==========================
def control_ALINEA(q_previous_rate, occupancy_measured, queuelength, QUEUE_MAX_LENGTH_RAMP):
\t"""
\tALINEA control logic for a single ramp.
\t
\tParameters:
\t- q_previous_rate: Previous ramp flow rate (veh/h).
\t- occupancy_measured: Measured occupancy (%).
\t- queuelength: Current queue length on ramp (# vehicles).
\t- QUEUE_MAX_LENGTH_RAMP: Maximum allowable queue length.
\t
\tReturns:
\t- q_rate: Updated flow rate (veh/h)
\t- metering_rate: Green share for ramp (0..1)
\t"""
\t# Compute new flow rate (ALINEA formula)
\tq_rate = q_previous_rate + K_R * (OCCUPANCY_TARGET - occupancy_measured)
\t# Bound flow rate
\tq_bounded = min(Q_MAX, max(q_rate, Q_MIN))
\t# Compute metering rate as fraction of signal cycle
\tmetering_rate = (q_bounded * VEHICLE_AV_ACC_TIME) / SIGNAL_CYCLE_DURATION
\tmetering_rate = min(1.0, math.floor(metering_rate * 10) / 10)  # discretize
\t
\tif queuelength > QUEUE_MAX_LENGTH_RAMP:
\t\t# Ramp queue too long, increase green
\t\tmetering_rate = 1.0
\t
\treturn q_rate, metering_rate

# ==========================
# Simulation (Scenario: {scenario_desc} - Sit1: ALINEA)
# ==========================
print("Simulation step length (DeltaT):", traci.simulation.getDeltaT(), "s")
STEP_INTERVAL = 30  # update every 30 simulation steps

# Initialize control variables
q_rate_THA = Q_MAX / 2
q_rate_HOR = Q_MAX / 2
q_rate_WAE = Q_MAX / 2

# Lists for monitoring
occList_THA, occList_HOR, occList_WAE = [], [], []
numVEHList_THA, numVEHList_HOR, numVEHList_WAE = [], [], []
QUEUEList_THA, QUEUEList_HOR, QUEUEList_WAE = [], [], []
speedList_THA_MID, speedList_HOR_MID, speedList_WAE_MID = [], [], []
speedList_THA_RAMP, speedList_HOR_RAMP, speedList_WAE_RAMP = [], [], []
meteringRateList_THA, meteringRateList_HOR, meteringRateList_WAE = [], [], []

for step in range(4500):
\ttraci.simulationStep()
\t
\tif step > 1000 and step % STEP_INTERVAL == 0:
\t\t# Get occupancies at mainline sensors (before merge)
\t\tocc_THA_0 = traci.inductionloop.getLastIntervalOccupancy("SENS_A3_THA_MID0")
\t\tocc_THA_1 = traci.inductionloop.getLastIntervalOccupancy("SENS_A3_THA_MID1")
\t\tocc_HOR_0 = traci.inductionloop.getLastIntervalOccupancy("SENS_A3_HOR_MID0")
\t\tocc_HOR_1 = traci.inductionloop.getLastIntervalOccupancy("SENS_A3_HOR_MID1")
\t\tocc_WAE_0 = traci.inductionloop.getLastIntervalOccupancy("SENS_A3_WAE_MID0")
\t\tocc_WAE_1 = traci.inductionloop.getLastIntervalOccupancy("SENS_A3_WAE_MID1")
\t\t
\t\tocc_THA = occ_THA_0 + occ_THA_1
\t\tocc_HOR = occ_HOR_0 + occ_HOR_1
\t\tocc_WAE = occ_WAE_0 + occ_WAE_1
\t\toccList_THA.append(occ_THA)
\t\toccList_HOR.append(occ_HOR)
\t\toccList_WAE.append(occ_WAE)
\t\t
\t\t# Get queue lengths at ramps
\t\tVEH_THA = traci.lanearea.getLastStepVehicleIDs("SENS_E_THA")
\t\tVEH_HOR = traci.lanearea.getLastStepVehicleIDs("SENS_E_HOR")
\t\tVEH_WAE = traci.lanearea.getLastStepVehicleIDs("SENS_E_WAE")
\t\t
\t\tQUEUE_THA = sum(1 for veh_id in VEH_THA if traci.vehicle.getSpeed(veh_id) < 0.01)
\t\tQUEUE_HOR = sum(1 for veh_id in VEH_HOR if traci.vehicle.getSpeed(veh_id) < 0.01)
\t\tQUEUE_WAE = sum(1 for veh_id in VEH_WAE if traci.vehicle.getSpeed(veh_id) < 0.01)
\t\t
\t\tQUEUEList_THA.append(QUEUE_THA)
\t\tQUEUEList_HOR.append(QUEUE_HOR)
\t\tQUEUEList_WAE.append(QUEUE_WAE)
\t\tnumVEHList_THA.append(len(VEH_THA))
\t\tnumVEHList_HOR.append(len(VEH_HOR))
\t\tnumVEHList_WAE.append(len(VEH_WAE))
\t\t
\t\t# Apply ALINEA control
\t\tq_rate_THA, metering_rate_THA = control_ALINEA(q_rate_THA, occ_THA, QUEUE_THA, QUEUE_MAX_LENGTH_RAMP_THA)
\t\tq_rate_HOR, metering_rate_HOR = control_ALINEA(q_rate_HOR, occ_HOR, QUEUE_HOR, QUEUE_MAX_LENGTH_RAMP_HOR)
\t\tq_rate_WAE, metering_rate_WAE = control_ALINEA(q_rate_WAE, occ_WAE, QUEUE_WAE, QUEUE_MAX_LENGTH_RAMP_WAE)
\t\t
\t\tmeteringRateList_THA.append(metering_rate_THA)
\t\tmeteringRateList_HOR.append(metering_rate_HOR)
\t\tmeteringRateList_WAE.append(metering_rate_WAE)
\t\t
\t\t# Set traffic light phases (simplified: green time proportional to metering rate)
\t\tgreen_time_THA = int(metering_rate_THA * SIGNAL_CYCLE_DURATION)
\t\tgreen_time_HOR = int(metering_rate_HOR * SIGNAL_CYCLE_DURATION)
\t\tgreen_time_WAE = int(metering_rate_WAE * SIGNAL_CYCLE_DURATION)
\t\t
\t\t# Set phase durations (alternating green/red)
\t\ttraci.trafficlight.setPhaseDuration(traffic_light_THA, green_time_THA)
\t\ttraci.trafficlight.setPhaseDuration(traffic_light_HOR, green_time_HOR)
\t\ttraci.trafficlight.setPhaseDuration(traffic_light_WAE, green_time_WAE)
\t\t
\t\t# Get speeds
\t\tspeed_THA_0 = traci.inductionloop.getLastIntervalMeanSpeed("SENS_A3_THA_MID0")
\t\tspeed_THA_1 = traci.inductionloop.getLastIntervalMeanSpeed("SENS_A3_THA_MID1")
\t\tspeed_HOR_0 = traci.inductionloop.getLastIntervalMeanSpeed("SENS_A3_HOR_MID0")
\t\tspeed_HOR_1 = traci.inductionloop.getLastIntervalMeanSpeed("SENS_A3_HOR_MID1")
\t\tspeed_WAE_0 = traci.inductionloop.getLastIntervalMeanSpeed("SENS_A3_WAE_MID0")
\t\tspeed_WAE_1 = traci.inductionloop.getLastIntervalMeanSpeed("SENS_A3_WAE_MID1")
\t\t
\t\tspeedList_THA_MID.append((speed_THA_0 + speed_THA_1) / 2)
\t\tspeedList_HOR_MID.append((speed_HOR_0 + speed_HOR_1) / 2)
\t\tspeedList_WAE_MID.append((speed_WAE_0 + speed_WAE_1) / 2)
\t\t
\t\tavg_speed_ramp_THA = np.mean([traci.vehicle.getSpeed(veh_id) for veh_id in VEH_THA]) if len(VEH_THA) > 0 else 0
\t\tavg_speed_ramp_HOR = np.mean([traci.vehicle.getSpeed(veh_id) for veh_id in VEH_HOR]) if len(VEH_HOR) > 0 else 0
\t\tavg_speed_ramp_WAE = np.mean([traci.vehicle.getSpeed(veh_id) for veh_id in VEH_WAE]) if len(VEH_WAE) > 0 else 0
\t\t
\t\tspeedList_THA_RAMP.append(avg_speed_ramp_THA)
\t\tspeedList_HOR_RAMP.append(avg_speed_ramp_HOR)
\t\tspeedList_WAE_RAMP.append(avg_speed_ramp_WAE)
\t
\ttime.sleep(0)

traci.close()

#%%
# ==========================
# SUMMARY STATISTICS
# ==========================
print("=" * 60)
print("SCENARIO: {scenario_desc} - SITUATION 1 (ALINEA)")
print("=" * 60)
print(f"Average Occupancy THA: {{np.mean(occList_THA):.2f}}%")
print(f"Average Occupancy HOR: {{np.mean(occList_HOR):.2f}}%")
print(f"Average Occupancy WAE: {{np.mean(occList_WAE):.2f}}%")
print(f"Average Queue THA: {{np.mean(QUEUEList_THA):.2f}} vehicles")
print(f"Average Queue HOR: {{np.mean(QUEUEList_HOR):.2f}} vehicles")
print(f"Average Queue WAE: {{np.mean(QUEUEList_WAE):.2f}} vehicles")
print(f"Average Metering Rate THA: {{np.mean(meteringRateList_THA):.2f}}")
print(f"Average Metering Rate HOR: {{np.mean(meteringRateList_HOR):.2f}}")
print(f"Average Metering Rate WAE: {{np.mean(meteringRateList_WAE):.2f}}")
print("=" * 60)

# %%
'''
    return content

def create_runsimulation_sit2(scenario_name, scenario_desc):
    """Create RunSimulation_Sit2.py for a scenario (ALINEA + HERO)"""
    content = f'''#%%
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
os.makedirs('../../plots/{scenario_name}_Sit2', exist_ok=True)

#%%
# ==========================
# START SUMO
# ==========================
if 'SUMO_HOME' in os.environ:
\tsys.path.append(os.path.join(os.environ['SUMO_HOME'], 'tools'))
\tprint(os.environ["SUMO_HOME"])
sumoBinary = r"C:\\Program Files (x86)\\Eclipse\\Sumo\\bin\\sumo-gui.exe"
sumoConfigFile = r"Configuration_Sit2.sumocfg"
sumoCmd = [sumoBinary, "-c", sumoConfigFile, "--start", "--quit-on-end", "--time-to-teleport", "-1",
           "--message-log", "sumo_messages.log", "--error-log", "sumo_errors.log", "-v"]
traci.start(sumoCmd)

# ==========================
# TIME-RELEVANT PARAMETERS
# ==========================
STEPS_PER_SECOND = 1  # steps/sec
SIGNAL_CYCLE_DURATION = 30  # sec
VEHICLE_AV_ACC_TIME = 2.0  # sec/veh
RECORDING_CONTROL_STATS_START_TIME = 240.0
Q_MIN = 0       # veh/h
Q_MAX = 1800    # veh/h

# ==========================
# CONTROL PARAMETERS (ALINEA)
# ==========================
K_R = 5/60  # veh/sec
OCCUPANCY_TARGET = 20.0  # %
QUEUE_MAX_LENGTH_RAMP_THA = 20  # veh
QUEUE_MAX_LENGTH_RAMP_HOR = 30  # veh
QUEUE_MAX_LENGTH_RAMP_WAE = 10  # veh

# ==========================
# HERO CONTROL PARAMETERS
# ==========================
HERO_QUEUE_THRESHOLD1 = 10   # vehicles: l_i > threshold1 -> activate ramp i-1 as slave
HERO_QUEUE_THRESHOLD2 = 20   # vehicles: l_i + l_{{i-1}} > threshold2 -> activate ramp i-2 as slave
HERO_MIN_RATE = 0.2          # minimum metering rate (0..1) applied to slave ramps

# ==========================
# RAMP ALINEA CONTROL PARAMETERS
# ==========================
traffic_light_THA = "RM_THA"
traffic_light_HOR = "RM_HOR"
traffic_light_WAE = "RM_WAED"

# ==========================
# RAMP ALINEA CONTROL FUNCTION
# ==========================
def control_ALINEA(q_previous_rate, occupancy_measured, queuelength, QUEUE_MAX_LENGTH_RAMP):
\t"""
\tALINEA control logic for a single ramp.
\t
\tParameters:
\t- q_previous_rate: Previous ramp flow rate (veh/h).
\t- occupancy_measured: Measured occupancy (%).
\t- queuelength: Current queue length on ramp (# vehicles).
\t- QUEUE_MAX_LENGTH_RAMP: Maximum allowable queue length.
\t
\tReturns:
\t- q_rate: Updated flow rate (veh/h)
\t- metering_rate: Green share for ramp (0..1)
\t"""
\t# Compute new flow rate (ALINEA formula)
\tq_rate = q_previous_rate + K_R * (OCCUPANCY_TARGET - occupancy_measured)
\t# Bound flow rate
\tq_bounded = min(Q_MAX, max(q_rate, Q_MIN))
\t# Compute metering rate as fraction of signal cycle
\tmetering_rate = (q_bounded * VEHICLE_AV_ACC_TIME) / SIGNAL_CYCLE_DURATION
\tmetering_rate = min(1.0, math.floor(metering_rate * 10) / 10)  # discretize
\t
\tif queuelength > QUEUE_MAX_LENGTH_RAMP:
\t\t# Ramp queue too long, increase green
\t\tmetering_rate = 1.0
\t
\treturn q_rate, metering_rate

# ==========================
# HERO COORDINATION FUNCTION
# ==========================
def apply_HERO_coordination(metering_rates, queue_lengths):
\t"""
\tApply HERO coordination to adjust metering rates based on downstream queue lengths.
\tRamps are ordered from downstream to upstream: WAE -> HOR -> THA
\t
\tParameters:
\t- metering_rates: dict with keys 'THA', 'HOR', 'WAE' containing ALINEA metering rates
\t- queue_lengths: dict with keys 'THA', 'HOR', 'WAE' containing current queue lengths
\t
\tReturns:
\t- adjusted_rates: dict with adjusted metering rates after HERO coordination
\t"""
\tadjusted = metering_rates.copy()
\t
\t# Check WAE (most downstream) - if congested, reduce HOR
\tif queue_lengths['WAE'] > HERO_QUEUE_THRESHOLD1:
\t\tadjusted['HOR'] = max(HERO_MIN_RATE, adjusted['HOR'] * 0.5)
\t\t
\t# Check HOR - if congested, reduce THA
\tif queue_lengths['HOR'] > HERO_QUEUE_THRESHOLD1:
\t\tadjusted['THA'] = max(HERO_MIN_RATE, adjusted['THA'] * 0.5)
\t\t
\t# Check combined WAE+HOR - if very congested, further reduce THA
\tif queue_lengths['WAE'] + queue_lengths['HOR'] > HERO_QUEUE_THRESHOLD2:
\t\tadjusted['THA'] = max(HERO_MIN_RATE, adjusted['THA'] * 0.3)
\t
\treturn adjusted

# ==========================
# Simulation (Scenario: {scenario_desc} - Sit2: ALINEA + HERO)
# ==========================
print("Simulation step length (DeltaT):", traci.simulation.getDeltaT(), "s")
STEP_INTERVAL = 30  # update every 30 simulation steps

# Initialize control variables
q_rate_THA = Q_MAX / 2
q_rate_HOR = Q_MAX / 2
q_rate_WAE = Q_MAX / 2

# Lists for monitoring
occList_THA, occList_HOR, occList_WAE = [], [], []
numVEHList_THA, numVEHList_HOR, numVEHList_WAE = [], [], []
QUEUEList_THA, QUEUEList_HOR, QUEUEList_WAE = [], [], []
speedList_THA_MID, speedList_HOR_MID, speedList_WAE_MID = [], [], []
speedList_THA_RAMP, speedList_HOR_RAMP, speedList_WAE_RAMP = [], [], []
meteringRateList_THA, meteringRateList_HOR, meteringRateList_WAE = [], [], []
heroAdjustmentList = []

for step in range(4500):
\ttraci.simulationStep()
\t
\tif step > 1000 and step % STEP_INTERVAL == 0:
\t\t# Get occupancies at mainline sensors (before merge)
\t\tocc_THA_0 = traci.inductionloop.getLastIntervalOccupancy("SENS_A3_THA_MID0")
\t\tocc_THA_1 = traci.inductionloop.getLastIntervalOccupancy("SENS_A3_THA_MID1")
\t\tocc_HOR_0 = traci.inductionloop.getLastIntervalOccupancy("SENS_A3_HOR_MID0")
\t\tocc_HOR_1 = traci.inductionloop.getLastIntervalOccupancy("SENS_A3_HOR_MID1")
\t\tocc_WAE_0 = traci.inductionloop.getLastIntervalOccupancy("SENS_A3_WAE_MID0")
\t\tocc_WAE_1 = traci.inductionloop.getLastIntervalOccupancy("SENS_A3_WAE_MID1")
\t\t
\t\tocc_THA = occ_THA_0 + occ_THA_1
\t\tocc_HOR = occ_HOR_0 + occ_HOR_1
\t\tocc_WAE = occ_WAE_0 + occ_WAE_1
\t\toccList_THA.append(occ_THA)
\t\toccList_HOR.append(occ_HOR)
\t\toccList_WAE.append(occ_WAE)
\t\t
\t\t# Get queue lengths at ramps
\t\tVEH_THA = traci.lanearea.getLastStepVehicleIDs("SENS_E_THA")
\t\tVEH_HOR = traci.lanearea.getLastStepVehicleIDs("SENS_E_HOR")
\t\tVEH_WAE = traci.lanearea.getLastStepVehicleIDs("SENS_E_WAE")
\t\t
\t\tQUEUE_THA = sum(1 for veh_id in VEH_THA if traci.vehicle.getSpeed(veh_id) < 0.01)
\t\tQUEUE_HOR = sum(1 for veh_id in VEH_HOR if traci.vehicle.getSpeed(veh_id) < 0.01)
\t\tQUEUE_WAE = sum(1 for veh_id in VEH_WAE if traci.vehicle.getSpeed(veh_id) < 0.01)
\t\t
\t\tQUEUEList_THA.append(QUEUE_THA)
\t\tQUEUEList_HOR.append(QUEUE_HOR)
\t\tQUEUEList_WAE.append(QUEUE_WAE)
\t\tnumVEHList_THA.append(len(VEH_THA))
\t\tnumVEHList_HOR.append(len(VEH_HOR))
\t\tnumVEHList_WAE.append(len(VEH_WAE))
\t\t
\t\t# Apply ALINEA control
\t\tq_rate_THA, metering_rate_THA = control_ALINEA(q_rate_THA, occ_THA, QUEUE_THA, QUEUE_MAX_LENGTH_RAMP_THA)
\t\tq_rate_HOR, metering_rate_HOR = control_ALINEA(q_rate_HOR, occ_HOR, QUEUE_HOR, QUEUE_MAX_LENGTH_RAMP_HOR)
\t\tq_rate_WAE, metering_rate_WAE = control_ALINEA(q_rate_WAE, occ_WAE, QUEUE_WAE, QUEUE_MAX_LENGTH_RAMP_WAE)
\t\t
\t\t# Apply HERO coordination
\t\tmetering_rates_alinea = {{'THA': metering_rate_THA, 'HOR': metering_rate_HOR, 'WAE': metering_rate_WAE}}
\t\tqueue_lengths = {{'THA': QUEUE_THA, 'HOR': QUEUE_HOR, 'WAE': QUEUE_WAE}}
\t\tmetering_rates_hero = apply_HERO_coordination(metering_rates_alinea, queue_lengths)
\t\t
\t\tmetering_rate_THA = metering_rates_hero['THA']
\t\tmetering_rate_HOR = metering_rates_hero['HOR']
\t\tmetering_rate_WAE = metering_rates_hero['WAE']
\t\t
\t\tmeteringRateList_THA.append(metering_rate_THA)
\t\tmeteringRateList_HOR.append(metering_rate_HOR)
\t\tmeteringRateList_WAE.append(metering_rate_WAE)
\t\t
\t\t# Track if HERO made adjustments
\t\thero_active = (metering_rates_hero != metering_rates_alinea)
\t\theroAdjustmentList.append(1 if hero_active else 0)
\t\t
\t\t# Set traffic light phases
\t\tgreen_time_THA = int(metering_rate_THA * SIGNAL_CYCLE_DURATION)
\t\tgreen_time_HOR = int(metering_rate_HOR * SIGNAL_CYCLE_DURATION)
\t\tgreen_time_WAE = int(metering_rate_WAE * SIGNAL_CYCLE_DURATION)
\t\t
\t\ttraci.trafficlight.setPhaseDuration(traffic_light_THA, green_time_THA)
\t\ttraci.trafficlight.setPhaseDuration(traffic_light_HOR, green_time_HOR)
\t\ttraci.trafficlight.setPhaseDuration(traffic_light_WAE, green_time_WAE)
\t\t
\t\t# Get speeds
\t\tspeed_THA_0 = traci.inductionloop.getLastIntervalMeanSpeed("SENS_A3_THA_MID0")
\t\tspeed_THA_1 = traci.inductionloop.getLastIntervalMeanSpeed("SENS_A3_THA_MID1")
\t\tspeed_HOR_0 = traci.inductionloop.getLastIntervalMeanSpeed("SENS_A3_HOR_MID0")
\t\tspeed_HOR_1 = traci.inductionloop.getLastIntervalMeanSpeed("SENS_A3_HOR_MID1")
\t\tspeed_WAE_0 = traci.inductionloop.getLastIntervalMeanSpeed("SENS_A3_WAE_MID0")
\t\tspeed_WAE_1 = traci.inductionloop.getLastIntervalMeanSpeed("SENS_A3_WAE_MID1")
\t\t
\t\tspeedList_THA_MID.append((speed_THA_0 + speed_THA_1) / 2)
\t\tspeedList_HOR_MID.append((speed_HOR_0 + speed_HOR_1) / 2)
\t\tspeedList_WAE_MID.append((speed_WAE_0 + speed_WAE_1) / 2)
\t\t
\t\tavg_speed_ramp_THA = np.mean([traci.vehicle.getSpeed(veh_id) for veh_id in VEH_THA]) if len(VEH_THA) > 0 else 0
\t\tavg_speed_ramp_HOR = np.mean([traci.vehicle.getSpeed(veh_id) for veh_id in VEH_HOR]) if len(VEH_HOR) > 0 else 0
\t\tavg_speed_ramp_WAE = np.mean([traci.vehicle.getSpeed(veh_id) for veh_id in VEH_WAE]) if len(VEH_WAE) > 0 else 0
\t\t
\t\tspeedList_THA_RAMP.append(avg_speed_ramp_THA)
\t\tspeedList_HOR_RAMP.append(avg_speed_ramp_HOR)
\t\tspeedList_WAE_RAMP.append(avg_speed_ramp_WAE)
\t
\ttime.sleep(0)

traci.close()

#%%
# ==========================
# SUMMARY STATISTICS
# ==========================
print("=" * 60)
print("SCENARIO: {scenario_desc} - SITUATION 2 (ALINEA + HERO)")
print("=" * 60)
print(f"Average Occupancy THA: {{np.mean(occList_THA):.2f}}%")
print(f"Average Occupancy HOR: {{np.mean(occList_HOR):.2f}}%")
print(f"Average Occupancy WAE: {{np.mean(occList_WAE):.2f}}%")
print(f"Average Queue THA: {{np.mean(QUEUEList_THA):.2f}} vehicles")
print(f"Average Queue HOR: {{np.mean(QUEUEList_HOR):.2f}} vehicles")
print(f"Average Queue WAE: {{np.mean(QUEUEList_WAE):.2f}} vehicles")
print(f"Average Metering Rate THA: {{np.mean(meteringRateList_THA):.2f}}")
print(f"Average Metering Rate HOR: {{np.mean(meteringRateList_HOR):.2f}}")
print(f"Average Metering Rate WAE: {{np.mean(meteringRateList_WAE):.2f}}")
print(f"HERO Coordination Active: {{np.sum(heroAdjustmentList)}}/{{len(heroAdjustmentList)}} intervals ({{np.mean(heroAdjustmentList)*100:.1f}}%)")
print("=" * 60)

# %%
'''
    return content

def main():
    """Main function to generate all files"""
    print("=" * 60)
    print("Generating Control Strategy Files")
    print("=" * 60)

    for scenario in SCENARIOS:
        scenario_name = scenario["name"]
        scenario_desc = scenario["description"]
        vtypes_file = scenario["vTypes"]

        scenario_path = os.path.join("simulation_models", scenario_name)

        print(f"\nProcessing {scenario_name}...")

        # Create Sit1 configuration
        config_sit1_path = os.path.join(scenario_path, "Configuration_Sit1.sumocfg")
        with open(config_sit1_path, 'w') as f:
            f.write(create_configuration_sit1(scenario_name, scenario_desc, vtypes_file))
        print(f"  ✓ Created {config_sit1_path}")

        # Create Sit2 configuration
        config_sit2_path = os.path.join(scenario_path, "Configuration_Sit2.sumocfg")
        with open(config_sit2_path, 'w') as f:
            f.write(create_configuration_sit2(scenario_name, scenario_desc, vtypes_file))
        print(f"  ✓ Created {config_sit2_path}")

        # Create Sit1 RunSimulation
        runsim_sit1_path = os.path.join(scenario_path, "RunSimulation_Sit1.py")
        with open(runsim_sit1_path, 'w') as f:
            f.write(create_runsimulation_sit1(scenario_name, scenario_desc))
        print(f"  ✓ Created {runsim_sit1_path}")

        # Create Sit2 RunSimulation
        runsim_sit2_path = os.path.join(scenario_path, "RunSimulation_Sit2.py")
        with open(runsim_sit2_path, 'w') as f:
            f.write(create_runsimulation_sit2(scenario_name, scenario_desc))
        print(f"  ✓ Created {runsim_sit2_path}")

    print("\n" + "=" * 60)
    print("✅ All files generated successfully!")
    print("=" * 60)
    print("\nSummary:")
    print("  - 8 Configuration files created (Sit1, Sit2 × 4 scenarios)")
    print("  - 8 RunSimulation files created (Sit1, Sit2 × 4 scenarios)")
    print("\nNext steps:")
    print("  1. Review the generated files")
    print("  2. Test one scenario with all 3 situations (Sit0, Sit1, Sit2)")
    print("  3. Run full analysis comparing all 12 configurations")

if __name__ == "__main__":
    main()
