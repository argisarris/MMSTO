#%%
# ==========================
# PYTHON IMPORTS
# ==========================
import os
import sys
import traci
import time

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
sumoConfigFile = os.path.join(script_dir, "Configuration_Sit0.sumocfg")
sumoCmd = [sumoBinary, "-c", sumoConfigFile, "--start", "--quit-on-end", "--time-to-teleport", "-1"]
traci.start(sumoCmd)

# ==========================
# Simulation (Scenario 0: 2025)
# ==========================
print("Simulation step length (DeltaT):", traci.simulation.getDeltaT(), "s")
for step in range(4500):
	traci.simulationStep()
	time.sleep(0)

traci.close()

print("Simulation completed successfully.")

# %%%
