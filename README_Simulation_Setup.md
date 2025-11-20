# A3 Highway Simulation Setup - User Guide

## Overview
This simulation models the A3 Highway with three entry/exit ramps (Thalwil, Horgen, Wädenswil) using SUMO (Simulation of Urban MObility) with ALINEA ramp metering control.

## Files Structure

### Configuration Files
- **00_asarris_Configuration_Sit1.sumocfg** - Main SUMO configuration file
- **Network.net.xml** - Network definition (A3 Highway with 3 ramps)
- **viewsettings_asarris_eth.xml** - GUI visualization settings with aerial imagery

### Route & Demand Files
- **vTypes.xml** - Vehicle type definitions (20 types with 5 aggressiveness levels)
- **routes.rou.xml** - Route definitions (7 routes for highway and ramps)
- **demand.rou.xml** - Traffic demand with behavioral distribution (70 flows)

### Control & Monitoring
- **detectors.add.xml** - Induction loop and lane area detector definitions
- **00_asarris_RunSimulation_Sit1.py** - Python/TraCI script with ALINEA control

## Vehicle Types & Aggressiveness Levels

The simulation includes **5 levels of driver aggressiveness** (20% distribution each):

### Highway Vehicles
- **car_aggr1 to car_aggr5** - Cars on mainline (green → yellow → red colors)
- **truck_aggr1 to truck_aggr5** - Trucks on mainline

### Ramp Vehicles (More Aggressive Merging)
- **ramp_car_aggr1 to ramp_car_aggr5** - Cars entering from ramps
- **ramp_truck_aggr1 to ramp_truck_aggr5** - Trucks entering from ramps

**Key Behavioral Parameters:**
- **Earlier merging**: `lcStrategic=100` for ramp vehicles
- **Smaller gap acceptance**: `minGapLat` down to 0.02m, `tau` down to 0.4s
- **Pushy behavior**: `lcPushy` up to 6.3, `lcCooperative` near 0
- **Yellow light running**: `jmDriveAfterYellowTime` up to 2.0s

## Running the Simulation

### Option 1: Python Script with ALINEA Control
```bash
python 00_asarris_RunSimulation_Sit1.py
```

This will:
- Launch SUMO-GUI with the configuration
- Run ALINEA ramp metering control for all 3 ramps
- Monitor mainline occupancy and ramp queues
- Adjust traffic light timings in real-time
- Generate visualization plots after simulation

### Option 2: SUMO-GUI Directly
```bash
sumo-gui -c 00_asarris_Configuration_Sit1.sumocfg
```

This runs the simulation without TraCI control (traffic lights remain static).

## ALINEA Ramp Metering Control

### Control Parameters
- **Target Occupancy**: 30% on mainline
- **Max Queue Length**: 5 vehicles on ramps
- **Update Interval**: Every 30 simulation steps
- **Flow Rate Range**: 0-1800 veh/h
- **K_R**: 5/60 veh/sec (regulator gain)
- **K_QUEUE**: 0.2 veh/sec (queue control gain)

### Controlled Ramps
1. **RM_THA** - Thalwil ramp metering
2. **RM_HOR** - Horgen ramp metering
3. **RM_WAED** - Wädenswil ramp metering

## Visualization & Output

### Real-time Visualization (During Simulation)
- SUMO-GUI with aerial imagery background
- Color-coded vehicles by aggressiveness (green=calm, red=aggressive)
- Traffic light states at ramp meters

### Post-simulation Plots (Matplotlib)
After running the Python script, you'll get **3 plots** (one for each ramp):
1. **Mainline occupancy** (%)
2. **Number of vehicles on ramp** (#)
3. **Queue length** (# standing vehicles)
4. **Red duration evolution** (seconds)

### Output Files
The simulation generates the following output files:
- **output_fcd.xml** - Full vehicle trajectories (position, speed, acceleration)
- **output_emissions.xml** - Emissions (CO2, NOx, fuel consumption)
- **output_summary.xml** - Aggregated statistics per time step
- **output_tripinfo.xml** - Individual trip statistics (travel time, delays)
- **output_detectors/** - Detector measurements

## Demand Summary

**Total Traffic: 6,620 vehicles/hour**

### Highway Through-traffic (4,080 veh/h)
- A3_full: 1,410 veh/h
- A3_to_WAED: 790 veh/h
- A3_to_HOR: 750 veh/h
- A3_to_THA: 1,130 veh/h

### Entry Ramps (2,540 veh/h)
- WAED_to_A3: 800 veh/h
- HOR_to_A3: 710 veh/h
- THA_to_A3: 1,030 veh/h

All flows are split into 5 aggressiveness levels (20% each).

## Simulation Scenario

**This is ONE INTEGRATED SIMULATION with behavioral heterogeneity:**
- 20% of drivers at each aggressiveness level (1-5)
- Realistic interaction between cautious and aggressive drivers
- Proper merging dynamics with different gap acceptance behaviors

**For sensitivity analysis**, you can create separate scenarios by modifying demand.rou.xml to use only specific aggressiveness levels (e.g., all aggr5 for "all aggressive" scenario).

## Technical Details

### Simulation Parameters
- **Duration**: 3,600 seconds (1 hour)
- **Time step**: 1 second
- **Lane change model**: SL2015 (sublane model)
- **Collision checking**: Enabled at junctions
- **Teleportation**: Disabled (time-to-teleport = -1)

### Requirements
- SUMO version 1.24.0 or higher
- Python 3.x with packages:
  - traci (comes with SUMO)
  - numpy
  - matplotlib

### SUMO Installation Path
Update line 20 in the Python script if your SUMO installation is different:
```python
sumoBinary = r"C:\Program Files (x86)\Eclipse\Sumo\bin\sumo-gui.exe"
```

## Troubleshooting

### Aerial Images Not Showing
- Check paths in viewsettings_asarris_eth.xml
- Ensure image files exist in the specified directory
- Images should be at: `C:\Users\asarris\ETH Zurich\...\Base Data\`

### TraCI Connection Error
- Ensure SUMO_HOME environment variable is set
- Check that config file path is correct in Python script
- Make sure no other SUMO instance is running

### No Vehicles Appearing
- Verify that vTypes.xml is in the same directory
- Check that routes.rou.xml includes the vTypes.xml file
- Ensure demand.rou.xml includes routes.rou.xml

## Further Analysis

### Analyzing Output Files
Use SUMO tools to analyze output:
```bash
# Plot emissions over time
python $SUMO_HOME/tools/xml/xml2csv.py output_emissions.xml -o emissions.csv

# Analyze trip statistics
python $SUMO_HOME/tools/xml/xml2csv.py output_tripinfo.xml -o tripinfo.csv
```

### Custom TraCI Analysis
Extend the Python script to add:
- Speed profiles along the highway
- Time-space diagrams
- Shock wave analysis
- Ramp metering efficiency metrics

## Contact & Credits
- Author: asarris
- Course: Microscopic Modelling and Simulation of Traffic Operations - ETH Zurich
- Date: November 2025
