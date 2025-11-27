# MMSTO Demand Scenarios

## Overview

This project contains 4 demand scenarios for A3 Highway SUMO simulation testing congestion under different behavioral distributions. All scenarios maintain the same total demand (6,620 veh/hour) but vary the aggression level distribution.

## Scenarios

### Scenario 1: 2040-None (BASELINE)
- **Distribution**: N/A (uses SUMO default vehicle types)
- **Purpose**: Neutral baseline behavior without custom behavioral parameters
- **Vehicle Types**: 2 types (`default_car`, `default_truck`)
- **Total Flows**: 14 (7 routes × 2 vehicle types)

### Scenario 2: 2040-Mixed
- **Distribution**: 20% / 20% / 20% / 20% / 20% (aggr1/aggr2/aggr3/aggr4/aggr5)
- **Purpose**: Evenly distributed aggression - realistic mixed traffic
- **Vehicle Types**: 10 types (car_aggr1-5, truck_aggr1-5)
- **Total Flows**: 70 (7 routes × 5 aggression levels × 2 vehicle types)

### Scenario 3: 2040-Mid
- **Distribution**: 10% / 30% / 20% / 30% / 10% (aggr1/aggr2/aggr3/aggr4/aggr5)
- **Purpose**: Moderate aggression dominance - fewer extremes
- **Vehicle Types**: 10 types (car_aggr1-5, truck_aggr1-5)
- **Total Flows**: 70 (7 routes × 5 aggression levels × 2 vehicle types)

### Scenario 4: 2040-Peace
- **Distribution**: 40% / 30% / 15% / 10% / 5% (aggr1/aggr2/aggr3/aggr4/aggr5)
- **Purpose**: Peaceful conservative driving - safety analysis
- **Vehicle Types**: 10 types (car_aggr1-5, truck_aggr1-5)
- **Total Flows**: 70 (7 routes × 5 aggression levels × 2 vehicle types)

## Total Demand (Same for All Scenarios)

| Route       | Cars  | Trucks | Total |
|-------------|-------|--------|-------|
| A3_full     | 1,360 | 50     | 1,410 |
| A3_to_WAED  | 770   | 20     | 790   |
| A3_to_HOR   | 730   | 20     | 750   |
| A3_to_THA   | 1,100 | 30     | 1,130 |
| WAED_to_A3  | 780   | 20     | 800   |
| HOR_to_A3   | 690   | 20     | 710   |
| THA_to_A3   | 1,000 | 30     | 1,030 |
| **TOTAL**   | **6,430** | **190** | **6,620** |

## Directory Structure

```
MMSTO/
├── shared_simulation_files/
│   ├── Network.net.xml                   [Shared network]
│   ├── vTypes.xml                        [10 aggression types]
│   ├── vTypes_default.xml                [SUMO defaults for Scenario 1]
│   ├── routes.rou.xml                    [7 route definitions]
│   ├── detectors.add.xml                 [Sensor positions]
│   └── viewsettings_asarris_personal.xml [GUI settings]
├── simulation_models/
│   ├── scenario_1_2040-None/
│   │   ├── Configuration.sumocfg
│   │   ├── Demand.rou.xml
│   │   └── RunSimulation.py
│   ├── scenario_2_2040-Mixed/
│   │   ├── Configuration.sumocfg
│   │   ├── Demand.rou.xml
│   │   └── RunSimulation.py
│   ├── scenario_3_2040-Mid/
│   │   ├── Configuration.sumocfg
│   │   ├── Demand.rou.xml
│   │   └── RunSimulation.py
│   └── scenario_4_2040-Peace/
│       ├── Configuration.sumocfg
│       ├── Demand.rou.xml
│       └── RunSimulation.py
└── simulation_output/
    ├── scenario_1_2040-None/
    ├── scenario_2_2040-Mixed/
    ├── scenario_3_2040-Mid/
    └── scenario_4_2040-Peace/
```

## Running Simulations

### To run a specific scenario:

1. Navigate to the scenario folder:
   ```bash
   cd simulation_models/scenario_X_YYYY/
   ```

2. Run the simulation:
   ```bash
   python RunSimulation.py
   ```

3. Output files will be generated in:
   ```
   ../../simulation_output/scenario_X_YYYY/
   ```

### Output Files Generated:
- `output_fcd.xml` - Full Car Data (vehicle trajectories)
- `output_emissions.xml` - Emissions data
- `output_summary.xml` - Simulation summary statistics
- `output_tripinfo.xml` - Trip information for all vehicles

### Plots Generated:
Plots are saved to `../../plots/scenario_X_YYYY/`:
- `THA_[scenario].png` - Thalwil metrics
- `HOR_[scenario].png` - Horgen metrics
- `WAE_[scenario].png` - Wädenswil metrics

## Key Differences Between Scenarios

- **Scenario 1 (None)**: Neutral baseline, no aggressive/passive extremes, uses SUMO default behaviors
- **Scenario 2 (Mixed)**: Balanced mix representing realistic diverse traffic
- **Scenario 3 (Mid)**: Emphasis on moderate aggression (aggr2/aggr4 at 30% each)
- **Scenario 4 (Peace)**: Heavily weighted toward cautious driving (aggr1 at 40%)

## Expected Analysis Outcomes

Congestion analysis should show differences in:
- Average travel times
- Lane changing frequency
- Speed distributions
- Emission patterns
- Traffic flow stability
- Queue formation at ramps

## Notes

- All configuration files use relative paths from the scenario folder
- Shared files (Network, routes, vTypes, detectors) are referenced from `../../shared_simulation_files/`
- Simulation duration: 4,500 seconds (75 minutes)
- Peak period: Morning peak (07:00-08:15)
- Time step: 1 second
