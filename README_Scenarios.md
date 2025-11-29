# MMSTO - A3 Highway Ramp Metering Simulation

## Overview

This project simulates the A3 Highway in Switzerland using SUMO to evaluate the effectiveness of ramp metering strategies. The simulation models a 75-minute morning peak period (07:00-08:15) with three entry ramps: Wädenswil (WAED), Horgen (HOR), and Thalwil (THA).

## Base Data

### Network Infrastructure
- **Highway Section**: A3 northbound toward Zurich
- **Entry Ramps**: 3 on-ramps (WAED, HOR, THA)
- **Exit Ramps**: 3 off-ramps (WAED, HOR, THA)
- **Network Files**: Located in each scenario folder
  - `Network.net.xml` (Scenario 0) / `Network_TL.net.xml` (Scenarios 1 & 2)
  - Infrastructure documentation in `infrastructure_data/`

### Traffic Demand (2040 Projection)
All scenarios use identical demand: **6,620 vehicles/hour**

| Route       | Cars  | Trucks | Total | Description |
|-------------|-------|--------|-------|-------------|
| A3_full     | 1,360 | 50     | 1,410 | Complete highway traverse |
| A3_to_WAED  | 770   | 20     | 790   | Exit at Wädenswil |
| A3_to_HOR   | 730   | 20     | 750   | Exit at Horgen |
| A3_to_THA   | 1,100 | 30     | 1,130 | Exit at Thalwil |
| WAED_to_A3  | 780   | 20     | 800   | Enter from Wädenswil |
| HOR_to_A3   | 690   | 20     | 710   | Enter from Horgen |
| THA_to_A3   | 1,000 | 30     | 1,030 | Enter from Thalwil |
| **TOTAL**   | **6,430** | **190** | **6,620** | |

### Vehicle Types
- **5 Aggression Levels**: `aggr1` (cautious) to `aggr5` (aggressive)
- **Distribution**: 20% per aggression level (equal distribution)
- **Vehicle Classes**: Cars and trucks for each aggression level
- **Behavioral Parameters**: Defined in `vTypes.xml`
  - Speed factors, gap acceptance, lane-changing behavior
  - Impatience, cooperation, assertiveness levels

### Sensors (Detectors)
- **Mainline Sensors**: Measure occupancy on highway lanes
  - `SENS_A3_[LOCATION]_MID0/MID1` - Loop detectors for ALINEA algorithm
  - `SENS_A3_[LOCATION]_N0/N1` - Additional monitoring
- **Ramp Sensors**: Monitor queue lengths
  - `SENS_E_[LOCATION]` - Lane area detectors on entry ramps

## Scenarios

### Scenario 0: Base (No Ramp Metering)
- **Control Strategy**: None - free flow operation
- **Purpose**: Baseline for comparison
- **Network**: Standard network without traffic light signals
- **Implementation**: Simple simulation loop without control algorithms
- **Expected Behavior**: Natural congestion patterns, potential bottlenecks at merge points

### Scenario 1: ALINEA Ramp Metering
- **Control Strategy**: ALINEA (local feedback control)
- **Purpose**: Test individual ramp metering effectiveness
- **Network**: Includes traffic lights at all three ramps (`RM_THA`, `RM_HOR`, `RM_WAED`)
- **Implementation**: Independent control for each ramp

#### How ALINEA Works in Scenario 1:
1. **Measurement**: Every 30 seconds, loop detectors measure occupancy downstream of each ramp
2. **Control Algorithm**: 
   ```
   q(k+1) = q(k) + K_R × (O_target - O_measured)
   ```
   - `q(k)`: Current ramp flow rate (veh/h)
   - `K_R`: Regulator gain (5/60 veh/sec)
   - `O_target`: Target occupancy (20%)
   - `O_measured`: Actual measured occupancy
3. **Metering Rate**: Flow rate converted to traffic light green time
   - `metering_rate = (q × 2.0 sec/veh) / 30 sec cycle`
   - Discretized to 0.1 intervals, bounded [0, 1]
4. **Queue Override**: If ramp queue exceeds threshold, metering_rate = 1.0 (green)
   - THA: 30 vehicles max
   - HOR: 30 vehicles max
   - WAE: 30 vehicles max

**Key Feature**: Each ramp operates independently based only on its own downstream conditions.

### Scenario 2: ALINEA + HERO Coordination
- **Control Strategy**: ALINEA with HERO (hierarchical coordination)
- **Purpose**: Test coordinated ramp metering with upstream/downstream cooperation
- **Network**: Same as Scenario 1 with traffic light controls
- **Implementation**: ALINEA as base + HERO coordination layer

#### How ALINEA + HERO Works in Scenario 2:
**Phase 1 - ALINEA (same as Scenario 1)**: Each ramp calculates its local metering rate

**Phase 2 - HERO Coordination**:
1. **Activation Condition**: Only activates when bottleneck occupancy > 20%
2. **Cascade Logic** (downstream to upstream):
   - **Level 1**: If THA queue > 10 vehicles → HOR becomes "slave" (restricted to min rate 0.2)
   - **Level 2**: If (THA + HOR queues) > 20 vehicles → WAE becomes "slave" (restricted to min rate 0.2)
3. **Queue Protection**: HERO cannot override flush-out condition
   - If any ramp queue > `QUEUE_MAX_LENGTH`, that ramp stays at metering_rate = 1.0
   - THA: 20 vehicles max
   - HOR: 30 vehicles max
   - WAE: 10 vehicles max

**Key Feature**: Upstream ramps reduce inflow to help downstream ramps clear congestion, creating coordinated behavior across the system.

#### HERO Parameters:
- `HERO_QUEUE_THRESHOLD1 = 10 veh`: Activates first level coordination
- `HERO_QUEUE_THRESHOLD2 = 20 veh`: Activates second level coordination  
- `HERO_MIN_RATE = 0.2`: Minimum metering rate for slave ramps (20% green time)

## Directory Structure

```
MMSTO/
├── infrastructure_data/
│   ├── network_edge_list.xlsx
│   ├── network_infrastructure_list.xlsx
│   └── network_with_junctions_and_tl.pdf
├── simulation_models/
│   ├── scenario_0_Base/
│   │   ├── Configuration_Sit0.sumocfg
│   │   ├── Demand_2040.rou.xml          [Traffic flows]
│   │   ├── Network.net.xml              [Road network - no signals]
│   │   ├── routes.rou.xml               [7 route definitions]
│   │   ├── vTypes.xml                   [10 vehicle types]
│   │   ├── detectors.add.xml            [Sensor positions]
│   │   └── RunSimulation_Sit0.py        [No control]
│   ├── scenario_1_ALINEA/
│   │   ├── Configuration_Sit1.sumocfg
│   │   ├── Demand_2040.rou.xml
│   │   ├── Network_TL.net.xml           [Road network with traffic lights]
│   │   ├── routes.rou.xml
│   │   ├── vTypes.xml
│   │   ├── detectors.add.xml
│   │   └── RunSimulation_Sit1.py        [ALINEA control]
│   └── scenario_2_ALINEA+HERO/
│       ├── Configuration_Sit2.sumocfg
│       ├── Demand_2040.rou.xml
│       ├── Network_TL.net.xml
│       ├── routes.rou.xml
│       ├── vTypes.xml
│       ├── detectors.add.xml
│       └── RunSimulation_Sit2.py        [ALINEA + HERO coordination]
├── simulation_output/
│   ├── scenario_0_Base/
│   │   ├── output_fcd_sit0.xml
│   │   ├── output_emissions_sit0.xml
│   │   ├── output_summary_sit0.xml
│   │   ├── output_tripinfo_sit0.xml
│   │   ├── output_detectors/
│   │   └── plots_sit0/
│   ├── scenario_1_ALINEA/
│   │   └── [same structure]
│   ├── scenario_2_ALINEA+HERO/
│   │   └── [same structure]
│   └── comparison_plots/
├── PostProcess_Detectors.py             [Detector analysis]
├── PostProcess_FCD.py                   [Vehicle trajectory analysis]
├── PostProcess_Compare_Scenarios.py     [Comparison plots]
└── plotting_infrastructure.py           [Shared plotting functions]
```

## Running Simulations

### Prerequisites
- SUMO installed at: `C:\Program Files (x86)\Eclipse\Sumo\`
- Python 3.x with TraCI library
- Required packages: `numpy`, `matplotlib`

### To run a specific scenario:

1. Navigate to the scenario folder:
   ```bash
   cd simulation_models/scenario_X_[NAME]/
   ```

2. Run the Python simulation script:
   ```bash
   # Scenario 0 (Base - No Control)
   python RunSimulation_Sit0.py

   # Scenario 1 (ALINEA)
   python RunSimulation_Sit1.py

   # Scenario 2 (ALINEA + HERO)
   python RunSimulation_Sit2.py
   ```

3. Output files are automatically generated in:
   ```
   ../../simulation_output/scenario_X_[NAME]/
   ```

### Simulation Parameters:
- **Duration**: 4,500 seconds (75 minutes)
- **Time Step**: 1 second
- **Control Update Interval**: 30 seconds (for Scenarios 1 & 2)
- **Warmup Period**: 240 seconds (4 minutes) before control activation

### Output Files Generated:
- `output_fcd_sitX.xml` - Full Car Data (vehicle trajectories, speeds, positions)
- `output_emissions_sitX.xml` - Emissions data (CO, CO2, NOx, fuel consumption)
- `output_summary_sitX.xml` - Aggregated statistics per time interval
- `output_tripinfo_sitX.xml` - Individual trip information (travel times, delays)
- `output_detectors/` - Individual detector measurements (occupancy, flow, speed)

### Post-Processing Analysis:

Run the analysis scripts from the main directory:

```bash
# Analyze individual scenario detectors
python PostProcess_Detectors.py

# Analyze vehicle trajectories and speeds
python PostProcess_FCD.py

# Compare all scenarios
python PostProcess_Compare_Scenarios.py
```

Comparison plots are saved to: `simulation_output/comparison_plots/`

## Expected Results & Analysis

### Key Performance Indicators:

1. **Mainline Flow**:
   - Total throughput on A3 highway
   - Impact of ramp metering on highway capacity utilization

2. **Travel Time**:
   - Average travel time for through traffic
   - Delay introduced by ramp metering vs. benefit of smoother flow

3. **Ramp Queue Lengths**:
   - Maximum and average queue lengths at THA, HOR, WAE
   - Queue management effectiveness

4. **Occupancy**:
   - Highway lane occupancy patterns
   - Success of maintaining target occupancy (20%)

5. **Speed Distributions**:
   - Average speeds on mainline
   - Speed variance (indicator of congestion)

### Expected Differences Between Scenarios:

**Scenario 0 (Base)**:
- Natural congestion at merge points
- Potential capacity breakdown during peak demand
- High speed variance during congestion
- No artificial delay on ramps

**Scenario 1 (ALINEA)**:
- Regulated mainline occupancy near 20% target
- Smoother mainline flow
- Ramp queues form but are managed locally
- Each ramp responds only to its downstream conditions
- Possible suboptimal coordination between ramps

**Scenario 2 (ALINEA + HERO)**:
- Coordinated system-wide response
- Upstream ramps "help" downstream congested areas
- Potentially longer queues at upstream ramps (WAE)
- Better overall system performance
- Cascade effect: congestion triggers sequential ramp restrictions

### Comparative Analysis:
- ALINEA vs. Base: Local control effectiveness
- ALINEA+HERO vs. ALINEA: Coordination benefits
- Trade-off: Mainline efficiency vs. ramp delay

## Technical Details

### Control Algorithm Implementation

**ALINEA Feedback Loop** (Scenarios 1 & 2):
```python
q_rate = q_previous_rate + K_R * (OCCUPANCY_TARGET - occupancy_measured)
q_bounded = min(Q_MAX, max(q_rate, Q_MIN))
metering_rate = (q_bounded * VEHICLE_AV_ACC_TIME) / SIGNAL_CYCLE_DURATION
```

**HERO Coordination** (Scenario 2 only):
```python
# Level 1: THA queue high → restrict HOR
if QUEUEstep_THA > 10:
    metering_rate_HOR = min(metering_rate_HOR, 0.2)

# Level 2: THA+HOR queues high → restrict WAE  
if (QUEUEstep_THA + QUEUEstep_HOR) > 20:
    metering_rate_WAE = min(metering_rate_WAE, 0.2)
```

### Sensor Configuration:
- **Loop Detectors**: Mainline occupancy measurement
  - Positioned downstream of each merge
  - Average of 2 lanes (MID0, MID1)
- **Lane Area Detectors**: Ramp queue monitoring
  - Cover entire ramp length
  - Count vehicles with speed < 0.01 m/s as queued

## Project Context

This simulation is part of a Multi-Modal Sustainable Transport Optimization (MMSTO) study evaluating ramp metering strategies for the A3 highway corridor in Switzerland. The goal is to quantify the benefits of coordinated ramp control in managing peak-period congestion with 2040 projected demand levels.

## Notes

- All simulations use identical demand (6,620 veh/h)
- All simulations use identical vehicle behavior distribution (20% per aggression level)
- Network geometry is identical except for traffic light additions in Scenarios 1 & 2
- Control parameters (K_R, occupancy target) are consistent across controlled scenarios
- Queue override thresholds vary slightly between scenarios to reflect calibration
