# MMSTO - A3 Highway Ramp Metering Simulation

## Overview

This project simulates the A3 Highway in Switzerland using SUMO to evaluate the effectiveness of ramp metering strategies. The simulation models a 75-minute morning peak period (07:00-08:15) with three entry ramps: Wädenswil (WAED), Horgen (HOR), and Thalwil (THA).

The repository implements **four scenarios** to compare no control, local ramp metering (ALINEA), coordinated ramp metering (ALINEA+HERO), and extended ALINEA control to assess the impact of different ramp metering strategies on highway performance.

## Repository Structure

```
MMSTO/
├── infrastructure_data/              # Network design documentation
│   ├── network_edge_list.xlsx
│   ├── network_infrastructure_list.xlsx
│   └── network_with_junctions_and_tl.pdf
│
├── simulation_models/                # SUMO simulation files for each scenario
│   ├── scenario_0_Base/
│   ├── scenario_1_ALINEA/
│   ├── scenario_2_ALINEA+HERO/
│   └── scenario_3_ALINEA_long/
│
├── simulation_output/                # Generated output files and plots
│   ├── scenario_0_Base/
│   ├── scenario_1_ALINEA/
│   ├── scenario_2_ALINEA+HERO/
│   ├── scenario_3_ALINEA_long/
│   └── comparison_plots/
│
├── PostProcess_Detectors.py          # Analyze detector measurements
├── PostProcess_FCD.py                # Analyze vehicle trajectories
├── PostProcess_Compare_Scenarios.py  # Generate comparison plots
├── plotting_infrastructure.py        # Shared plotting utilities
└── README_Scenarios.md               # This file
```

### Simulation Model Files (per scenario):

Each scenario folder contains:
- **Configuration_SitX.sumocfg**: SUMO configuration file
- **Network_TL.net.xml** or **Network.net.xml**: Road network (with/without traffic lights)
- **Demand_2040.rou.xml**: Traffic demand flows
- **routes.rou.xml**: Route definitions (7 routes)
- **vTypes.xml**: Vehicle type definitions (10 types: 5 aggression levels × 2 vehicle classes)
- **detectors.add.xml**: Sensor placement configuration
- **RunSimulation_SitX.py**: Python script implementing control logic and running simulation

### Output Files (per scenario):

- **output_fcd_sitX.xml**: Full Car Data (vehicle positions, speeds, lanes)
- **output_emissions_sitX.xml**: Emissions data (CO, CO2, NOx, fuel)
- **output_summary_sitX.xml**: Aggregated statistics per time step
- **output_tripinfo_sitX.xml**: Individual trip data (travel times, delays)
- **output_detectors/**: Directory with individual detector XML files
- **plots_sitX/**: Analysis plots for single scenario

## Base Data

### Network Infrastructure
- **Highway Section**: A3 northbound toward Zurich
- **Entry Ramps**: 3 on-ramps (WAED, HOR, THA)
- **Exit Ramps**: 3 off-ramps (WAED, HOR, THA)
- **Mainline Lanes**: 2 lanes throughout
- **Network Files**: 
  - Scenario 0: `Network.net.xml` (no traffic lights)
  - Scenarios 1-3: `Network_TL.net.xml` (includes ramp metering signals)

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
- **Mainline Loop Detectors**: Measure occupancy on highway lanes
  - `SENS_A3_[LOCATION]_MID0/MID1`: Positioned just before merge points (used by ALINEA)
  - `SENS_A3_[LOCATION]_N0/N1`: Positioned after merge points (monitoring)
- **Ramp Lane Area Detectors**: Monitor queue lengths
  - `SENS_E_[LOCATION]`: Cover entire ramp length to count queued vehicles

## Scenarios Overview

| Scenario | Name | Control Strategy | Purpose |
|----------|------|------------------|---------|
| 0 | Base | None | Baseline (no ramp metering) |
| 1 | ALINEA | Local feedback | Individual ramp control |
| 2 | ALINEA+HERO | Coordinated | System-wide optimization |
| 3 | ALINEA_long | Extended local feedback | Long-duration ALINEA test |

---

## Scenario 0: Base (No Ramp Metering)

### Control Strategy
**None** - Free flow operation without any ramp metering control.

### Network Configuration
- Standard network without traffic light signals
- Ramps merge freely into mainline traffic
- No artificial delay imposed on entering vehicles

### Implementation
- Simple simulation loop without control algorithms
- No TraCI-based traffic light manipulation
- Pure capacity-based flow dynamics

### Purpose
Establish baseline performance metrics for comparison:
- Natural congestion patterns
- Uncontrolled bottleneck formation at merge points
- Maximum system throughput without intervention
- Baseline travel times and delays

### Expected Behavior
- Congestion develops naturally when demand exceeds capacity
- Potential breakdown at merge points during peak periods
- High speed variance during congestion
- No ramp queues (vehicles merge immediately or as soon as gap allows)

---

## Scenario 1: ALINEA (Local Ramp Metering)

### Control Strategy
**ALINEA** - Asservissement LINéaire d'Entrée Autoroutière (Linear Feedback Control for Highway On-Ramps)

### How ALINEA Works

ALINEA is a **local feedback control algorithm** where each ramp operates **independently** based only on its own downstream mainline conditions.

#### Control Loop (Every 30 seconds):

1. **Measurement Phase**:
   - Loop detectors (`SENS_A3_[LOC]_MID0`, `SENS_A3_[LOC]_MID1`) measure occupancy downstream of each ramp
   - Average occupancy across both lanes is calculated
   - Ramp queue length is measured using lane area detector (`SENS_E_[LOC]`)

2. **ALINEA Algorithm**:
   ```
   q(k+1) = q(k) + K_R × (O_target - O_measured)
   ```
   Where:
   - `q(k)`: Current ramp flow rate (veh/h)
   - `q(k+1)`: Next ramp flow rate (veh/h)
   - `K_R`: Regulator gain = 5/60 veh/sec = 300 veh/h per unit occupancy error
   - `O_target`: Target occupancy = 20% (0.20)
   - `O_measured`: Actual measured occupancy (0.0 to 1.0)
   
3. **Flow Rate Bounds**:
   ```
   q_bounded = min(Q_MAX, max(q_rate, Q_MIN))
   ```
   - `Q_MIN = 300 veh/h`: Minimum flow (maintain minimum service)
   - `Q_MAX = 1200 veh/h`: Maximum flow (ramp capacity)

4. **Metering Rate Calculation**:
   ```
   metering_rate = (q_bounded × 2.0 sec/veh) / 30 sec cycle
   green_time = metering_rate × 30 sec
   ```
   - Metering rate bounded: [0.0, 1.0]
   - Discretized to 0.1 intervals (0.0, 0.1, 0.2, ..., 1.0)
   - Traffic light cycle = 30 seconds

5. **Queue Override (Safety Mechanism)**:
   ```
   if queue_length > QUEUE_MAX_LENGTH:
       metering_rate = 1.0  # Full green (flush queue)
   ```
   Queue thresholds:
   - **THA**: 30 vehicles
   - **HOR**: 30 vehicles
   - **WAE**: 30 vehicles

#### Key Characteristics:
- ✅ **Independent control**: Each ramp only considers its own downstream occupancy
- ✅ **Reactive**: Responds to local conditions in real-time
- ✅ **Simple**: Single feedback loop per ramp
- ❌ **No coordination**: Ramps don't communicate or cooperate
- ❌ **Potential inefficiency**: Upstream ramps may add vehicles while downstream is congested

### Network Configuration
- Network with traffic lights at all three ramps: `RM_THA`, `RM_HOR`, `RM_WAED`
- Traffic lights controlled via TraCI interface
- Each ramp has dedicated control logic

### Purpose
Test the effectiveness of **individual local ramp metering** without system-wide coordination.

---

## Scenario 2: ALINEA + HERO (Coordinated Ramp Metering)

### Control Strategy
**ALINEA with HERO** - Heuristic Ramp-metering coordination

### How ALINEA + HERO Works

HERO adds a **coordination layer** on top of ALINEA, allowing ramps to work together as a system.

#### Two-Phase Control:

**Phase 1 - Local ALINEA Control**:
- Each ramp independently calculates its metering rate using ALINEA (same as Scenario 1)
- This provides the **base control** for each ramp

**Phase 2 - HERO Coordination**:
- HERO **only activates** when the bottleneck detector shows occupancy > 20%
- Implements a **cascade restriction** from downstream to upstream

#### HERO Cascade Logic:

```
Step 1: Check THA (most downstream ramp)
if QUEUEstep_THA > HERO_QUEUE_THRESHOLD1:
    Activate Level 1 coordination
    → HOR becomes "slave" ramp
    → metering_rate_HOR = min(metering_rate_HOR, HERO_MIN_RATE)
    → HOR restricted to maximum 20% green time

Step 2: Check THA + HOR combined
if (QUEUEstep_THA + QUEUEstep_HOR) > HERO_QUEUE_THRESHOLD2:
    Activate Level 2 coordination
    → WAE becomes "slave" ramp
    → metering_rate_WAE = min(metering_rate_WAE, HERO_MIN_RATE)
    → WAE restricted to maximum 20% green time
```

#### HERO Parameters:
- `HERO_QUEUE_THRESHOLD1 = 10 vehicles`: Triggers Level 1 (restrict HOR)
- `HERO_QUEUE_THRESHOLD2 = 20 vehicles`: Triggers Level 2 (restrict WAE)
- `HERO_MIN_RATE = 0.2`: Minimum metering rate for slave ramps (20% green time)

#### Queue Protection (Critical Safety):
HERO coordination **cannot override** queue flush-out:
```
if queue_length > QUEUE_MAX_LENGTH:
    metering_rate = 1.0  # Override HERO restriction
```
Queue thresholds (different from Scenario 1):
- **THA**: 20 vehicles (tighter control)
- **HOR**: 30 vehicles
- **WAE**: 10 vehicles (more restrictive due to upstream position)

#### Key Characteristics:
- ✅ **Coordinated control**: Ramps work together as a system
- ✅ **Proactive**: Upstream ramps help downstream congestion before it spreads
- ✅ **Cascade effect**: Sequential activation of coordination levels
- ✅ **Protected**: Queue limits prevent excessive ramp delays
- ⚠️ **Trade-off**: Longer queues at upstream ramps in exchange for better mainline flow

### Network Configuration
- Same network as Scenario 1: `Network_TL.net.xml`
- Identical traffic light setup
- Enhanced control logic with coordination module

### Purpose
Evaluate **system-wide coordination benefits** over local control alone:
- Can coordination improve overall throughput?
- What is the trade-off between mainline efficiency and ramp delay?
- How does cascade control distribute congestion across ramps?

---

## Scenario 3: ALINEA_long (Extended Local Ramp Metering)

### Control Strategy
**ALINEA** - Same as Scenario 1, but with **extended simulation duration**

### How It Differs from Scenario 1
- **Identical control algorithm**: Same ALINEA implementation
- **Same parameters**: K_R, occupancy target, queue thresholds all identical
- **Extended duration**: Longer simulation time to observe long-term dynamics
- **Additional analysis**: Study control stability and performance over extended periods

### Purpose
- Test **long-term stability** of ALINEA control
- Observe **sustained congestion** patterns
- Validate control effectiveness beyond typical peak period
- Compare short-term vs. long-term performance metrics

### Network Configuration
- Identical to Scenario 1: `Network_TL.net.xml`
- Same ALINEA control parameters
- Same queue thresholds (THA: 30, HOR: 30, WAE: 30)

### Expected Insights
- Does ALINEA maintain effectiveness over longer periods?
- How do queue patterns evolve with extended demand?
- Is there control drift or instability over time?

---

## Control Algorithm Details

### ALINEA Feedback Controller (Scenarios 1, 2, 3)

```python
# Measure occupancy (average of 2 lanes)
occupancy_measured = (occupancy_lane0 + occupancy_lane1) / 2.0

# ALINEA feedback equation
q_rate = q_previous_rate + K_R * (OCCUPANCY_TARGET - occupancy_measured)

# Apply bounds
q_bounded = min(Q_MAX, max(q_rate, Q_MIN))

# Convert to metering rate (green time fraction)
metering_rate = (q_bounded * VEHICLE_AV_ACC_TIME) / SIGNAL_CYCLE_DURATION
metering_rate = round(metering_rate, 1)  # Discretize to 0.1
metering_rate = min(1.0, max(0.0, metering_rate))

# Queue override
if queue_length > QUEUE_MAX_LENGTH:
    metering_rate = 1.0

# Apply to traffic light
green_time = metering_rate * SIGNAL_CYCLE_DURATION
red_time = SIGNAL_CYCLE_DURATION - green_time
```

**Parameters**:
- `K_R = 5/60 veh/sec = 300 veh/h`: Regulator gain
- `OCCUPANCY_TARGET = 0.20`: Target 20% occupancy
- `Q_MIN = 300 veh/h`: Minimum service flow
- `Q_MAX = 1200 veh/h`: Maximum ramp capacity
- `VEHICLE_AV_ACC_TIME = 2.0 sec`: Average vehicle acceptance time
- `SIGNAL_CYCLE_DURATION = 30 sec`: Control cycle period

### HERO Coordination Layer (Scenario 2 Only)

```python
# Only activate if bottleneck is congested
if occupancy_bottleneck > 0.20:
    
    # Level 1: THA queue high → restrict HOR
    if QUEUEstep_THA > HERO_QUEUE_THRESHOLD1:
        metering_rate_HOR = min(metering_rate_HOR, HERO_MIN_RATE)
    
    # Level 2: THA+HOR queues high → restrict WAE
    if (QUEUEstep_THA + QUEUEstep_HOR) > HERO_QUEUE_THRESHOLD2:
        metering_rate_WAE = min(metering_rate_WAE, HERO_MIN_RATE)
    
    # Safety: Queue protection overrides HERO
    if queue_HOR > QUEUE_MAX_LENGTH_HOR:
        metering_rate_HOR = 1.0
    if queue_WAE > QUEUE_MAX_LENGTH_WAE:
        metering_rate_WAE = 1.0
```

**HERO Parameters**:
- `HERO_QUEUE_THRESHOLD1 = 10 veh`: Activate Level 1
- `HERO_QUEUE_THRESHOLD2 = 20 veh`: Activate Level 2
- `HERO_MIN_RATE = 0.2`: Slave ramp minimum rate (20%)

---

## Running Simulations

### Prerequisites
- SUMO installed at: `C:\Program Files (x86)\Eclipse\Sumo\`
- Python 3.x with TraCI library
- Required packages: `numpy`, `matplotlib`, `pandas`

### Execute Simulations

Navigate to the scenario folder and run:

```bash
# Scenario 0: Base (No Control)
cd simulation_models/scenario_0_Base/
python RunSimulation_Sit0.py

# Scenario 1: ALINEA
cd simulation_models/scenario_1_ALINEA/
python RunSimulation_Sit1.py

# Scenario 2: ALINEA + HERO
cd simulation_models/scenario_2_ALINEA+HERO/
python RunSimulation_Sit2.py

# Scenario 3: ALINEA_long
cd simulation_models/scenario_3_ALINEA_long/
python RunSimulation_Sit3.py
```

### Simulation Parameters
- **Duration**: 4,500 seconds (75 minutes)
- **Time Step**: 1 second
- **Control Update Interval**: 30 seconds (Scenarios 1-3)
- **Warmup Period**: 240 seconds (4 minutes) before control activation

### Output Generation
Outputs are automatically saved to:
```
simulation_output/scenario_X_[NAME]/
├── output_fcd_sitX.xml
├── output_emissions_sitX.xml
├── output_summary_sitX.xml
├── output_tripinfo_sitX.xml
└── output_detectors/
    ├── SENS_A3_THA_MID0.xml
    ├── SENS_A3_THA_MID1.xml
    ├── SENS_E_THA.xml
    └── ...
```

---

## Post-Processing Analysis

Run analysis scripts from the repository root:

### 1. Individual Scenario Analysis

```bash
# Analyze detector measurements (occupancy, flow, speed)
python PostProcess_Detectors.py
# Configure: Set SITUATION = "sit0", "sit1", "sit2", or "sit3"

# Analyze vehicle trajectories (FCD data)
python PostProcess_FCD.py
# Configure: Set SITUATION = "sit0", "sit1", "sit2", or "sit3"
```

Outputs:
- `simulation_output/scenario_X_[NAME]/plots_sitX/`
- Detector analysis plots (08-10)
- FCD analysis plots (01-06)

### 2. Comparative Analysis

```bash
# Compare all four scenarios
python PostProcess_Compare_Scenarios.py
```

Outputs:
- `simulation_output/comparison_plots/`
- Network-wide speed comparison (01)
- Vehicle count comparison (02)
- Ramp-specific comparisons (03-05)
- Summary statistics (06)

---

## Expected Results & Analysis

### Key Performance Indicators

1. **Mainline Throughput**:
   - Total vehicles passing through network
   - Highway capacity utilization
   - Impact of metering on mainline flow

2. **Travel Time**:
   - Average travel time for through traffic
   - Delay distribution across routes
   - Ramp delay vs. mainline benefit trade-off

3. **Ramp Queue Lengths**:
   - Maximum and average queues at THA, HOR, WAE
   - Queue management effectiveness
   - Spillback prevention

4. **Occupancy Control**:
   - Success at maintaining target occupancy (20%)
   - Occupancy variance (stability indicator)
   - Spatial occupancy distribution

5. **Speed Metrics**:
   - Average mainline speed
   - Speed variance (congestion indicator)
   - Speed distributions by location

### Expected Scenario Comparisons

| Metric | Scenario 0 | Scenario 1 | Scenario 2 | Scenario 3 |
|--------|------------|------------|------------|------------|
| Mainline Speed | Low, high variance | Improved, stable | Best, very stable | Similar to Sit1 |
| Ramp Queues | None | Moderate, balanced | Longer at WAE/HOR | Moderate, balanced |
| Throughput | Lowest | Improved | Highest | Similar to Sit1 |
| Coordination | None | None | Cascade control | None |
| Delay Distribution | Mainline only | Distributed to ramps | Optimally distributed | Distributed to ramps |

### Scenario-Specific Insights

**Scenario 0 (Base)**:
- Natural congestion at merge points
- Capacity breakdown during peak demand
- High speed variance
- Maximum mainline delay, zero ramp delay

**Scenario 1 (ALINEA)**:
- Regulated mainline occupancy near 20%
- Smoother mainline flow
- Local queue management
- Each ramp operates independently
- Possible inefficiencies from lack of coordination

**Scenario 2 (ALINEA + HERO)**:
- Coordinated system response
- Upstream ramps assist downstream congestion
- Longer queues at WAE (upstream sacrifice for system benefit)
- Cascade activation during peak periods
- Optimized overall throughput

**Scenario 3 (ALINEA_long)**:
- Long-term stability assessment
- Sustained control performance
- Extended queue dynamics
- Validation of Scenario 1 over longer duration

---

## Summary: Control Strategy Comparison

| Feature | Scenario 0 | Scenario 1 | Scenario 2 | Scenario 3 |
|---------|-----------|-----------|-----------|-----------|
| **Control Type** | None | Local ALINEA | ALINEA + HERO | Local ALINEA |
| **Coordination** | ❌ | ❌ | ✅ | ❌ |
| **Ramp Independence** | N/A | ✅ | ❌ | ✅ |
| **Cascade Logic** | ❌ | ❌ | ✅ | ❌ |
| **Queue Protection** | N/A | ✅ | ✅ | ✅ |
| **Simulation Duration** | Standard | Standard | Standard | Extended |
| **Primary Goal** | Baseline | Local control | System optimization | Long-term validation |

---

## Project Context

This simulation is part of a **Multi-Modal Sustainable Transport Optimization (MMSTO)** study evaluating ramp metering strategies for the A3 highway corridor in Switzerland. The goal is to quantify the benefits of coordinated ramp control in managing peak-period congestion with 2040 projected demand levels.

### Research Questions:
1. How effective is local ALINEA control compared to no control?
2. What additional benefits does HERO coordination provide?
3. What are the trade-offs between mainline efficiency and ramp delay?
4. How does control performance vary over short vs. long durations?

---

## Notes

- All scenarios use **identical demand** (6,620 veh/h)
- All scenarios use **identical vehicle behavior** distribution (20% per aggression level)
- Network geometry is **identical** except for traffic light additions in Scenarios 1-3
- Control parameters (K_R, occupancy target) are **consistent** across controlled scenarios
- Queue override thresholds vary between Scenario 1 and 2 to reflect different control philosophies
- Scenario 3 uses same thresholds as Scenario 1 for direct comparison

---

## Contact & Support

For questions or issues related to this simulation project, please refer to the project documentation or contact the MMSTO research team.
