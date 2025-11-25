#!/usr/bin/env python3
"""
SUMO Traffic Simulation Post-Processing Analysis
=================================================
Generates 8 advanced traffic engineering visualizations (A-H) to evaluate
ALINEA ramp metering effectiveness by processing existing XML output files.

Author: Claude Code
Date: 2025-11-25
"""

import os
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter, uniform_filter1d
import argparse
import logging
import pickle
from typing import Dict, List, Tuple, Optional, Any

# ==========================
# CONFIGURATION & CONSTANTS
# ==========================

# HCM 2010 Level of Service thresholds for freeways (veh/km/lane)
LOS_THRESHOLDS = {
    'A': (0, 11),
    'B': (11, 18),
    'C': (18, 26),
    'D': (26, 35),
    'E': (35, 45),
    'F': (45, 1000),
}

LOS_COLORS = {
    'A': '#2ecc71',  # Green
    'B': '#27ae60',  # Dark green
    'C': '#f1c40f',  # Yellow
    'D': '#e67e22',  # Orange
    'E': '#e74c3c',  # Red
    'F': '#c0392b',  # Dark red
}

# Network configuration (approximate values, will be refined from Network_TL.net.xml)
ZONE_BOUNDARIES = {
    'THA': {
        'on_ramp': (50, 200),
        'acceleration': (200, 325),
        'merge': (325, 426),
        'mid_section': (426, 806),
    },
    'HOR': {
        'on_ramp': (1350, 1615),
        'acceleration': (1615, 1830),
        'merge': (1830, 2000),
        'mid_section': (2000, 2423),
    },
    'WAE': {
        'on_ramp': (2900, 3165),
        'acceleration': (3165, 3269),
        'merge': (3269, 3450),
        'mid_section': (3450, 3695),
    }
}

# Zone configuration for metrics extraction
ZONES_CONFIG = {
    'THA': {
        'mid_edge': 'A3_THA_MID',
        'mid_range': (426, 806),
        'ramp_edge': 'E34_THA',
    },
    'HOR': {
        'mid_edge': 'A3_HOR_MID',
        'mid_range': (2000, 2423),
        'ramp_edge': 'E35_HOR',
    },
    'WAE': {
        'mid_edge': 'A3_WAED_MID',
        'mid_range': (3450, 3695),
        'ramp_edge': 'E36_WAED',
    },
}

# Visualization styling
PLOT_STYLE = {
    'figure.dpi': 300,
    'figure.figsize': (12, 8),
    'font.size': 11,
    'axes.labelsize': 12,
    'axes.titlesize': 14,
    'legend.fontsize': 10,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
}


# ==========================
# LOGGING SETUP
# ==========================

def setup_logging(log_file='post_analysis.log'):
    """Setup logging configuration."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s',
        handlers=[
            logging.FileHandler(log_file),
            logging.StreamHandler()
        ]
    )
    return logging.getLogger(__name__)


# ==========================
# FILE MANAGER CLASS
# ==========================

class FileManager:
    """Handles file I/O operations and path management."""

    def __init__(self, base_dir='.'):
        self.base_dir = base_dir
        self.output_files = {
            'Sit0': {
                'fcd': os.path.join(base_dir, 'output_fcd.xml'),
                'tripinfo': os.path.join(base_dir, 'output_tripinfo.xml'),
                'summary': os.path.join(base_dir, 'output_summary.xml'),
            },
            'Sit1': {
                'fcd': os.path.join(base_dir, 'output_fcd_sit1.xml'),
                'tripinfo': os.path.join(base_dir, 'output_tripinfo_sit1.xml'),
                'summary': os.path.join(base_dir, 'output_summary_sit1.xml'),
            }
        }

        self.network_file = os.path.join(base_dir, 'Network_TL.net.xml')
        self.plot_dir = os.path.join(base_dir, 'plots')

        # Create output directory
        os.makedirs(self.plot_dir, exist_ok=True)

    def validate_files(self, scenario='Sit0'):
        """
        Check if all required files exist for a scenario.

        Args:
            scenario: 'Sit0' or 'Sit1'

        Returns:
            bool: True if all files exist

        Raises:
            FileNotFoundError: If any required file is missing
        """
        missing = []

        for file_type, file_path in self.output_files[scenario].items():
            if not os.path.exists(file_path):
                missing.append(file_path)

        if missing:
            raise FileNotFoundError(
                f"Missing required files for {scenario}:\n" +
                "\n".join(missing)
            )

        return True

    def get_cache_path(self, scenario='Sit0'):
        """Get path for cached FCD data."""
        return os.path.join(self.base_dir, f'fcd_cache_{scenario}.pkl')


# ==========================
# SPATIAL SEGMENT MANAGER CLASS
# ==========================

class SpatialSegmentManager:
    """Handles spatial discretization and coordinate mapping."""

    def __init__(self, network_file='Network_TL.net.xml', segment_length=75):
        """
        Initialize spatial segmentation.

        Args:
            network_file: Path to SUMO network file
            segment_length: Length of each segment in meters (default: 75m)
        """
        self.network_file = network_file
        self.segment_length = segment_length
        self.edge_positions = {}
        self.segments = []

        # Parse network topology
        if os.path.exists(network_file):
            self._parse_network_topology()
        else:
            logging.warning(f"Network file not found: {network_file}. Using default values.")
            self._use_default_topology()

    def _parse_network_topology(self):
        """Extract edge lengths and positions from network file."""
        logging.info(f"Parsing network topology from {self.network_file}")

        try:
            tree = ET.parse(self.network_file)
            root = tree.getroot()

            # Extract edges and their lengths
            for edge in root.findall('.//edge'):
                edge_id = edge.get('id')

                # Skip internal edges
                if edge_id and not edge_id.startswith(':'):
                    # Get lane length (use first lane as representative)
                    lane = edge.find('lane')
                    if lane is not None:
                        length = float(lane.get('length', 0))
                        self.edge_positions[edge_id] = {
                            'length': length,
                            'start_pos': None,  # Will be calculated
                            'end_pos': None
                        }

            logging.info(f"Parsed {len(self.edge_positions)} edges from network")

        except Exception as e:
            logging.error(f"Error parsing network file: {e}")
            self._use_default_topology()

    def _use_default_topology(self):
        """Use predefined topology if network file unavailable."""
        logging.info("Using default network topology")

        # Simplified edge definitions based on exploration
        self.edge_positions = {
            'E34_THA': {'length': 173.82, 'start_pos': 50},
            'E34_THA_ACC': {'length': 125.00, 'start_pos': 200},
            'A3_THA_MERG': {'length': 267.93, 'start_pos': 325},
            'A3_THA_MID': {'length': 273.45, 'start_pos': 426},
            'A3_THA_S': {'length': 874.64, 'start_pos': 806},
            'A3_THA_N': {'length': 152.90, 'start_pos': 0},

            'E35_HOR': {'length': 265.00, 'start_pos': 1350},
            'E35_HOR_ACC': {'length': 215.00, 'start_pos': 1615},
            'A3_HOR_MERG': {'length': 218.12, 'start_pos': 1830},
            'A3_HOR_MID': {'length': 911.98, 'start_pos': 2000},
            'A3_HOR_S': {'length': 394.82, 'start_pos': 2423},

            'E36_WAED': {'length': 106.30, 'start_pos': 2900},
            'E36_WAED_ACC': {'length': 104.00, 'start_pos': 3165},
            'A3_WAED_MERG': {'length': 213.18, 'start_pos': 3269},
            'A3_WAED_MID': {'length': 293.05, 'start_pos': 3450},
            'A3_WAED_S': {'length': 206.48, 'start_pos': 3695},
        }

        # Calculate end positions
        for edge_id, info in self.edge_positions.items():
            info['end_pos'] = info['start_pos'] + info['length']

    def map_vehicle_to_segment(self, lane: str, pos: float) -> Optional[int]:
        """
        Convert (lane, position) to global segment index.

        Args:
            lane: Lane ID (e.g., "A3_HOR_MID_1")
            pos: Position on edge in meters

        Returns:
            Segment index (int) or None if not on mainline
        """
        # Extract edge ID from lane (remove lane number suffix)
        if '_' in lane:
            parts = lane.rsplit('_', 1)
            if parts[-1].isdigit():
                edge_id = parts[0]
            else:
                edge_id = lane
        else:
            edge_id = lane

        if edge_id not in self.edge_positions:
            return None

        # Calculate global position
        edge_info = self.edge_positions[edge_id]
        start_pos = edge_info.get('start_pos') or 0
        global_pos = start_pos + pos

        # Convert to segment index
        segment_idx = int(global_pos // self.segment_length)

        return segment_idx

    def get_zone_boundaries(self):
        """Return position ranges for each ramp zone."""
        return ZONE_BOUNDARIES

    def get_total_segments(self):
        """Calculate total number of segments."""
        # Find maximum position
        max_pos = 0
        for edge_info in self.edge_positions.values():
            start_pos = edge_info.get('start_pos') or 0
            length = edge_info.get('length') or 0
            end_pos = edge_info.get('end_pos') or (start_pos + length)
            max_pos = max(max_pos, end_pos)

        return int(np.ceil(max_pos / self.segment_length))


# ==========================
# XML DATA PARSER CLASS
# ==========================

class XMLDataParser:
    """Efficient XML parsing for SUMO output files."""

    def __init__(self, file_manager: FileManager):
        self.file_manager = file_manager
        self.logger = logging.getLogger(__name__)

    def parse_fcd_incremental(self, fcd_file: str, use_cache=True) -> Dict:
        """
        Memory-efficient incremental parsing of large FCD files.

        Args:
            fcd_file: Path to FCD XML file
            use_cache: If True, use cached data if available

        Returns:
            Dict mapping timestep to vehicle data
        """
        cache_file = fcd_file.replace('.xml', '_cache.pkl')

        # Try to load from cache
        if use_cache and os.path.exists(cache_file):
            self.logger.info(f"Loading FCD data from cache: {cache_file}")
            try:
                with open(cache_file, 'rb') as f:
                    return pickle.load(f)
            except Exception as e:
                self.logger.warning(f"Failed to load cache: {e}. Parsing from XML...")

        self.logger.info(f"Parsing FCD file: {fcd_file} (this may take 2-3 minutes)")

        fcd_data = {}  # {timestep: {veh_id: {attributes}}}

        try:
            context = ET.iterparse(fcd_file, events=('start', 'end'))
            context = iter(context)
            event, root = next(context)

            processed_timesteps = 0

            for event, elem in context:
                if event == 'end' and elem.tag == 'timestep':
                    time = float(elem.get('time'))
                    fcd_data[int(time)] = {}

                    # Process all vehicles in this timestep
                    for veh in elem.findall('vehicle'):
                        veh_id = veh.get('id')
                        fcd_data[int(time)][veh_id] = {
                            'x': float(veh.get('x')),
                            'y': float(veh.get('y')),
                            'speed': float(veh.get('speed')),
                            'lane': veh.get('lane'),
                            'pos': float(veh.get('pos')),
                            'angle': float(veh.get('angle', 0)),
                        }

                    # Clear processed elements to free memory
                    elem.clear()
                    root.clear()

                    processed_timesteps += 1
                    if processed_timesteps % 500 == 0:
                        self.logger.info(f"Processed {processed_timesteps} timesteps...")

            self.logger.info(f"Completed parsing {processed_timesteps} timesteps")

            # Cache the parsed data
            if use_cache:
                self.logger.info(f"Caching FCD data to: {cache_file}")
                with open(cache_file, 'wb') as f:
                    pickle.dump(fcd_data, f)

            return fcd_data

        except Exception as e:
            self.logger.error(f"Error parsing FCD file: {e}")
            raise

    def parse_tripinfo(self, tripinfo_file: str) -> List[Dict]:
        """
        Parse tripinfo.xml into structured list.

        Args:
            tripinfo_file: Path to tripinfo XML file

        Returns:
            List of trip dictionaries
        """
        self.logger.info(f"Parsing tripinfo file: {tripinfo_file}")

        tree = ET.parse(tripinfo_file)
        root = tree.getroot()

        trips = []
        for trip in root.findall('tripinfo'):
            trips.append({
                'id': trip.get('id'),
                'depart': float(trip.get('depart')),
                'arrival': float(trip.get('arrival')),
                'duration': float(trip.get('duration')),
                'routeLength': float(trip.get('routeLength')),
                'timeLoss': float(trip.get('timeLoss')),
                'waitingTime': float(trip.get('waitingTime')),
                'waitingCount': int(trip.get('waitingCount', 0)),
            })

        self.logger.info(f"Parsed {len(trips)} trip records")
        return trips

    def parse_summary(self, summary_file: str) -> List[Dict]:
        """
        Parse summary.xml for system-level statistics.

        Args:
            summary_file: Path to summary XML file

        Returns:
            List of summary dictionaries per timestep
        """
        self.logger.info(f"Parsing summary file: {summary_file}")

        tree = ET.parse(summary_file)
        root = tree.getroot()

        summary_data = []
        for step in root.findall('step'):
            summary_data.append({
                'time': float(step.get('time')),
                'running': int(step.get('running', 0)),
                'arrived': int(step.get('arrived', 0)),
                'meanSpeed': float(step.get('meanSpeed', 0)),
                'meanWaitingTime': float(step.get('meanWaitingTime', 0)),
            })

        self.logger.info(f"Parsed {len(summary_data)} summary records")
        return summary_data


# ==========================
# TRAFFIC METRICS CALCULATOR CLASS
# ==========================

class TrafficMetricsCalculator:
    """Calculate fundamental traffic flow metrics."""

    def __init__(self, segment_manager: SpatialSegmentManager):
        self.segment_manager = segment_manager
        self.logger = logging.getLogger(__name__)

    def calculate_flow_density(self, fcd_data: Dict, edge: str, pos_range: Tuple[float, float],
                               time_window: int = 30, num_lanes: int = 2) -> Dict:
        """
        Calculate flow and density for a specific location over time.

        Args:
            fcd_data: FCD data dictionary
            edge: Edge ID to analyze
            pos_range: (start, end) position range in meters
            time_window: Time window in seconds (default: 30)
            num_lanes: Number of lanes (default: 2)

        Returns:
            Dict with 'density', 'flow', 'speed', 'time' arrays
        """
        densities = []
        flows = []
        speeds = []
        times = []

        segment_length_km = (pos_range[1] - pos_range[0]) / 1000

        max_time = max(fcd_data.keys())

        for time_start in range(0, max_time, time_window):
            time_end = min(time_start + time_window, max_time)

            vehicles_in_window = []

            for t in range(time_start, time_end):
                if t not in fcd_data:
                    continue

                for veh_id, veh_data in fcd_data[t].items():
                    lane = veh_data['lane']
                    if lane.startswith(edge) and pos_range[0] <= veh_data['pos'] <= pos_range[1]:
                        vehicles_in_window.append(veh_data['speed'])

            if len(vehicles_in_window) > 0:
                # Calculate metrics
                density = len(vehicles_in_window) / (segment_length_km * num_lanes * time_window)
                flow = len(vehicles_in_window) / ((time_window / 3600) * num_lanes)
                avg_speed = np.mean(vehicles_in_window)

                densities.append(density)
                flows.append(flow)
                speeds.append(avg_speed)
                times.append(time_start)

        return {
            'density': np.array(densities),
            'flow': np.array(flows),
            'speed': np.array(speeds),
            'time': np.array(times)
        }

    def classify_LOS(self, density: float) -> str:
        """
        Classify Level of Service based on density.

        Args:
            density: Density in veh/km/lane

        Returns:
            LOS category ('A' through 'F')
        """
        for level, (min_d, max_d) in LOS_THRESHOLDS.items():
            if min_d <= density < max_d:
                return level
        return 'F'


# ==========================
# VISUALIZATION ENGINE CLASS
# ==========================

class VisualizationEngine:
    """Generate all visualization plots (A-H)."""

    def __init__(self, segment_manager: SpatialSegmentManager,
                 metrics_calc: TrafficMetricsCalculator,
                 file_manager: FileManager):
        self.segment_manager = segment_manager
        self.metrics_calc = metrics_calc
        self.file_manager = file_manager
        self.logger = logging.getLogger(__name__)

        # Apply plot styling
        plt.rcParams.update(PLOT_STYLE)

    def plot_G_multi_metric_dashboard(self, fcd_data: Dict, scenario='Sit0'):
        """
        Generate Plot G: Multi-Metric Dashboard.
        4-panel view with speed, occupancy, vehicles, queue for all 3 ramps.
        """
        self.logger.info(f"Generating Plot G: Multi-Metric Dashboard for {scenario}")

        # Extract metrics (starting from t=1030, every 30 seconds, like original script)
        time_steps = list(range(1030, max(fcd_data.keys()), 30))

        metrics = {
            'THA': {'speed': [], 'occupancy': [], 'num_veh': [], 'queue': []},
            'HOR': {'speed': [], 'occupancy': [], 'num_veh': [], 'queue': []},
            'WAE': {'speed': [], 'occupancy': [], 'num_veh': [], 'queue': []},
        }

        for time_step in time_steps:
            if time_step not in fcd_data:
                continue

            for zone_name, config in ZONES_CONFIG.items():
                speeds = []
                vehicles_mid = []
                vehicles_ramp = []
                queue_count = 0

                for veh_id, veh_data in fcd_data[time_step].items():
                    # Mainline (mid section)
                    if (veh_data['lane'].startswith(config['mid_edge']) and
                        config['mid_range'][0] <= veh_data['pos'] <= config['mid_range'][1]):
                        speeds.append(veh_data['speed'])
                        vehicles_mid.append(veh_id)

                    # On-ramp
                    if veh_data['lane'].startswith(config['ramp_edge']):
                        vehicles_ramp.append(veh_id)
                        if veh_data['speed'] < 0.1:  # Standing queue
                            queue_count += 1

                # Calculate metrics
                avg_speed = np.mean(speeds) if speeds else 0

                # Occupancy: (num_vehicles × avg_veh_length) / detector_length × 100
                avg_veh_length = 5  # meters
                detector_length = config['mid_range'][1] - config['mid_range'][0]
                occupancy = (len(vehicles_mid) * avg_veh_length / detector_length) * 100

                metrics[zone_name]['speed'].append(avg_speed)
                metrics[zone_name]['occupancy'].append(occupancy)
                metrics[zone_name]['num_veh'].append(len(vehicles_ramp))
                metrics[zone_name]['queue'].append(queue_count)

        # Create 4×3 subplot grid
        fig, axes = plt.subplots(4, 3, figsize=(18, 16), sharex=True)

        ramp_names = ['THA', 'HOR', 'WAE']
        metric_names = ['speed', 'occupancy', 'num_veh', 'queue']
        metric_labels = [
            'Mainline Speed (m/s)',
            'Mainline Occupancy (%)',
            'Vehicles on Ramp (#)',
            'Queue Length (#)'
        ]
        colors = ['green', 'blue', 'red', 'purple']

        for row, (metric, label, color) in enumerate(zip(metric_names, metric_labels, colors)):
            for col, ramp in enumerate(ramp_names):
                ax = axes[row, col]

                ax.plot(time_steps, metrics[ramp][metric],
                       color=color, linewidth=2)

                # Add threshold lines for speed
                if metric == 'speed':
                    ax.axhline(13.89, color='orange', linestyle='--',
                              linewidth=1, alpha=0.5, label='50 km/h')
                    ax.axhline(8.33, color='red', linestyle='--',
                              linewidth=1, alpha=0.5, label='30 km/h')
                    if col == 2:
                        ax.legend(loc='upper right', fontsize=8)

                # Formatting
                if row == 0:
                    ax.set_title(f'{ramp} Ramp', fontsize=12, fontweight='bold')
                if col == 0:
                    ax.set_ylabel(label, fontsize=10)
                if row == 3:
                    ax.set_xlabel('Time (s)', fontsize=10)

                ax.grid(True, alpha=0.3)

        fig.suptitle(f'Multi-Metric Dashboard - {scenario}',
                     fontsize=16, fontweight='bold', y=0.995)

        plt.tight_layout()
        plot_path = os.path.join(self.file_manager.plot_dir,
                                 f'{scenario}_G_multi_metric_dashboard.png')
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        self.logger.info(f"Saved: {plot_path}")
        plt.close()

    # Placeholder methods for other plots (to be implemented)
    def plot_A_time_space_diagram(self, fcd_data: Dict, scenario='Sit0'):
        """
        Generate Plot A: Time-Space Diagram.
        Show congestion wave propagation (position vs time, color=speed).
        """
        self.logger.info(f"Generating Plot A: Time-Space Diagram for {scenario}")

        # 1. Initialize matrix
        max_time = max(fcd_data.keys())
        num_segments = self.segment_manager.get_total_segments()

        speed_matrix = np.full((max_time, num_segments), np.nan)
        count_matrix = np.zeros((max_time, num_segments))

        # 2. Populate matrix from FCD data
        for time_step in range(max_time):
            if time_step not in fcd_data:
                continue

            for veh_data in fcd_data[time_step].values():
                seg_idx = self.segment_manager.map_vehicle_to_segment(
                    veh_data['lane'], veh_data['pos']
                )

                if seg_idx is not None and 0 <= seg_idx < num_segments:
                    if np.isnan(speed_matrix[time_step, seg_idx]):
                        speed_matrix[time_step, seg_idx] = veh_data['speed']
                        count_matrix[time_step, seg_idx] = 1
                    else:
                        # Running average
                        current_avg = speed_matrix[time_step, seg_idx]
                        count = count_matrix[time_step, seg_idx]
                        speed_matrix[time_step, seg_idx] = (current_avg * count + veh_data['speed']) / (count + 1)
                        count_matrix[time_step, seg_idx] += 1

        # 3. Interpolate missing values (light smoothing)
        speed_matrix_smoothed = gaussian_filter(
            np.nan_to_num(speed_matrix, nan=0),
            sigma=(2, 1)  # More smoothing in time dimension
        )

        # 4. Create visualization
        fig, ax = plt.subplots(figsize=(14, 8))

        # Convert speed to km/h
        speed_kmh = speed_matrix_smoothed * 3.6

        max_pos = num_segments * self.segment_manager.segment_length

        im = ax.imshow(
            speed_kmh,
            aspect='auto',
            origin='lower',
            extent=[0, max_pos, 0, max_time],
            cmap='RdYlGn',
            vmin=0,
            vmax=120,
            interpolation='bilinear'
        )

        # 5. Add zone annotations
        zones = self.segment_manager.get_zone_boundaries()
        for ramp, zone_dict in zones.items():
            merge_pos = np.mean(zone_dict['merge'])
            ax.axvline(merge_pos, color='white', linestyle='--',
                      linewidth=1, alpha=0.5)
            ax.text(merge_pos, max_time + 100, f'{ramp}',
                   color='white', fontweight='bold', ha='center', fontsize=9)

        ax.set_xlabel('Position along Highway (m)', fontsize=12)
        ax.set_ylabel('Time (s)', fontsize=12)
        ax.set_title(f'Time-Space Diagram - {scenario}\nCongestion Wave Propagation',
                    fontsize=14, fontweight='bold')

        cbar = plt.colorbar(im, ax=ax, label='Speed (km/h)')
        cbar.ax.axhline(30, color='red', linewidth=2, alpha=0.7)

        plt.tight_layout()
        plot_path = os.path.join(self.file_manager.plot_dir,
                                 f'{scenario}_A_time_space_diagram.png')
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        self.logger.info(f"Saved: {plot_path}")
        plt.close()

    def plot_B_fundamental_diagram(self, fcd_data: Dict, scenario='Sit0'):
        """
        Generate Plot B: Flow-Density Fundamental Diagram.
        Classic traffic engineering relationship showing free-flow vs congested regimes.
        """
        self.logger.info(f"Generating Plot B: Fundamental Diagram for {scenario}")

        # Measurement zones (mid-sections after each ramp)
        measurement_zones = {
            'THA_MID': ('A3_THA_MID', 426, 806, 2),
            'HOR_MID': ('A3_HOR_MID', 2000, 2423, 2),
            'WAE_MID': ('A3_WAED_MID', 3450, 3695, 2),
        }

        fig, ax = plt.subplots(figsize=(10, 7))

        colors = {'THA_MID': 'blue', 'HOR_MID': 'green', 'WAE_MID': 'red'}
        markers = {'free': 'o', 'congested': 'x'}

        for zone_name, (edge, pos_start, pos_end, num_lanes) in measurement_zones.items():
            # Calculate flow and density using metrics calculator
            metrics = self.metrics_calc.calculate_flow_density(
                fcd_data, edge, (pos_start, pos_end), time_window=30, num_lanes=num_lanes
            )

            if len(metrics['density']) == 0:
                continue

            # Classify by speed (free-flow vs congested)
            speed_kmh = metrics['speed'] * 3.6
            free_flow_mask = speed_kmh >= 50
            congested_mask = speed_kmh < 50

            # Plot free-flow points
            if np.any(free_flow_mask):
                ax.scatter(metrics['density'][free_flow_mask],
                          metrics['flow'][free_flow_mask],
                          c=colors[zone_name], alpha=0.6, s=50,
                          marker=markers['free'],
                          label=f'{zone_name} (free-flow)')

            # Plot congested points
            if np.any(congested_mask):
                ax.scatter(metrics['density'][congested_mask],
                          metrics['flow'][congested_mask],
                          c=colors[zone_name], alpha=0.6, s=50,
                          marker=markers['congested'],
                          label=f'{zone_name} (congested)')

        ax.set_xlabel('Density (veh/km/lane)', fontsize=12)
        ax.set_ylabel('Flow (veh/h/lane)', fontsize=12)
        ax.set_title(f'Fundamental Diagram - {scenario}', fontsize=14, fontweight='bold')
        ax.legend(loc='best', fontsize=9, ncol=2)
        ax.grid(True, alpha=0.3)
        ax.set_xlim(left=0)
        ax.set_ylim(bottom=0)

        plt.tight_layout()
        plot_path = os.path.join(self.file_manager.plot_dir,
                                 f'{scenario}_B_fundamental_diagram.png')
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        self.logger.info(f"Saved: {plot_path}")
        plt.close()

    def plot_C_travel_time_analysis(self, tripinfo_data: List, scenario='Sit0'):
        """
        Generate Plot C: Travel Time Analysis.
        Track journey times vs simulation time to show congestion impact.
        """
        self.logger.info(f"Generating Plot C: Travel Time Analysis for {scenario}")

        # 1. Categorize trips by route
        routes = {
            'THA_to_End': [],
            'HOR_to_End': [],
            'WAE_to_End': [],
            'Full_Highway': [],
        }

        for trip in tripinfo_data:
            veh_id = trip['id']

            # Categorize based on ID prefix
            if 'THA' in veh_id and 'to_A3' in veh_id:
                route_type = 'THA_to_End'
            elif 'HOR' in veh_id and 'to_A3' in veh_id:
                route_type = 'HOR_to_End'
            elif ('WAED' in veh_id or 'WAE' in veh_id) and 'to_A3' in veh_id:
                route_type = 'WAE_to_End'
            elif 'A3_full' in veh_id or 'flow_A3' in veh_id:
                route_type = 'Full_Highway'
            else:
                continue

            routes[route_type].append({
                'depart': trip['depart'],
                'duration': trip['duration'],
                'route_length': trip['routeLength'],
                'time_loss': trip['timeLoss']
            })

        # 2. Calculate free-flow travel times (based on route length and speed limit)
        speed_limit = 120 / 3.6  # 120 km/h in m/s

        # 3. Create time series plot
        fig, ax = plt.subplots(figsize=(14, 7))

        colors = {'THA_to_End': 'blue', 'HOR_to_End': 'green',
                  'WAE_to_End': 'red', 'Full_Highway': 'purple'}

        for route_name, trips in routes.items():
            if len(trips) == 0:
                continue

            departs = [t['depart'] for t in trips]
            durations = [t['duration'] for t in trips]

            # Sort by departure time
            sorted_indices = np.argsort(departs)
            departs = np.array(departs)[sorted_indices]
            durations = np.array(durations)[sorted_indices]

            # Apply rolling average (window=min(50, len/10))
            window_size = min(50, max(5, len(durations) // 10))
            if len(durations) > window_size:
                durations_smooth = uniform_filter1d(durations, size=window_size, mode='nearest')
            else:
                durations_smooth = durations

            # Plot actual travel times
            ax.plot(departs, durations_smooth,
                   label=f'{route_name.replace("_", " ")} (n={len(trips)})',
                   color=colors[route_name],
                   linewidth=2, alpha=0.8)

            # Calculate and plot free-flow reference
            if len(trips) > 0 and trips[0]['route_length'] > 0:
                avg_route_length = np.mean([t['route_length'] for t in trips])
                free_flow_tt = avg_route_length / speed_limit
                ax.axhline(free_flow_tt,
                          color=colors[route_name], linestyle='--',
                          linewidth=1, alpha=0.4)

        ax.set_xlabel('Departure Time (s)', fontsize=12)
        ax.set_ylabel('Travel Time (s)', fontsize=12)
        ax.set_title(f'Travel Time Analysis - {scenario}', fontsize=14, fontweight='bold')
        ax.legend(loc='upper left', fontsize=10)
        ax.grid(True, alpha=0.3)

        # Annotate congestion periods
        ax.axvspan(1000, 3000, alpha=0.1, color='red', label='_nolegend_')
        ax.text(2000, ax.get_ylim()[1] * 0.95, 'Peak Period',
               ha='center', fontsize=10, bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        plt.tight_layout()
        plot_path = os.path.join(self.file_manager.plot_dir,
                                 f'{scenario}_C_travel_time_analysis.png')
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        self.logger.info(f"Saved: {plot_path}")
        plt.close()

    def plot_D_spatial_heatmap(self, fcd_data: Dict, scenario='Sit0'):
        """
        Generate Plot D: Spatial Heatmap.
        Average speed by location and time (red=congested, green=free-flow).
        """
        self.logger.info(f"Generating Plot D: Spatial Heatmap for {scenario}")

        # 1. Create 2D grid (60-second time bins × 75m position bins)
        time_bin_size = 60  # 1-minute resolution
        max_time = max(fcd_data.keys())
        num_segments = self.segment_manager.get_total_segments()

        time_bins = np.arange(0, max_time, time_bin_size)
        num_time_bins = len(time_bins)

        speed_grid = np.zeros((num_time_bins, num_segments))
        count_grid = np.zeros((num_time_bins, num_segments))

        # 2. Populate grid
        for time_idx, time_start in enumerate(time_bins):
            time_end = min(time_start + time_bin_size, max_time)

            for t in range(int(time_start), int(time_end)):
                if t not in fcd_data:
                    continue

                for veh_id, veh_data in fcd_data[t].items():
                    seg_idx = self.segment_manager.map_vehicle_to_segment(
                        veh_data['lane'], veh_data['pos']
                    )

                    if seg_idx is not None and 0 <= seg_idx < num_segments:
                        speed_grid[time_idx, seg_idx] += veh_data['speed']
                        count_grid[time_idx, seg_idx] += 1

        # 3. Average (avoid division by zero)
        with np.errstate(divide='ignore', invalid='ignore'):
            avg_speed_grid = speed_grid / count_grid
            avg_speed_grid[count_grid == 0] = np.nan

        # 4. Apply light Gaussian smoothing for visual continuity
        avg_speed_grid_smooth = gaussian_filter(
            np.nan_to_num(avg_speed_grid, nan=0),
            sigma=1.0
        )

        # 5. Create heatmap
        fig, ax = plt.subplots(figsize=(16, 8))

        # Convert to km/h
        avg_speed_grid_kmh = avg_speed_grid_smooth * 3.6

        # Calculate extent (position in meters, time in minutes)
        max_pos = num_segments * self.segment_manager.segment_length
        time_max_minutes = max_time / 60

        im = ax.imshow(
            avg_speed_grid_kmh,
            aspect='auto',
            origin='lower',
            extent=[0, max_pos, 0, time_max_minutes],
            cmap='RdYlGn',
            vmin=0,
            vmax=120,
            interpolation='bilinear'
        )

        # 6. Overlay zone annotations
        zones = self.segment_manager.get_zone_boundaries()
        for ramp, zone_dict in zones.items():
            # On-ramp marker
            ramp_pos = np.mean(zone_dict['on_ramp'])
            ax.axvline(ramp_pos, color='cyan', linestyle=':', linewidth=2, alpha=0.7)
            ax.text(ramp_pos, time_max_minutes + 2, f'{ramp}\nRamp',
                   color='cyan', fontweight='bold', ha='center', fontsize=9)

            # Merge zone highlight
            merge_zone = zone_dict['merge']
            ax.axvspan(merge_zone[0], merge_zone[1],
                      alpha=0.1, color='yellow', label='_nolegend_')

        ax.set_xlabel('Position along Highway (m)', fontsize=12)
        ax.set_ylabel('Time (minutes)', fontsize=12)
        ax.set_title(f'Spatial Speed Heatmap - {scenario}', fontsize=14, fontweight='bold')

        cbar = plt.colorbar(im, ax=ax, label='Average Speed (km/h)')
        cbar.ax.axhline(30, color='red', linewidth=2, alpha=0.7)  # Congestion threshold

        plt.tight_layout()
        plot_path = os.path.join(self.file_manager.plot_dir,
                                 f'{scenario}_D_spatial_heatmap.png')
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        self.logger.info(f"Saved: {plot_path}")
        plt.close()

    def plot_E_cumulative_curves(self, fcd_data: Dict, scenario='Sit0'):
        """
        Generate Plot E: Cumulative Vehicle Curves (N-Curves).
        Visualize queue formation and delay at each ramp.
        """
        self.logger.info(f"Generating Plot E: N-Curves for {scenario}")

        # Virtual detectors for each ramp (simplified approach)
        detectors = {
            'THA': {'ramp': 'E34_THA', 'mainline': 'A3_THA_MID'},
            'HOR': {'ramp': 'E35_HOR', 'mainline': 'A3_HOR_MID'},
            'WAE': {'ramp': 'E36_WAED', 'mainline': 'A3_WAED_MID'},
        }

        fig, axes = plt.subplots(1, 3, figsize=(18, 6), sharey=True)

        for idx, (ramp, det_dict) in enumerate(detectors.items()):
            ax = axes[idx]

            # Track cumulative vehicle counts
            ramp_cumulative = []
            mainline_cumulative = []
            times = []

            ramp_vehicles = set()
            mainline_vehicles = set()

            max_time = max(fcd_data.keys())
            for t in range(0, max_time, 10):  # Sample every 10 seconds
                if t not in fcd_data:
                    continue

                # Count vehicles on ramp
                for veh_id, veh_data in fcd_data[t].items():
                    if veh_data['lane'].startswith(det_dict['ramp']):
                        ramp_vehicles.add(veh_id)
                    if veh_data['lane'].startswith(det_dict['mainline']):
                        mainline_vehicles.add(veh_id)

                times.append(t)
                ramp_cumulative.append(len(ramp_vehicles))
                mainline_cumulative.append(len(mainline_vehicles))

            # Plot curves
            ax.plot(times, ramp_cumulative, 'b-', linewidth=2, label='Ramp Entry')
            ax.plot(times, mainline_cumulative, 'g-', linewidth=2, label='Mainline')

            # Shade queue area
            if len(times) > 0:
                ax.fill_between(times, ramp_cumulative, mainline_cumulative,
                               where=np.array(ramp_cumulative) >= np.array(mainline_cumulative),
                               alpha=0.2, color='red', label='Queue')

            ax.set_xlabel('Time (s)', fontsize=11)
            if idx == 0:
                ax.set_ylabel('Cumulative Vehicle Count', fontsize=11)
            ax.set_title(f'{ramp} Ramp', fontsize=12, fontweight='bold')
            ax.legend(loc='upper left', fontsize=9)
            ax.grid(True, alpha=0.3)

        fig.suptitle(f'Cumulative Vehicle Curves (N-Curves) - {scenario}',
                     fontsize=14, fontweight='bold', y=1.00)

        plt.tight_layout()
        plot_path = os.path.join(self.file_manager.plot_dir,
                                 f'{scenario}_E_cumulative_curves.png')
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        self.logger.info(f"Saved: {plot_path}")
        plt.close()

    def plot_F_level_of_service(self, fcd_data: Dict, scenario='Sit0'):
        """
        Generate Plot F: Level of Service (LOS) Classification.
        HCM-standard congestion assessment over time.
        """
        self.logger.info(f"Generating Plot F: Level of Service for {scenario}")

        # Measurement zones
        measurement_zones = {
            'THA': ('A3_THA_MID', 426, 806, 2),
            'HOR': ('A3_HOR_MID', 2000, 2423, 2),
            'WAE': ('A3_WAED_MID', 3450, 3695, 2),
        }

        # Calculate LOS over time
        los_time_series = {zone: [] for zone in measurement_zones}
        time_axis = []

        window_size = 30
        max_time = max(fcd_data.keys())

        for time_window_start in range(0, max_time, window_size):
            time_axis.append(time_window_start)

            for zone_name, (edge, pos_start, pos_end, num_lanes) in measurement_zones.items():
                vehicle_count = 0

                for t in range(time_window_start, min(time_window_start + window_size, max_time)):
                    if t not in fcd_data:
                        continue

                    for veh_data in fcd_data[t].values():
                        if (veh_data['lane'].startswith(edge) and
                            pos_start <= veh_data['pos'] <= pos_end):
                            vehicle_count += 1

                # Calculate density (veh/km/lane)
                segment_length_km = (pos_end - pos_start) / 1000
                density = vehicle_count / (segment_length_km * num_lanes * window_size) if window_size > 0 else 0

                # Classify LOS
                los = self.metrics_calc.classify_LOS(density)
                los_time_series[zone_name].append(los)

        # Create visualization
        fig, axes = plt.subplots(2, 2, figsize=(16, 10))

        # Plot time series for each zone
        for idx, zone_name in enumerate(['THA', 'HOR', 'WAE']):
            ax = axes[idx // 2, idx % 2]

            # Convert LOS letters to numeric
            los_numeric = {'A': 0, 'B': 1, 'C': 2, 'D': 3, 'E': 4, 'F': 5}
            los_values = [los_numeric[los] for los in los_time_series[zone_name]]

            # Create color-coded scatter
            for level, color in LOS_COLORS.items():
                mask = np.array(los_time_series[zone_name]) == level
                if np.any(mask):
                    ax.scatter(np.array(time_axis)[mask],
                              np.array(los_values)[mask],
                              c=color, label=f'LOS {level}',
                              s=30, alpha=0.7)

            ax.set_xlabel('Time (s)', fontsize=10)
            ax.set_ylabel('Level of Service', fontsize=10)
            ax.set_yticks([0, 1, 2, 3, 4, 5])
            ax.set_yticklabels(['A', 'B', 'C', 'D', 'E', 'F'])
            ax.set_title(f'{zone_name} Zone', fontsize=11, fontweight='bold')
            ax.grid(True, alpha=0.3)
            if idx == 0:
                ax.legend(loc='upper right', fontsize=8, ncol=2)

        # Overall LOS distribution (pie chart)
        ax = axes[1, 1]
        all_los = []
        for zone_data in los_time_series.values():
            all_los.extend(zone_data)

        los_counts = {level: all_los.count(level) for level in ['A', 'B', 'C', 'D', 'E', 'F']}
        los_percentages = {level: count/len(all_los)*100
                          for level, count in los_counts.items() if count > 0}

        if los_percentages:
            ax.pie(los_percentages.values(),
                  labels=[f'LOS {k}\n{v:.1f}%' for k, v in los_percentages.items()],
                  colors=[LOS_COLORS[k] for k in los_percentages.keys()],
                  autopct='',
                  startangle=90)
            ax.set_title('Overall LOS Distribution', fontsize=11, fontweight='bold')

        fig.suptitle(f'Level of Service Analysis - {scenario}',
                     fontsize=14, fontweight='bold')

        plt.tight_layout()
        plot_path = os.path.join(self.file_manager.plot_dir,
                                 f'{scenario}_F_level_of_service.png')
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        self.logger.info(f"Saved: {plot_path}")
        plt.close()

    def plot_H_before_after_comparison(self, fcd_sit0: Dict, fcd_sit1: Dict):
        """
        Generate Plot H: Before/After Comparison.
        Side-by-side Sit0 vs Sit1 comparison.
        """
        self.logger.info(f"Generating Plot H: Before/After Comparison")

        # Extract metrics for both scenarios
        time_steps = list(range(1030, min(max(fcd_sit0.keys()), max(fcd_sit1.keys())), 30))

        scenarios_data = {'Sit0': fcd_sit0, 'Sit1': fcd_sit1}
        comparison_metrics = {}

        for scenario_name, fcd_data in scenarios_data.items():
            metrics = {
                'THA': {'speed': [], 'queue': []},
                'HOR': {'speed': [], 'queue': []},
                'WAE': {'speed': [], 'queue': []},
            }

            for time_step in time_steps:
                if time_step not in fcd_data:
                    continue

                for zone_name, config in ZONES_CONFIG.items():
                    speeds = []
                    queue_count = 0

                    for veh_data in fcd_data[time_step].values():
                        # Mainline speed
                        if (veh_data['lane'].startswith(config['mid_edge']) and
                            config['mid_range'][0] <= veh_data['pos'] <= config['mid_range'][1]):
                            speeds.append(veh_data['speed'])

                        # Ramp queue
                        if veh_data['lane'].startswith(config['ramp_edge']):
                            if veh_data['speed'] < 0.1:
                                queue_count += 1

                    metrics[zone_name]['speed'].append(np.mean(speeds) if speeds else 0)
                    metrics[zone_name]['queue'].append(queue_count)

            comparison_metrics[scenario_name] = metrics

        # Create 2×3 subplot grid
        fig, axes = plt.subplots(2, 3, figsize=(20, 12))

        ramp_names = ['THA', 'HOR', 'WAE']

        for col, ramp in enumerate(ramp_names):
            for row, scenario in enumerate(['Sit0', 'Sit1']):
                ax = axes[row, col]
                ax_twin = ax.twinx()

                # Speed (left y-axis)
                speed_data = comparison_metrics[scenario][ramp]['speed']
                ax.plot(time_steps[:len(speed_data)], speed_data,
                       color='green', linewidth=2, label='Mainline Speed')
                ax.set_ylabel('Speed (m/s)', fontsize=11, color='green')
                ax.tick_params(axis='y', labelcolor='green')

                # Queue (right y-axis)
                queue_data = comparison_metrics[scenario][ramp]['queue']
                ax_twin.plot(time_steps[:len(queue_data)], queue_data,
                            color='purple', linewidth=2, label='Queue Length', linestyle='--')
                ax_twin.set_ylabel('Queue Length (#)', fontsize=11, color='purple')
                ax_twin.tick_params(axis='y', labelcolor='purple')

                title = f'{ramp} - {"Baseline (No Control)" if scenario == "Sit0" else "ALINEA Control"}'
                ax.set_title(title, fontsize=12, fontweight='bold')

                if row == 1:
                    ax.set_xlabel('Time (s)', fontsize=11)

                ax.grid(True, alpha=0.3)

                # Calculate improvements for Sit1
                if scenario == 'Sit1' and len(speed_data) > 0:
                    speed_sit0 = comparison_metrics['Sit0'][ramp]['speed']
                    queue_sit0 = comparison_metrics['Sit1'][ramp]['queue']

                    if len(speed_sit0) > 0 and np.mean(speed_sit0) > 0:
                        speed_improv = (np.mean(speed_data) - np.mean(speed_sit0)) / np.mean(speed_sit0) * 100
                        queue_reduc = (np.mean(queue_sit0) - np.mean(queue_data)) / max(np.mean(queue_sit0), 1) * 100

                        ax.text(0.02, 0.98,
                               f'Speed: {speed_improv:+.1f}%\nQueue: {queue_reduc:+.1f}%',
                               transform=ax.transAxes,
                               fontsize=10, fontweight='bold',
                               verticalalignment='top',
                               bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

        fig.suptitle('Before/After Comparison: Baseline vs ALINEA',
                     fontsize=16, fontweight='bold', y=0.995)

        plt.tight_layout()
        plot_path = os.path.join(self.file_manager.plot_dir,
                                 'H_before_after_comparison.png')
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        self.logger.info(f"Saved: {plot_path}")
        plt.close()


# ==========================
# MAIN PROCESSOR CLASS
# ==========================

class SUMOPostProcessor:
    """Main orchestrator for SUMO post-processing analysis."""

    def __init__(self, base_dir='.'):
        self.file_manager = FileManager(base_dir)
        self.segment_manager = SpatialSegmentManager(
            self.file_manager.network_file
        )
        self.xml_parser = XMLDataParser(self.file_manager)
        self.metrics_calc = TrafficMetricsCalculator(self.segment_manager)
        self.viz_engine = VisualizationEngine(
            self.segment_manager,
            self.metrics_calc,
            self.file_manager
        )
        self.logger = logging.getLogger(__name__)

    def analyze_scenario(self, scenario='Sit0', plots_to_generate='all'):
        """
        Main analysis pipeline for a single scenario.

        Args:
            scenario: 'Sit0' or 'Sit1'
            plots_to_generate: 'all' or list like ['A', 'B', 'C']
        """
        self.logger.info(f"=" * 60)
        self.logger.info(f"Starting analysis for {scenario}")
        self.logger.info(f"=" * 60)

        # 1. Validate files
        self.file_manager.validate_files(scenario)

        # 2. Parse data
        self.logger.info("Parsing XML files...")
        fcd_data = self.xml_parser.parse_fcd_incremental(
            self.file_manager.output_files[scenario]['fcd']
        )
        tripinfo_data = self.xml_parser.parse_tripinfo(
            self.file_manager.output_files[scenario]['tripinfo']
        )
        summary_data = self.xml_parser.parse_summary(
            self.file_manager.output_files[scenario]['summary']
        )

        # 3. Generate plots
        plot_mapping = {
            'A': lambda: self.viz_engine.plot_A_time_space_diagram(fcd_data, scenario),
            'B': lambda: self.viz_engine.plot_B_fundamental_diagram(fcd_data, scenario),
            'C': lambda: self.viz_engine.plot_C_travel_time_analysis(tripinfo_data, scenario),
            'D': lambda: self.viz_engine.plot_D_spatial_heatmap(fcd_data, scenario),
            'E': lambda: self.viz_engine.plot_E_cumulative_curves(fcd_data, scenario),
            'F': lambda: self.viz_engine.plot_F_level_of_service(fcd_data, scenario),
            'G': lambda: self.viz_engine.plot_G_multi_metric_dashboard(fcd_data, scenario),
        }

        if plots_to_generate == 'all' or (isinstance(plots_to_generate, list) and 'all' in plots_to_generate):
            plots_to_generate = list(plot_mapping.keys())

        for plot_id in plots_to_generate:
            if plot_id in plot_mapping:
                self.logger.info(f"Generating Plot {plot_id}...")
                plot_mapping[plot_id]()

        self.logger.info(f"Analysis complete for {scenario}")

        return fcd_data, tripinfo_data, summary_data

    def compare_scenarios(self):
        """Generate Plot H: Before/After Comparison."""
        self.logger.info("=" * 60)
        self.logger.info("Generating before/after comparison (Plot H)...")
        self.logger.info("=" * 60)

        # Parse both scenarios
        fcd_sit0 = self.xml_parser.parse_fcd_incremental(
            self.file_manager.output_files['Sit0']['fcd']
        )
        fcd_sit1 = self.xml_parser.parse_fcd_incremental(
            self.file_manager.output_files['Sit1']['fcd']
        )

        self.viz_engine.plot_H_before_after_comparison(fcd_sit0, fcd_sit1)

        self.logger.info("Comparison plot complete")


# ==========================
# MAIN ENTRY POINT
# ==========================

def main():
    """Main entry point with CLI interface."""
    parser = argparse.ArgumentParser(
        description='Post-process SUMO traffic simulation outputs',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Analyze Sit0 only, all plots
  python post_analysis_sumo.py --scenario Sit0

  # Analyze Sit1, specific plots
  python post_analysis_sumo.py --scenario Sit1 --plots A B C

  # Analyze both scenarios and generate comparison
  python post_analysis_sumo.py --scenario both --compare

  # Generate only comparison plot
  python post_analysis_sumo.py --plots H
        """
    )

    parser.add_argument(
        '--scenario',
        choices=['Sit0', 'Sit1', 'both'],
        default='Sit0',
        help='Which scenario to analyze'
    )
    parser.add_argument(
        '--plots',
        nargs='+',
        choices=['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'all'],
        default=['all'],
        help='Which plots to generate'
    )
    parser.add_argument(
        '--base-dir',
        default='.',
        help='Base directory containing output files'
    )
    parser.add_argument(
        '--compare',
        action='store_true',
        help='Generate before/after comparison (Plot H)'
    )

    args = parser.parse_args()

    # Setup logging
    logger = setup_logging()

    logger.info("=" * 60)
    logger.info("SUMO Traffic Simulation Post-Processing Analysis")
    logger.info("=" * 60)

    # Initialize processor
    processor = SUMOPostProcessor(base_dir=args.base_dir)

    # Run analysis
    try:
        if args.scenario in ['Sit0', 'both']:
            processor.analyze_scenario('Sit0', args.plots)

        if args.scenario in ['Sit1', 'both']:
            processor.analyze_scenario('Sit1', args.plots)

        if args.compare or 'H' in args.plots:
            processor.compare_scenarios()

        logger.info("\n" + "=" * 60)
        logger.info("Analysis complete! Plots saved to: " + processor.file_manager.plot_dir)
        logger.info("=" * 60)

    except Exception as e:
        logger.error(f"Analysis failed: {e}", exc_info=True)
        sys.exit(1)


if __name__ == '__main__':
    main()
