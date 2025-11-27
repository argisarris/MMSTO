"""
Network Infrastructure Visualization Script
Parses SUMO network file and generates edge list + schematic plots

Author: Claude Code
Date: 2025-11-25
"""

import os
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import Rectangle, FancyArrowPatch
import pandas as pd
import numpy as np
from typing import Dict, List, Tuple, Optional
import openpyxl  # For Excel export

# Configuration constants
PLOT_STYLE = {
    'figure.dpi': 300,
    'font.size': 10,
    'axes.labelsize': 11,
    'axes.titlesize': 13,
    'legend.fontsize': 9,
}

COLOR_SCHEME = {
    'mainline': '#1f77b4',      # Blue
    'on_ramp': '#2ca02c',        # Green
    'off_ramp': '#d62728',       # Red
    'acceleration': '#ff7f0e',   # Orange
    'merge': '#ff9896',          # Light coral
}


def categorize_length(length_m: float) -> Tuple[str, float]:
    """
    Categorize edge length into schematic bins.

    Args:
        length_m: Actual edge length in meters

    Returns:
        Tuple of (category_name, visual_width)
    """
    if length_m < 200:
        return ('Short', 50)
    elif length_m < 500:
        return ('Medium', 100)
    elif length_m < 1000:
        return ('Long', 150)
    else:
        return ('Very Long', 200)


def create_vertical_ramp(x_pos: float, y_start: float, vertical_extent: float,
                        width: float) -> Rectangle:
    """
    Create rectangle for vertical ramp extending downward from mainline.

    Args:
        x_pos: Horizontal center position
        y_start: Starting y position (mainline level)
        vertical_extent: Vertical length (categorical: 50, 100, 150, or 200)
        width: Rectangle width (num_lanes * 0.5)

    Returns:
        Rectangle patch for the vertical ramp
    """
    # Ramps extend downward, so bottom-left corner is at (x_pos - width/2, y_start - vertical_extent)
    return Rectangle((x_pos - width/2, y_start - vertical_extent),
                    width, vertical_extent)


def get_junction_height(junction_id: str, edges: Dict, heights_map: Dict) -> float:
    """
    Calculate junction height as maximum of adjacent edges.

    Args:
        junction_id: Junction identifier
        edges: Dictionary of edge information
        heights_map: Dictionary mapping edge_id to height

    Returns:
        Maximum height among connected edges
    """
    connected_edges = [
        edge_id for edge_id, edge_data in edges.items()
        if edge_data['from'] == junction_id or edge_data['to'] == junction_id
    ]

    if connected_edges:
        return max([heights_map.get(e, 0.5) for e in connected_edges])
    return 0.5  # Default minimum


def parse_network_xml(xml_file: str = 'Network.net.xml') -> Dict:
    """
    Parse SUMO network XML file and extract edge information.

    Args:
        xml_file: Path to the network XML file

    Returns:
        Dictionary with edge information
    """
    tree = ET.parse(xml_file)
    root = tree.getroot()

    edges_data = {}

    for edge in root.findall('edge'):
        edge_id = edge.get('id')

        # Skip internal junction edges (start with ':')
        if edge_id.startswith(':'):
            continue

        # Extract edge attributes
        from_junction = edge.get('from', '')
        to_junction = edge.get('to', '')
        priority = edge.get('priority', '')
        shape = edge.get('shape', '')

        # Extract lane information
        lanes = edge.findall('lane')
        num_lanes = len(lanes)

        # Get speed and length from first lane (typically same for all lanes)
        if lanes:
            first_lane = lanes[0]
            speed_ms = float(first_lane.get('speed', 0))
            length_m = float(first_lane.get('length', 0))
        else:
            speed_ms = 0
            length_m = 0

        # Parse shape coordinates to get position
        x_coords = []
        if shape:
            coords = shape.split()
            for coord in coords:
                x, y = coord.split(',')
                x_coords.append(float(x))

        avg_x = np.mean(x_coords) if x_coords else 0

        edges_data[edge_id] = {
            'id': edge_id,
            'from': from_junction,
            'to': to_junction,
            'priority': priority,
            'num_lanes': num_lanes,
            'speed_ms': speed_ms,
            'speed_kmh': speed_ms * 3.6,
            'length_m': length_m,
            'shape': shape,
            'avg_x': avg_x,
            'x_coords': x_coords
        }

    return edges_data


def categorize_edge(edge_id: str) -> str:
    """
    Categorize edge based on its ID pattern.

    Args:
        edge_id: Edge identifier

    Returns:
        Category string
    """
    if edge_id.startswith('A3_'):
        if '_MERG' in edge_id:
            return 'Merge'
        return 'Mainline'
    elif edge_id.startswith('E3') and 'ACC' in edge_id:
        return 'Acceleration'
    elif edge_id.startswith('E3'):
        return 'On-ramp'
    elif edge_id.startswith('A3') and edge_id[2] in ['4', '5', '6']:
        return 'Off-ramp'
    else:
        return 'Other'


def get_section(edge_id: str) -> str:
    """Get geographic section (THA, HOR, WAED) from edge ID."""
    if 'THA' in edge_id:
        return 'THA'
    elif 'HOR' in edge_id:
        return 'HOR'
    elif 'WAE' in edge_id:
        return 'WAED'
    else:
        return 'Unknown'


def create_edge_dataframe(edges: Dict, detectors: Dict = None) -> pd.DataFrame:
    """
    Create a pandas DataFrame from edges dictionary with detector information.
    Edges are ordered sequentially as they appear in the network (WAE -> HOR -> THA).

    Args:
        edges: Dictionary of edge information
        detectors: Dictionary of detector information (optional)

    Returns:
        Formatted DataFrame
    """
    # Define sequential order matching the network topology (south to north, WAE -> HOR -> THA)
    # Based on the order from 00_asarris_RunSimulation_Sit0.py lines 342-360
    sequential_order = [
        'A3_WAED_S',
        'A3_WAED_MID',
        'E36_WAED',
        'E36_WAED_ACC',
        'A3_WAED_MERG',
        'A36_WAED',
        'A3_HOR_S',
        'A3_HOR_MID',
        'E35_HOR',
        'E35_HOR_ACC',
        'A3_HOR_MERG',
        'A35_HOR',
        'A3_THA_S',
        'A3_THA_MID',
        'E34_THA',
        'E34_THA_ACC',
        'A3_THA_MERG',
        'A34_THA',
        'A3_THA_N',
    ]

    # Build detector mapping by edge
    detector_map = {}
    if detectors:
        # Process induction loops
        for loop in detectors.get('induction_loops', []):
            lane = loop['lane']
            edge_id = lane.rsplit('_', 1)[0]  # Extract edge ID from lane
            if edge_id not in detector_map:
                detector_map[edge_id] = []
            detector_map[edge_id].append({
                'id': loop['id'],
                'type': 'Induction Loop'
            })

        # Process lane area detectors
        for area in detectors.get('lane_area_detectors', []):
            lane = area['lane']
            edge_id = lane.rsplit('_', 1)[0]  # Extract edge ID from lane
            if edge_id not in detector_map:
                detector_map[edge_id] = []
            detector_map[edge_id].append({
                'id': area['id'],
                'type': 'Zone Detector'
            })

    data_list = []

    for edge_id in sequential_order:
        if edge_id in edges:
            edge_data = edges[edge_id]
            category = categorize_edge(edge_id)
            section = get_section(edge_id)

            # Get detector info for this edge
            detector_ids = []
            detector_types = []
            if edge_id in detector_map:
                for det in detector_map[edge_id]:
                    detector_ids.append(det['id'])
                    detector_types.append(det['type'])

            detector_id_str = ', '.join(detector_ids) if detector_ids else ''
            detector_type_str = ', '.join(detector_types) if detector_types else ''

            data_list.append({
                'Edge ID': edge_id,
                'Category': category,
                'Section': section,
                'From -> To': f"{edge_data['from']} -> {edge_data['to']}",
                'Lanes': edge_data['num_lanes'],
                'Speed (m/s)': round(edge_data['speed_ms'], 2),
                'Speed (km/h)': round(edge_data['speed_kmh'], 1),
                'Length (m)': round(edge_data['length_m'], 2),
                'Priority': edge_data['priority'],
                'Detector ID': detector_id_str,
                'Detector Type': detector_type_str,
            })

    df = pd.DataFrame(data_list)
    return df


def create_junction_dataframe(junctions: Dict, edges: Dict) -> pd.DataFrame:
    """
    Create a pandas DataFrame from junctions dictionary with individual internal lane data.

    Args:
        junctions: Dictionary of junction information (with internal edges)
        edges: Dictionary of edge information (to find connected edges)

    Returns:
        Formatted DataFrame with one row per internal lane
    """
    data_list = []

    for junc_id, junc_data in junctions.items():
        # Skip internal junctions
        if junc_id.startswith(':'):
            continue

        # Find connected edges
        incoming_edges = []
        outgoing_edges = []

        for edge_id, edge_data in edges.items():
            if edge_data['to'] == junc_id:
                incoming_edges.append(edge_id)
            if edge_data['from'] == junc_id:
                outgoing_edges.append(edge_id)

        # Get internal edges (lanes)
        internal_edges = junc_data.get('internal_edges', [])

        if internal_edges:
            # Create a row for each internal lane
            for i, ie in enumerate(internal_edges, 1):
                data_list.append({
                    'Junction ID': junc_id,
                    'Type': junc_data['type'],
                    'Internal Lane': i,
                    'Internal Edge ID': ie['edge_id'],
                    'Internal Lane ID': ie['lane_id'],
                    'Length (m)': round(ie['length_m'], 2),
                    'Speed (m/s)': round(ie['speed_ms'], 2),
                    'X Coordinate': round(junc_data['x'], 2),
                    'Y Coordinate': round(junc_data['y'], 2),
                    'Incoming Edges': ', '.join(incoming_edges) if incoming_edges else '',
                    'Outgoing Edges': ', '.join(outgoing_edges) if outgoing_edges else '',
                    'Total Connections': len(incoming_edges) + len(outgoing_edges),
                })
        else:
            # Junction with no internal edges
            data_list.append({
                'Junction ID': junc_id,
                'Type': junc_data['type'],
                'Internal Lane': 0,
                'Internal Edge ID': '',
                'Internal Lane ID': '',
                'Length (m)': 0,
                'Speed (m/s)': 0,
                'X Coordinate': round(junc_data['x'], 2),
                'Y Coordinate': round(junc_data['y'], 2),
                'Incoming Edges': ', '.join(incoming_edges) if incoming_edges else '',
                'Outgoing Edges': ', '.join(outgoing_edges) if outgoing_edges else '',
                'Total Connections': len(incoming_edges) + len(outgoing_edges),
            })

    # Sort by junction ID and internal lane number
    df = pd.DataFrame(data_list)
    df = df.sort_values(['Junction ID', 'Internal Lane']).reset_index(drop=True)
    return df


def print_edge_list(df: pd.DataFrame) -> None:
    """
    Print formatted edge list table to console.

    Args:
        df: Edge data DataFrame
    """
    print("\n" + "="*100)
    print("NETWORK INFRASTRUCTURE - EDGE LIST (Sequential Order: WAE -> HOR -> THA)")
    print("="*100 + "\n")

    # Print in sequential order without grouping
    print(df.to_string(index=True))

    print("\n" + "="*100)
    print(f"TOTAL: {len(df)} edges")
    print("="*100 + "\n")


def plot_lane_level_schematic(df: pd.DataFrame, edges: Dict, output_file: str = 'network_infrastructure_lane_level.png') -> None:
    """
    Create lane-level schematic plot with individual lanes shown.

    Args:
        df: Edge data DataFrame
        edges: Full edge data dictionary
        output_file: Output filename
    """
    plt.rcParams.update(PLOT_STYLE)
    fig, ax = plt.subplots(figsize=(18, 12))

    # Y-position tracking
    y_pos = 0
    y_spacing = 0.5  # Space between lanes
    edge_spacing = 1.5  # Space between edges

    # Track y-positions for annotations
    edge_positions = {}

    # Process edges in sequential order (as they appear in the dataframe)
    for _, row in df.iterrows():
        edge_id = row['Edge ID']
        category = row['Category']
        edge_data = edges[edge_id]
        num_lanes = edge_data['num_lanes']
        speed_kmh = edge_data['speed_kmh']
        length_m = edge_data['length_m']

        # Determine color based on category
        if category == 'Mainline':
            color = COLOR_SCHEME['mainline']
        elif category == 'Merge':
            color = COLOR_SCHEME['merge']
        elif category == 'On-ramp':
            color = COLOR_SCHEME['on_ramp']
        elif category == 'Acceleration':
            color = COLOR_SCHEME['acceleration']
        elif category == 'Off-ramp':
            color = COLOR_SCHEME['off_ramp']
        else:
            color = 'gray'

        # Normalized length for visualization (scale to reasonable size)
        viz_length = length_m / 50  # Scale down for visualization

        # Plot each lane
        edge_start_y = y_pos
        for lane_idx in range(num_lanes):
            # Draw lane as rectangle
            rect = Rectangle((0, y_pos), viz_length, 0.4,
                           facecolor=color, edgecolor='black', linewidth=0.5, alpha=0.7)
            ax.add_patch(rect)

            # Add lane number
            ax.text(viz_length + 0.5, y_pos + 0.2, f"L{lane_idx}",
                   fontsize=7, va='center')

            y_pos += y_spacing

        # Add edge label and speed
        edge_center_y = edge_start_y + (num_lanes - 1) * y_spacing / 2
        edge_positions[edge_id] = edge_center_y

        ax.text(-1, edge_center_y, edge_id, fontsize=8, va='center', ha='right', weight='bold')
        ax.text(viz_length / 2, edge_center_y, f"{speed_kmh:.0f} km/h",
               fontsize=7, va='center', ha='center', color='white', weight='bold',
               bbox=dict(boxstyle='round,pad=0.3', facecolor='black', alpha=0.5))

        y_pos += edge_spacing

    # Add section markers
    sections = ['THA (Thalwil)', 'HOR (Horgen)', 'WAED (Wädenswil)']
    section_y = -3
    for i, section in enumerate(sections):
        ax.text(i * 10 + 5, section_y, section, fontsize=11, weight='bold', ha='center',
               bbox=dict(boxstyle='round,pad=0.5', facecolor='lightgray', alpha=0.7))

    # Create legend
    legend_elements = [
        mpatches.Patch(color=COLOR_SCHEME['mainline'], label='Mainline A3', alpha=0.7),
        mpatches.Patch(color=COLOR_SCHEME['merge'], label='Merge Sections', alpha=0.7),
        mpatches.Patch(color=COLOR_SCHEME['on_ramp'], label='On-ramps', alpha=0.7),
        mpatches.Patch(color=COLOR_SCHEME['acceleration'], label='Acceleration Lanes', alpha=0.7),
        mpatches.Patch(color=COLOR_SCHEME['off_ramp'], label='Off-ramps', alpha=0.7),
    ]
    ax.legend(handles=legend_elements, loc='upper right', framealpha=0.9)

    # Formatting
    ax.set_xlim(-8, 40)
    ax.set_ylim(section_y - 2, y_pos + 2)
    ax.set_xlabel('Relative Length (scaled)', fontsize=11)
    ax.set_ylabel('Lane Configuration', fontsize=11)
    ax.set_title('Network Infrastructure - Lane-Level Schematic\nSUMO Network: A3 Highway (THA - HOR - WAED)',
                fontsize=14, weight='bold', pad=20)
    ax.grid(True, alpha=0.3, linestyle='--')
    ax.set_axisbelow(True)

    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"[OK] Lane-level schematic saved: {output_file}")
    plt.close()


def plot_edge_level_schematic(df: pd.DataFrame, edges: Dict, output_file: str = 'network_infrastructure_edge_level.png') -> None:
    """
    Create edge-level schematic plot (simplified, one line per edge).

    Args:
        df: Edge data DataFrame
        edges: Full edge data dictionary
        output_file: Output filename
    """
    plt.rcParams.update(PLOT_STYLE)
    fig, ax = plt.subplots(figsize=(16, 10))

    # Y-position tracking
    y_pos = 0
    edge_spacing = 1.2

    # Track positions
    edge_positions = {}

    # Process edges in sequential order
    for _, row in df.iterrows():
            edge_id = row['Edge ID']
            category = row['Category']
            edge_data = edges[edge_id]
            num_lanes = row['Lanes']
            speed_kmh = row['Speed (km/h)']
            length_m = row['Length (m)']

            # Determine color
            if category == 'Mainline':
                color = COLOR_SCHEME['mainline']
            elif category == 'Merge':
                color = COLOR_SCHEME['merge']
            elif category == 'On-ramp':
                color = COLOR_SCHEME['on_ramp']
            elif category == 'Acceleration':
                color = COLOR_SCHEME['acceleration']
            elif category == 'Off-ramp':
                color = COLOR_SCHEME['off_ramp']
            else:
                color = 'gray'

            # Normalized length
            viz_length = length_m / 50

            # Line thickness proportional to lanes
            linewidth = num_lanes * 3

            # Draw edge as thick line
            ax.plot([0, viz_length], [y_pos, y_pos], color=color, linewidth=linewidth,
                   solid_capstyle='round', alpha=0.7)

            edge_positions[edge_id] = y_pos

            # Add edge label
            ax.text(-1, y_pos, edge_id, fontsize=9, va='center', ha='right', weight='bold')

            # Add lane count and speed info
            info_text = f"{num_lanes}L | {speed_kmh:.0f} km/h | {length_m:.0f}m"
            ax.text(viz_length / 2, y_pos, info_text, fontsize=8, va='center', ha='center',
                   color='white', weight='bold',
                   bbox=dict(boxstyle='round,pad=0.4', facecolor='black', alpha=0.6))

            y_pos += edge_spacing

    # Add section markers
    sections = ['THA (Thalwil)', 'HOR (Horgen)', 'WAED (Wädenswil)']
    section_y = -2.5
    for i, section in enumerate(sections):
        ax.text(i * 10 + 5, section_y, section, fontsize=12, weight='bold', ha='center',
               bbox=dict(boxstyle='round,pad=0.6', facecolor='lightgray', edgecolor='black', linewidth=1.5, alpha=0.8))

    # Create legend
    legend_elements = [
        mpatches.Patch(color=COLOR_SCHEME['mainline'], label='Mainline A3', alpha=0.7),
        mpatches.Patch(color=COLOR_SCHEME['merge'], label='Merge Sections', alpha=0.7),
        mpatches.Patch(color=COLOR_SCHEME['on_ramp'], label='On-ramps', alpha=0.7),
        mpatches.Patch(color=COLOR_SCHEME['acceleration'], label='Acceleration Lanes', alpha=0.7),
        mpatches.Patch(color=COLOR_SCHEME['off_ramp'], label='Off-ramps', alpha=0.7),
    ]
    ax.legend(handles=legend_elements, loc='upper right', framealpha=0.9, fontsize=10)

    # Add note about line thickness
    ax.text(0.5, 0.02, 'Note: Line thickness represents number of lanes',
           transform=ax.transAxes, fontsize=9, style='italic', ha='center',
           bbox=dict(boxstyle='round,pad=0.5', facecolor='yellow', alpha=0.3))

    # Formatting
    ax.set_xlim(-10, 40)
    ax.set_ylim(section_y - 2, y_pos + 1)
    ax.set_xlabel('Relative Length (scaled)', fontsize=12)
    ax.set_ylabel('Edge Configuration', fontsize=12)
    ax.set_title('Network Infrastructure - Edge-Level Schematic\nSUMO Network: A3 Highway (THA - HOR - WAED)',
                fontsize=14, weight='bold', pad=20)
    ax.grid(True, alpha=0.3, linestyle='--', axis='x')
    ax.set_axisbelow(True)

    # Remove y-axis ticks (not meaningful for schematic)
    ax.set_yticks([])

    plt.tight_layout()
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"[OK] Edge-level schematic saved: {output_file}")
    plt.close()


def parse_junctions(xml_file: str = 'Network.net.xml') -> Dict:
    """
    Parse junction information from SUMO network XML including internal edge data.

    Args:
        xml_file: Path to the network XML file

    Returns:
        Dictionary with junction information including internal edge lengths and speeds
    """
    tree = ET.parse(xml_file)
    root = tree.getroot()

    junctions = {}

    # First pass: Parse junction nodes
    for junction in root.findall('junction'):
        junction_id = junction.get('id')
        junction_type = junction.get('type')
        x = float(junction.get('x', 0))
        y = float(junction.get('y', 0))

        junctions[junction_id] = {
            'id': junction_id,
            'x': x,
            'y': y,
            'type': junction_type,
            'internal_edges': []
        }

    # Second pass: Parse internal edges to get junction lengths and speeds
    for edge in root.findall('edge'):
        edge_id = edge.get('id')

        # Only process internal junction edges (start with ':')
        if edge_id.startswith(':'):
            # Extract junction ID from internal edge name (e.g., ':EN34_THA_0' -> 'EN34_THA')
            junction_id = edge_id.split('_')[0][1:]  # Remove ':' and get first part
            # Reconstruct full junction name
            parts = edge_id[1:].split('_')
            if len(parts) >= 2:
                junction_id = '_'.join(parts[:-1])  # Everything except the last number

            # Get lane information
            lanes = edge.findall('lane')
            if lanes:
                for lane in lanes:
                    length_m = float(lane.get('length', 0))
                    speed_ms = float(lane.get('speed', 0))

                    if junction_id in junctions:
                        junctions[junction_id]['internal_edges'].append({
                            'edge_id': edge_id,
                            'lane_id': lane.get('id'),
                            'length_m': length_m,
                            'speed_ms': speed_ms,
                            'speed_kmh': speed_ms * 3.6
                        })

    return junctions


def parse_detectors(detector_file: str = 'detectors.add.xml') -> Dict:
    """
    Parse detector information from detectors.add.xml.

    Args:
        detector_file: Path to the detector XML file

    Returns:
        Dictionary with detector information
    """
    tree = ET.parse(detector_file)
    root = tree.getroot()

    detectors = {'induction_loops': [], 'lane_area_detectors': []}

    # Parse induction loops (point detectors)
    for loop in root.findall('inductionLoop'):
        detector_id = loop.get('id')
        lane = loop.get('lane')
        pos = float(loop.get('pos', 0))

        detectors['induction_loops'].append({
            'id': detector_id,
            'lane': lane,
            'pos': pos,
            'type': 'point'
        })

    # Parse lane area detectors (zone detectors)
    for area in root.findall('laneAreaDetector'):
        detector_id = area.get('id')
        lane = area.get('lane')
        pos = float(area.get('pos', 0))
        length = float(area.get('length', 0))

        detectors['lane_area_detectors'].append({
            'id': detector_id,
            'lane': lane,
            'pos': pos,
            'length': length,
            'type': 'zone'
        })

    return detectors


def parse_traffic_lights(xml_file: str = 'Network_TL.net.xml') -> Dict:
    """
    Parse traffic light information from network XML.

    Args:
        xml_file: Path to the network XML file with traffic lights

    Returns:
        Dictionary with traffic light junction IDs
    """
    tree = ET.parse(xml_file)
    root = tree.getroot()

    tl_junctions = {}

    # Find all junctions with type="traffic_light"
    for junction in root.findall('junction'):
        if junction.get('type') == 'traffic_light':
            junction_id = junction.get('id')
            x = float(junction.get('x', 0))
            y = float(junction.get('y', 0))

            tl_junctions[junction_id] = {
                'id': junction_id,
                'x': x,
                'y': y
            }

    return tl_junctions


def calculate_cumulative_positions(df: pd.DataFrame, edges: Dict) -> Dict:
    """
    Calculate cumulative positions for linear schematic layout using categorical visual widths.
    Only mainline and merge segments contribute to cumulative distance.

    Args:
        df: Edge data DataFrame in sequential order
        edges: Full edge data dictionary

    Returns:
        Dictionary with cumulative positions for all edges
    """
    cumulative_pos = {}
    current_pos = 0

    for _, row in df.iterrows():
        edge_id = row['Edge ID']
        category = row['Category']
        length = row['Length (m)']

        # Get schematic visual width
        category_name, visual_width = categorize_length(length)

        # Only mainline and merge segments advance the cumulative position
        if category in ['Mainline', 'Merge']:
            cumulative_pos[edge_id] = {
                'start': current_pos,
                'end': current_pos + visual_width,
                'length': length,  # Actual length
                'visual_width': visual_width,  # Schematic width
                'length_category': category_name,
                'category': category
            }
            current_pos += visual_width
        else:
            # Ramps and acceleration lanes don't contribute to cumulative distance
            # They will be positioned based on their connection to mainline
            cumulative_pos[edge_id] = {
                'start': None,  # Will be calculated based on junction
                'end': None,
                'length': length,  # Actual length
                'visual_width': visual_width,  # Schematic width
                'length_category': category_name,
                'category': category
            }

    return cumulative_pos


def get_junction_positions(junctions: Dict, edges: Dict, df: pd.DataFrame, cumulative_pos: Dict) -> Dict:
    """
    Calculate junction positions on the linear cumulative axis.

    Args:
        junctions: Junction data dictionary
        edges: Edge data dictionary
        df: Edge data DataFrame
        cumulative_pos: Cumulative position dictionary

    Returns:
        Dictionary mapping junction_id to cumulative position
    """
    junction_positions = {}

    # For each junction, find which edges connect to it
    for junc_id, junc_data in junctions.items():
        # Skip dead ends
        if junc_data['type'] == 'dead_end':
            continue

        # Find edges that end at this junction
        for edge_id, edge_data in edges.items():
            if edge_data['to'] == junc_id:
                # Check if this edge is in cumulative_pos and has a position
                if edge_id in cumulative_pos and cumulative_pos[edge_id]['start'] is not None:
                    # Junction is at the end of this edge
                    junction_positions[junc_id] = cumulative_pos[edge_id]['end']
                    break

    return junction_positions


def plot_network_with_junctions(xml_file: str = 'Network.net.xml',
                                detector_file: str = 'detectors.add.xml',
                                output_file: str = 'infrastructure_data/network_with_junctions.png') -> None:
    """
    Create comprehensive network visualization with linear cumulative positioning.
    Matches the layout style of the speed limit schematic.

    Args:
        xml_file: Path to network XML file
        detector_file: Path to detectors XML file
        output_file: Output filename
    """
    # Parse data
    edges = parse_network_xml(xml_file)
    junctions = parse_junctions(xml_file)
    detectors = parse_detectors(detector_file)
    df = create_edge_dataframe(edges)

    # Calculate cumulative positions
    cumulative_pos = calculate_cumulative_positions(df, edges)
    junction_pos = get_junction_positions(junctions, edges, df, cumulative_pos)

    plt.rcParams.update(PLOT_STYLE)
    fig, ax = plt.subplots(figsize=(18, 10))

    # Y-level constants
    y_mainline = 5
    y_ramp = 2

    # Track edge heights for junction sizing and RM junction positions
    edge_heights = {}
    rm_junction_positions = {}

    # Draw mainline and merge segments
    for _, row in df.iterrows():
        edge_id = row['Edge ID']
        category = row['Category']
        speed_kmh = row['Speed (km/h)']
        length_m = row['Length (m)']
        lanes = row['Lanes']

        if category in ['Mainline', 'Merge']:
            start = cumulative_pos[edge_id]['start']
            visual_width = cumulative_pos[edge_id]['visual_width']

            # Calculate lane-based height
            height = lanes * 0.5

            # Store height for junction sizing
            edge_heights[edge_id] = height

            # Determine color
            if category == 'Mainline':
                color = COLOR_SCHEME['mainline']
            elif category == 'Merge':
                color = COLOR_SCHEME['merge']
            else:
                color = 'gray'

            # Draw horizontal bar with schematic width and lane-based height
            ax.barh(y_mainline, visual_width, left=start, height=height,
                   color=color, edgecolor='black', linewidth=1.5, alpha=0.7)

            # Add edge label with actual length
            mid_x = start + visual_width / 2
            label_text = f"{edge_id}\n{speed_kmh:.0f}km/h\n{length_m:.0f}m | {lanes}L"
            ax.text(mid_x, y_mainline, label_text,
                   ha='center', va='center', fontsize=7, weight='bold')

    # Draw vertical ramps and acceleration lanes
    # Three-pass approach: acceleration lanes → on-ramps → off-ramps

    # First pass: Draw acceleration lanes and calculate RM junction positions
    for acceleration_edge_id in ['E34_THA_ACC', 'E35_HOR_ACC', 'E36_WAED_ACC']:
        if acceleration_edge_id in edges:
            edge_data = edges[acceleration_edge_id]
            from_junc = edge_data['from']  # RM junction
            to_junc = edge_data['to']      # EN junction

            # Get EN junction position (on mainline)
            if to_junc in junction_pos:
                en_x = junction_pos[to_junc]
                en_y_top = y_mainline  # Top of mainline

                # Use minimum length category for vertical edges (reduced to 25%)
                length_m = edge_data['length_m']
                visual_width = 12.5  # 25% of minimum category

                # Calculate RM junction position (at bottom of acc lane)
                rm_y = en_y_top - visual_width
                rm_junction_positions[from_junc] = {'x': en_x, 'y': rm_y}

                # Draw acceleration lane (vertical orange rectangle)
                width = edge_data['num_lanes'] * 0.5
                edge_heights[acceleration_edge_id] = width
                rect = Rectangle((en_x - width/2, rm_y), width, visual_width,
                               facecolor=COLOR_SCHEME['acceleration'],
                               edgecolor='black', linewidth=1.5, alpha=0.7)
                ax.add_patch(rect)

                # Add label
                label_y = en_y_top - visual_width/2
                speed_kmh = edge_data['speed_kmh']
                label_text = f"{acceleration_edge_id}\n{speed_kmh:.0f}km/h\n{length_m:.0f}m"
                ax.text(en_x, label_y, label_text,
                       ha='center', va='center', fontsize=7, weight='bold',
                       bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))

    # Second pass: Draw on-ramps extending from RM junctions
    for onramp_edge_id in ['E34_THA', 'E35_HOR', 'E36_WAED']:
        if onramp_edge_id in edges:
            edge_data = edges[onramp_edge_id]
            to_junc = edge_data['to']  # RM junction

            # Get RM junction position
            if to_junc in rm_junction_positions:
                rm_x = rm_junction_positions[to_junc]['x']
                rm_y = rm_junction_positions[to_junc]['y']

                # Use minimum length category for vertical edges (reduced to 25%)
                length_m = edge_data['length_m']
                visual_width = 12.5  # 25% of minimum category

                # Draw on-ramp (vertical green rectangle extending downward)
                width = edge_data['num_lanes'] * 0.5
                edge_heights[onramp_edge_id] = width
                bottom_y = rm_y - visual_width
                rect = Rectangle((rm_x - width/2, bottom_y), width, visual_width,
                               facecolor=COLOR_SCHEME['on_ramp'],
                               edgecolor='black', linewidth=1.5, alpha=0.7)
                ax.add_patch(rect)

                # Add label
                label_y = rm_y - visual_width/2
                speed_kmh = edge_data['speed_kmh']
                label_text = f"{onramp_edge_id}\n{speed_kmh:.0f}km/h\n{length_m:.0f}m"
                ax.text(rm_x, label_y, label_text,
                       ha='center', va='center', fontsize=7, weight='bold',
                       bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))

    # Third pass: Draw off-ramps from EX junctions
    for _, row in df.iterrows():
        edge_id = row['Edge ID']
        category = row['Category']

        if category == 'Off-ramp':  # A34_THA, A35_HOR, A36_WAED
            edge_data = edges[edge_id]
            from_junc = edge_data['from']  # EX junction

            if from_junc in junction_pos:
                ex_x = junction_pos[from_junc]
                ex_y = y_mainline

                # Use minimum length category for vertical edges (reduced to 25%)
                length_m = edge_data['length_m']
                visual_width = 12.5  # 25% of minimum category

                # Draw off-ramp (vertical red rectangle extending downward)
                width = edge_data['num_lanes'] * 0.5
                edge_heights[edge_id] = width
                bottom_y = ex_y - visual_width
                rect = Rectangle((ex_x - width/2, bottom_y), width, visual_width,
                               facecolor=COLOR_SCHEME['off_ramp'],
                               edgecolor='black', linewidth=1.5, alpha=0.7)
                ax.add_patch(rect)

                # Add label
                label_y = ex_y - visual_width/2
                speed_kmh = edge_data['speed_kmh']
                label_text = f"{edge_id}\n{speed_kmh:.0f}km/h\n{length_m:.0f}m"
                ax.text(ex_x, label_y, label_text,
                       ha='center', va='center', fontsize=7, weight='bold',
                       bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))

    # Draw RM junctions
    for rm_junc_id, pos_data in rm_junction_positions.items():
        x = pos_data['x']
        y = pos_data['y']

        # Junction width = 0.5 (one lane)
        rm_junction_width = 0.5
        rm_junction_height = 0.5  # Small height for RM junctions

        # Color: orange for priority junctions (no traffic lights in this version)
        color = 'orange'
        alpha = 0.7

        rect = Rectangle((x - rm_junction_width/2, y - rm_junction_height/2),
                       rm_junction_width, rm_junction_height,
                       facecolor=color, edgecolor='black', linewidth=2, alpha=alpha, zorder=10)
        ax.add_patch(rect)

        # Add label
        ax.text(x, y + rm_junction_height/2 + 0.3, rm_junc_id,
               fontsize=6, ha='center', va='bottom', weight='bold',
               bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8))

    # Draw junctions as rectangles with adaptive heights based on connected edges
    junction_width = 0.5  # Changed from 15 to match one lane width
    for junc_id, position in junction_pos.items():
        if junc_id in junctions:
            junc_type = junctions[junc_id]['type']

            # Calculate junction height as max of adjacent edges
            junction_height = get_junction_height(junc_id, edges, edge_heights)

            # Color based on type
            if junc_type == 'traffic_light':
                color = 'red'
                alpha = 0.9
            elif junc_type == 'priority':
                color = 'orange'
                alpha = 0.7
            elif junc_type == 'unregulated':
                color = 'gray'
                alpha = 0.6
            else:
                continue

            rect = Rectangle((position - junction_width/2, y_mainline - junction_height/2),
                           junction_width, junction_height,
                           facecolor=color, edgecolor='black', linewidth=2, alpha=alpha, zorder=10)
            ax.add_patch(rect)

            # Add small junction label
            ax.text(position, y_mainline + junction_height/2 + 0.3, junc_id,
                   fontsize=6, ha='center', va='bottom', weight='bold',
                   bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8))

    # Draw detectors
    for loop in detectors['induction_loops']:
        lane = loop['lane']
        pos = loop['pos']
        detector_id = loop['id']

        # Extract edge ID from lane
        edge_id = lane.rsplit('_', 1)[0]

        if edge_id in cumulative_pos and cumulative_pos[edge_id]['start'] is not None:
            # Calculate detector position
            if pos < 0:
                # Negative pos means distance from end
                detector_cumulative_pos = cumulative_pos[edge_id]['end'] + pos
            else:
                detector_cumulative_pos = cumulative_pos[edge_id]['start'] + pos

            # Plot detector
            ax.plot(detector_cumulative_pos, y_mainline, marker='D', markersize=8,
                   color='cyan', markeredgecolor='black', markeredgewidth=1.5, zorder=5)

            # Add detector label
            ax.text(detector_cumulative_pos, y_mainline + 1.2, detector_id,
                   fontsize=5, ha='center', va='bottom', rotation=90,
                   bbox=dict(boxstyle='round,pad=0.2', facecolor='cyan', alpha=0.6))

    # Zone detectors on on-ramps
    for area in detectors['lane_area_detectors']:
        lane = area['lane']
        detector_id = area['id']

        # Extract edge ID from lane
        edge_id = lane.rsplit('_', 1)[0]

        # Check if this is an on-ramp edge
        if edge_id in ['E34_THA', 'E35_HOR', 'E36_WAED'] and edge_id in edges:
            # Get corresponding RM junction
            rm_junc = edges[edge_id]['to']

            if rm_junc in rm_junction_positions:
                x = rm_junction_positions[rm_junc]['x']
                y = rm_junction_positions[rm_junc]['y']  # Top of on-ramp

                # Plot detector at top of on-ramp
                ax.plot(x, y, marker='^', markersize=10,
                       color='lime', markeredgecolor='black', markeredgewidth=1.5, zorder=5)

                # Add detector label
                ax.text(x + 1.5, y, detector_id,
                       fontsize=5, ha='left', va='center',
                       bbox=dict(boxstyle='round,pad=0.2', facecolor='lime', alpha=0.6))

    # Create legend
    legend_elements = [
        mpatches.Patch(color=COLOR_SCHEME['mainline'], label='Mainline A3', alpha=0.7),
        mpatches.Patch(color=COLOR_SCHEME['merge'], label='Merge Sections', alpha=0.7),
        mpatches.Patch(color=COLOR_SCHEME['on_ramp'], label='On-ramps', alpha=0.7),
        mpatches.Patch(color=COLOR_SCHEME['acceleration'], label='Acceleration Lanes', alpha=0.7),
        mpatches.Patch(color=COLOR_SCHEME['off_ramp'], label='Off-ramps', alpha=0.7),
        mpatches.Patch(color='orange', label='Priority Junction', alpha=0.7),
        mpatches.Patch(color='gray', label='Unregulated Junction', alpha=0.6),
        plt.Line2D([0], [0], marker='D', color='w', markerfacecolor='cyan',
                  markeredgecolor='black', markersize=8, label='Induction Loop'),
        plt.Line2D([0], [0], marker='^', color='w', markerfacecolor='lime',
                  markeredgecolor='black', markersize=10, label='Zone Detector'),
    ]
    ax.legend(handles=legend_elements, loc='upper left', framealpha=0.9, fontsize=10)

    # Formatting
    max_pos = max([p['end'] for p in cumulative_pos.values() if p['end'] is not None])

    # Calculate minimum y to accommodate vertical ramps
    # Find the lowest point (on-ramps extend furthest down)
    min_y = 0
    if rm_junction_positions:
        # Find the lowest RM junction and subtract the vertical ramp extent
        min_rm_y = min(pos['y'] for pos in rm_junction_positions.values())
        # All vertical edges use reduced height (25% of minimum category)
        vertical_ramp_extent = 12.5  # 25% of minimum category
        min_y = min_rm_y - vertical_ramp_extent - 2  # Add margin

    ax.set_xlim(-500, max_pos + 500)
    ax.set_ylim(min_y, 8)
    ax.set_xlabel('Network Topology', fontsize=12, weight='bold')
    ax.set_yticks([])
    ax.set_title('Network Infrastructure - Complete Visualization\nSUMO Network: A3 Highway (WAE → HOR → THA)',
                fontsize=14, weight='bold', pad=20)
    ax.grid(axis='x', alpha=0.3)
    ax.set_axisbelow(True)

    plt.tight_layout()
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"[OK] Network visualization saved: {output_file}")

    # Also save as PDF
    pdf_file = output_file.replace('.png', '.pdf')
    plt.savefig(pdf_file, format='pdf', bbox_inches='tight')
    print(f"[OK] Network visualization PDF saved: {pdf_file}")

    plt.close()


def plot_network_with_junctions_and_tl(xml_file: str = 'Network_TL.net.xml',
                                       detector_file: str = 'detectors.add.xml',
                                       output_file: str = 'infrastructure_data/network_with_junctions_and_tl.png') -> None:
    """
    Create comprehensive network visualization with linear cumulative positioning and traffic lights.
    Matches the layout style of the speed limit schematic.

    Args:
        xml_file: Path to network XML file with traffic lights
        detector_file: Path to detectors XML file
        output_file: Output filename
    """
    # Parse data
    edges = parse_network_xml(xml_file)
    junctions = parse_junctions(xml_file)
    detectors = parse_detectors(detector_file)
    tl_junctions = parse_traffic_lights(xml_file)
    df = create_edge_dataframe(edges)

    # Calculate cumulative positions
    cumulative_pos = calculate_cumulative_positions(df, edges)
    junction_pos = get_junction_positions(junctions, edges, df, cumulative_pos)

    plt.rcParams.update(PLOT_STYLE)
    fig, ax = plt.subplots(figsize=(18, 10))

    # Y-level constants
    y_mainline = 5
    y_ramp = 2

    # Track edge heights for junction sizing and RM junction positions
    edge_heights = {}
    rm_junction_positions = {}

    # Draw mainline and merge segments
    for _, row in df.iterrows():
        edge_id = row['Edge ID']
        category = row['Category']
        speed_kmh = row['Speed (km/h)']
        length_m = row['Length (m)']
        lanes = row['Lanes']

        if category in ['Mainline', 'Merge']:
            start = cumulative_pos[edge_id]['start']
            visual_width = cumulative_pos[edge_id]['visual_width']

            # Calculate lane-based height
            height = lanes * 0.5

            # Store height for junction sizing
            edge_heights[edge_id] = height

            # Determine color
            if category == 'Mainline':
                color = COLOR_SCHEME['mainline']
            elif category == 'Merge':
                color = COLOR_SCHEME['merge']
            else:
                color = 'gray'

            # Draw horizontal bar with schematic width and lane-based height
            ax.barh(y_mainline, visual_width, left=start, height=height,
                   color=color, edgecolor='black', linewidth=1.5, alpha=0.7)

            # Add edge label with actual length
            mid_x = start + visual_width / 2
            label_text = f"{edge_id}\n{speed_kmh:.0f}km/h\n{length_m:.0f}m | {lanes}L"
            ax.text(mid_x, y_mainline, label_text,
                   ha='center', va='center', fontsize=7, weight='bold')

    # Draw vertical ramps and acceleration lanes
    # Three-pass approach: acceleration lanes → on-ramps → off-ramps

    # First pass: Draw acceleration lanes and calculate RM junction positions
    for acceleration_edge_id in ['E34_THA_ACC', 'E35_HOR_ACC', 'E36_WAED_ACC']:
        if acceleration_edge_id in edges:
            edge_data = edges[acceleration_edge_id]
            from_junc = edge_data['from']  # RM junction
            to_junc = edge_data['to']      # EN junction

            # Get EN junction position (on mainline)
            if to_junc in junction_pos:
                en_x = junction_pos[to_junc]
                en_y_top = y_mainline  # Top of mainline

                # Use minimum length category for vertical edges (reduced to 25%)
                length_m = edge_data['length_m']
                visual_width = 12.5  # 25% of minimum category

                # Calculate RM junction position (at bottom of acc lane)
                rm_y = en_y_top - visual_width
                rm_junction_positions[from_junc] = {'x': en_x, 'y': rm_y}

                # Draw acceleration lane (vertical orange rectangle)
                width = edge_data['num_lanes'] * 0.5
                edge_heights[acceleration_edge_id] = width
                rect = Rectangle((en_x - width/2, rm_y), width, visual_width,
                               facecolor=COLOR_SCHEME['acceleration'],
                               edgecolor='black', linewidth=1.5, alpha=0.7)
                ax.add_patch(rect)

                # Add label
                label_y = en_y_top - visual_width/2
                speed_kmh = edge_data['speed_kmh']
                label_text = f"{acceleration_edge_id}\n{speed_kmh:.0f}km/h\n{length_m:.0f}m"
                ax.text(en_x, label_y, label_text,
                       ha='center', va='center', fontsize=7, weight='bold',
                       bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))

    # Second pass: Draw on-ramps extending from RM junctions
    for onramp_edge_id in ['E34_THA', 'E35_HOR', 'E36_WAED']:
        if onramp_edge_id in edges:
            edge_data = edges[onramp_edge_id]
            to_junc = edge_data['to']  # RM junction

            # Get RM junction position
            if to_junc in rm_junction_positions:
                rm_x = rm_junction_positions[to_junc]['x']
                rm_y = rm_junction_positions[to_junc]['y']

                # Use minimum length category for vertical edges (reduced to 25%)
                length_m = edge_data['length_m']
                visual_width = 12.5  # 25% of minimum category

                # Draw on-ramp (vertical green rectangle extending downward)
                width = edge_data['num_lanes'] * 0.5
                edge_heights[onramp_edge_id] = width
                bottom_y = rm_y - visual_width
                rect = Rectangle((rm_x - width/2, bottom_y), width, visual_width,
                               facecolor=COLOR_SCHEME['on_ramp'],
                               edgecolor='black', linewidth=1.5, alpha=0.7)
                ax.add_patch(rect)

                # Add label
                label_y = rm_y - visual_width/2
                speed_kmh = edge_data['speed_kmh']
                label_text = f"{onramp_edge_id}\n{speed_kmh:.0f}km/h\n{length_m:.0f}m"
                ax.text(rm_x, label_y, label_text,
                       ha='center', va='center', fontsize=7, weight='bold',
                       bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))

    # Third pass: Draw off-ramps from EX junctions
    for _, row in df.iterrows():
        edge_id = row['Edge ID']
        category = row['Category']

        if category == 'Off-ramp':  # A34_THA, A35_HOR, A36_WAED
            edge_data = edges[edge_id]
            from_junc = edge_data['from']  # EX junction

            if from_junc in junction_pos:
                ex_x = junction_pos[from_junc]
                ex_y = y_mainline

                # Use minimum length category for vertical edges (reduced to 25%)
                length_m = edge_data['length_m']
                visual_width = 12.5  # 25% of minimum category

                # Draw off-ramp (vertical red rectangle extending downward)
                width = edge_data['num_lanes'] * 0.5
                edge_heights[edge_id] = width
                bottom_y = ex_y - visual_width
                rect = Rectangle((ex_x - width/2, bottom_y), width, visual_width,
                               facecolor=COLOR_SCHEME['off_ramp'],
                               edgecolor='black', linewidth=1.5, alpha=0.7)
                ax.add_patch(rect)

                # Add label
                label_y = ex_y - visual_width/2
                speed_kmh = edge_data['speed_kmh']
                label_text = f"{edge_id}\n{speed_kmh:.0f}km/h\n{length_m:.0f}m"
                ax.text(ex_x, label_y, label_text,
                       ha='center', va='center', fontsize=7, weight='bold',
                       bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))

    # Draw RM junctions
    for rm_junc_id, pos_data in rm_junction_positions.items():
        x = pos_data['x']
        y = pos_data['y']

        # Junction width = 0.5 (one lane)
        rm_junction_width = 0.5
        rm_junction_height = 0.5  # Small height for RM junctions

        # Color: red for traffic light junctions, orange for priority
        if rm_junc_id in tl_junctions:
            color = 'red'
            alpha = 0.9
        else:
            color = 'orange'
            alpha = 0.7

        rect = Rectangle((x - rm_junction_width/2, y - rm_junction_height/2),
                       rm_junction_width, rm_junction_height,
                       facecolor=color, edgecolor='black', linewidth=2, alpha=alpha, zorder=10)
        ax.add_patch(rect)

        # Add label
        ax.text(x, y + rm_junction_height/2 + 0.3, rm_junc_id,
               fontsize=6, ha='center', va='bottom', weight='bold',
               bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8))

    # Draw junctions as rectangles with adaptive heights based on connected edges
    junction_width = 0.5  # Changed from 15 to match one lane width
    for junc_id, position in junction_pos.items():
        if junc_id in junctions:
            junc_type = junctions[junc_id]['type']

            # Calculate junction height as max of adjacent edges
            junction_height = get_junction_height(junc_id, edges, edge_heights)

            # Color based on type (traffic_light takes priority)
            if junc_type == 'traffic_light':
                color = 'red'
                alpha = 0.9
            elif junc_type == 'priority':
                color = 'orange'
                alpha = 0.7
            elif junc_type == 'unregulated':
                color = 'gray'
                alpha = 0.6
            else:
                continue

            rect = Rectangle((position - junction_width/2, y_mainline - junction_height/2),
                           junction_width, junction_height,
                           facecolor=color, edgecolor='black', linewidth=2, alpha=alpha, zorder=10)
            ax.add_patch(rect)

            # Add small junction label
            ax.text(position, y_mainline + junction_height/2 + 0.3, junc_id,
                   fontsize=6, ha='center', va='bottom', weight='bold',
                   bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8))

    # Draw traffic light icons above RM junctions
    for tl_id in tl_junctions.keys():
        if tl_id in rm_junction_positions:
            x = rm_junction_positions[tl_id]['x']
            y = rm_junction_positions[tl_id]['y']

            # Plot traffic light icon (circle with TL symbol) above the RM junction
            circle = plt.Circle((x, y + 1.5), 0.6, color='red',
                              edgecolor='black', linewidth=2, alpha=0.9, zorder=11)
            ax.add_patch(circle)
            ax.text(x, y + 1.5, 'TL', fontsize=8,
                   ha='center', va='center', weight='bold', color='white', zorder=12)

    # Draw detectors
    for loop in detectors['induction_loops']:
        lane = loop['lane']
        pos = loop['pos']
        detector_id = loop['id']

        # Extract edge ID from lane
        edge_id = lane.rsplit('_', 1)[0]

        if edge_id in cumulative_pos and cumulative_pos[edge_id]['start'] is not None:
            # Calculate detector position
            if pos < 0:
                # Negative pos means distance from end
                detector_cumulative_pos = cumulative_pos[edge_id]['end'] + pos
            else:
                detector_cumulative_pos = cumulative_pos[edge_id]['start'] + pos

            # Plot detector
            ax.plot(detector_cumulative_pos, y_mainline, marker='D', markersize=8,
                   color='cyan', markeredgecolor='black', markeredgewidth=1.5, zorder=5)

            # Add detector label
            ax.text(detector_cumulative_pos, y_mainline + 1.2, detector_id,
                   fontsize=5, ha='center', va='bottom', rotation=90,
                   bbox=dict(boxstyle='round,pad=0.2', facecolor='cyan', alpha=0.6))

    # Zone detectors on on-ramps
    for area in detectors['lane_area_detectors']:
        lane = area['lane']
        detector_id = area['id']

        # Extract edge ID from lane
        edge_id = lane.rsplit('_', 1)[0]

        # Check if this is an on-ramp edge
        if edge_id in ['E34_THA', 'E35_HOR', 'E36_WAED'] and edge_id in edges:
            # Get corresponding RM junction
            rm_junc = edges[edge_id]['to']

            if rm_junc in rm_junction_positions:
                x = rm_junction_positions[rm_junc]['x']
                y = rm_junction_positions[rm_junc]['y']  # Top of on-ramp

                # Plot detector at top of on-ramp
                ax.plot(x, y, marker='^', markersize=10,
                       color='lime', markeredgecolor='black', markeredgewidth=1.5, zorder=5)

                # Add detector label
                ax.text(x + 1.5, y, detector_id,
                       fontsize=5, ha='left', va='center',
                       bbox=dict(boxstyle='round,pad=0.2', facecolor='lime', alpha=0.6))

    # Create legend
    legend_elements = [
        mpatches.Patch(color=COLOR_SCHEME['mainline'], label='Mainline A3', alpha=0.7),
        mpatches.Patch(color=COLOR_SCHEME['merge'], label='Merge Sections', alpha=0.7),
        mpatches.Patch(color=COLOR_SCHEME['on_ramp'], label='On-ramps', alpha=0.7),
        mpatches.Patch(color=COLOR_SCHEME['acceleration'], label='Acceleration Lanes', alpha=0.7),
        mpatches.Patch(color=COLOR_SCHEME['off_ramp'], label='Off-ramps', alpha=0.7),
        mpatches.Patch(color='red', label='Traffic Light Junction', alpha=0.9),
        mpatches.Patch(color='orange', label='Priority Junction', alpha=0.7),
        mpatches.Patch(color='gray', label='Unregulated Junction', alpha=0.6),
        plt.Line2D([0], [0], marker='D', color='w', markerfacecolor='cyan',
                  markeredgecolor='black', markersize=8, label='Induction Loop'),
        plt.Line2D([0], [0], marker='^', color='w', markerfacecolor='lime',
                  markeredgecolor='black', markersize=10, label='Zone Detector'),
        plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='red',
                  markersize=10, label='Traffic Light', markeredgecolor='black'),
    ]
    ax.legend(handles=legend_elements, loc='upper left', framealpha=0.9, fontsize=10)

    # Formatting
    max_pos = max([p['end'] for p in cumulative_pos.values() if p['end'] is not None])

    # Calculate minimum y to accommodate vertical ramps
    # Find the lowest point (on-ramps extend furthest down)
    min_y = 0
    if rm_junction_positions:
        # Find the lowest RM junction and subtract the vertical ramp extent
        min_rm_y = min(pos['y'] for pos in rm_junction_positions.values())
        # All vertical edges use reduced height (25% of minimum category)
        vertical_ramp_extent = 12.5  # 25% of minimum category
        min_y = min_rm_y - vertical_ramp_extent - 2  # Add margin

    ax.set_xlim(-500, max_pos + 500)
    ax.set_ylim(min_y, 8)
    ax.set_xlabel('Network Topology', fontsize=12, weight='bold')
    ax.set_yticks([])
    ax.set_title('Network Infrastructure with Traffic Lights\nSUMO Network: A3 Highway (WAE → HOR → THA)',
                fontsize=14, weight='bold', pad=20)
    ax.grid(axis='x', alpha=0.3)
    ax.set_axisbelow(True)

    plt.tight_layout()
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"[OK] Network visualization with traffic lights saved: {output_file}")

    # Also save as PDF
    pdf_file = output_file.replace('.png', '.pdf')
    plt.savefig(pdf_file, format='pdf', bbox_inches='tight')
    print(f"[OK] Network visualization with traffic lights PDF saved: {pdf_file}")

    plt.close()


def main():
    """Main execution function."""
    print("\n" + "="*100)
    print("NETWORK INFRASTRUCTURE ANALYSIS")
    print("="*100)

    # Step 1: Parse XML files
    print("\n[1/6] Parsing Network.net.xml...")
    try:
        edges = parse_network_xml('Network.net.xml')
        print(f"[OK] Successfully parsed {len(edges)} edges")
    except FileNotFoundError:
        print("[ERROR] Network.net.xml not found in current directory")
        return
    except Exception as e:
        print(f"[ERROR] Error parsing XML: {e}")
        return

    print("\n[2/6] Parsing junctions and detectors...")
    try:
        junctions = parse_junctions('Network.net.xml')
        print(f"[OK] Successfully parsed {len(junctions)} junctions")

        detectors = parse_detectors('detectors.add.xml')
        num_loops = len(detectors.get('induction_loops', []))
        num_areas = len(detectors.get('lane_area_detectors', []))
        print(f"[OK] Successfully parsed {num_loops} induction loops and {num_areas} zone detectors")
    except FileNotFoundError as e:
        print(f"[WARNING] Detector file not found: {e}")
        detectors = None
    except Exception as e:
        print(f"[ERROR] Error parsing junctions/detectors: {e}")
        detectors = None

    # Step 3: Create DataFrames
    print("\n[3/6] Creating edge and junction dataframes...")
    df_edges = create_edge_dataframe(edges, detectors)
    print(f"[OK] Edge list created with {len(df_edges)} entries")

    df_junctions = create_junction_dataframe(junctions, edges)
    print(f"[OK] Junction list created with {len(df_junctions)} entries")

    # Step 4: Print edge list
    print("\n[4/6] Displaying edge list...")
    print_edge_list(df_edges)

    # Step 5: Save to Excel with multiple sheets
    os.makedirs('infrastructure_data', exist_ok=True)
    excel_file = 'infrastructure_data/network_infrastructure_list.xlsx'

    with pd.ExcelWriter(excel_file, engine='openpyxl') as writer:
        df_edges.to_excel(writer, index=False, sheet_name='Edges')
        df_junctions.to_excel(writer, index=False, sheet_name='Junctions')

    print(f"[OK] Infrastructure data saved to: {excel_file}")
    print(f"     - Sheet 'Edges': {len(df_edges)} edges with detector information")
    print(f"     - Sheet 'Junctions': {len(df_junctions)} junctions")

    # Step 6: Create network visualization with junctions (Network.net.xml)
    print("\n[5/6] Creating network visualization with junctions (Network.net.xml)...")
    try:
        plot_network_with_junctions('Network.net.xml', 'detectors.add.xml',
                                   'infrastructure_data/network_with_junctions.png')
    except Exception as e:
        print(f"[ERROR] Error creating network visualization: {e}")
        import traceback
        traceback.print_exc()

    # Step 7: Create network visualization with traffic lights (Network_TL.net.xml)
    print("\n[6/6] Creating network visualization with traffic lights (Network_TL.net.xml)...")
    try:
        plot_network_with_junctions_and_tl('Network_TL.net.xml', 'detectors.add.xml',
                                          'infrastructure_data/network_with_junctions_and_tl.png')
    except Exception as e:
        print(f"[ERROR] Error creating network visualization with traffic lights: {e}")
        import traceback
        traceback.print_exc()

    print("\n" + "="*100)
    print("ANALYSIS COMPLETE")
    print("="*100)
    print("\nGenerated files:")
    print("  - infrastructure_data/network_infrastructure_list.xlsx (Edges + Junctions)")
    print("  - infrastructure_data/network_with_junctions.png")
    print("  - infrastructure_data/network_with_junctions.pdf")
    print("  - infrastructure_data/network_with_junctions_and_tl.png")
    print("  - infrastructure_data/network_with_junctions_and_tl.pdf")
    print()


if __name__ == '__main__':
    main()
