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
from matplotlib.patches import Rectangle
import pandas as pd
import numpy as np
from typing import Dict, Tuple
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
    'mainline': '#1f77b4',
    'on_ramp': '#2ca02c',
    'off_ramp': '#d62728',
    'acceleration': '#ff7f0e',
    'merge': '#ff9896',
}

# Fixed dimensions for visualization
HORIZONTAL_EDGE_WIDTH = 200
LANE_HEIGHT = 25  # Halved from 50 to 25
VERTICAL_EDGE_HEIGHT = 100
LANE_WIDTH = 50


def parse_network_xml(xml_file: str = 'shared_simulation_files/Network.net.xml') -> Dict:
    """Parse SUMO network XML file and extract edge information."""
    tree = ET.parse(xml_file)
    root = tree.getroot()
    edges_data = {}

    for edge in root.findall('edge'):
        edge_id = edge.get('id')
        if edge_id.startswith(':'):
            continue

        lanes = edge.findall('lane')
        if lanes:
            first_lane = lanes[0]
            speed_ms = float(first_lane.get('speed', 0))
            length_m = float(first_lane.get('length', 0))  # <-- LENGTH EXTRACTED HERE
        else:
            speed_ms = 0
            length_m = 0

        edges_data[edge_id] = {
            'id': edge_id,
            'from': edge.get('from', ''),
            'to': edge.get('to', ''),
            'priority': edge.get('priority', ''),
            'num_lanes': len(lanes),
            'speed_ms': speed_ms,
            'speed_kmh': speed_ms * 3.6,
            'length_m': length_m,  # <-- STORED IN EDGES DICTIONARY
        }

    return edges_data


def parse_junctions(xml_file: str = 'shared_simulation_files/Network.net.xml') -> Dict:
    """Parse junction information from SUMO network XML."""
    tree = ET.parse(xml_file)
    root = tree.getroot()
    junctions = {}

    for junction in root.findall('junction'):
        junction_id = junction.get('id')
        junctions[junction_id] = {
            'id': junction_id,
            'x': float(junction.get('x', 0)),
            'y': float(junction.get('y', 0)),
            'type': junction.get('type'),
            'internal_edges': []
        }

    for edge in root.findall('edge'):
        edge_id = edge.get('id')
        if edge_id.startswith(':'):
            parts = edge_id[1:].split('_')
            if len(parts) >= 2:
                junction_id = '_'.join(parts[:-1])
                lanes = edge.findall('lane')
                if lanes and junction_id in junctions:
                    for lane in lanes:
                        junctions[junction_id]['internal_edges'].append({
                            'edge_id': edge_id,
                            'lane_id': lane.get('id'),
                            'length_m': float(lane.get('length', 0)),
                            'speed_ms': float(lane.get('speed', 0)),
                            'speed_kmh': float(lane.get('speed', 0)) * 3.6
                        })

    return junctions


def parse_detectors(detector_file: str = 'shared_simulation_files/detectors.add.xml') -> Dict:
    """Parse detector information from detectors.add.xml."""
    tree = ET.parse(detector_file)
    root = tree.getroot()
    detectors = {'induction_loops': [], 'lane_area_detectors': []}

    for loop in root.findall('inductionLoop'):
        detectors['induction_loops'].append({
            'id': loop.get('id'),
            'lane': loop.get('lane'),
            'pos': float(loop.get('pos', 0)),
            'type': 'point'
        })

    for area in root.findall('laneAreaDetector'):
        detectors['lane_area_detectors'].append({
            'id': area.get('id'),
            'lane': area.get('lane'),
            'pos': float(area.get('pos', 0)),
            'length': float(area.get('length', 0)),
            'type': 'zone'
        })

    return detectors


def parse_traffic_lights(xml_file: str = 'shared_simulation_files/Network_TL.net.xml') -> Dict:
    """Parse traffic light information from network XML."""
    tree = ET.parse(xml_file)
    root = tree.getroot()
    tl_junctions = {}

    for junction in root.findall('junction'):
        if junction.get('type') == 'traffic_light':
            junction_id = junction.get('id')
            tl_junctions[junction_id] = {
                'id': junction_id,
                'x': float(junction.get('x', 0)),
                'y': float(junction.get('y', 0))
            }

    return tl_junctions


def categorize_edge(edge_id: str) -> str:
    """Categorize edge based on its ID pattern."""
    if edge_id.startswith('A3_'):
        return 'Merge' if '_MERG' in edge_id else 'Mainline'
    elif edge_id.startswith('E3') and 'ACC' in edge_id:
        return 'Acceleration'
    elif edge_id.startswith('E3'):
        return 'On-ramp'
    elif edge_id.startswith('A3') and edge_id[2] in ['4', '5', '6']:
        return 'Off-ramp'
    return 'Other'


def get_section(edge_id: str) -> str:
    """Get geographic section (THA, HOR, WAED) from edge ID."""
    if 'THA' in edge_id:
        return 'THA'
    elif 'HOR' in edge_id:
        return 'HOR'
    elif 'WAE' in edge_id:
        return 'WAED'
    return 'Unknown'


def create_edge_dataframe(edges: Dict, detectors: Dict = None) -> pd.DataFrame:
    """Create a pandas DataFrame from edges dictionary with detector information."""
    sequential_order = [
        'A3_WAED_S', 'A3_WAED_MID', 'E36_WAED', 'E36_WAED_ACC', 'A3_WAED_MERG', 'A36_WAED',
        'A3_HOR_S', 'A3_HOR_MID', 'E35_HOR', 'E35_HOR_ACC', 'A3_HOR_MERG', 'A35_HOR',
        'A3_THA_S', 'A3_THA_MID', 'E34_THA', 'E34_THA_ACC', 'A3_THA_MERG', 'A34_THA', 'A3_THA_N',
    ]

    detector_map = {}
    if detectors:
        for loop in detectors.get('induction_loops', []):
            edge_id = loop['lane'].rsplit('_', 1)[0]
            if edge_id not in detector_map:
                detector_map[edge_id] = []
            detector_map[edge_id].append({'id': loop['id'], 'type': 'Induction Loop'})

        for area in detectors.get('lane_area_detectors', []):
            edge_id = area['lane'].rsplit('_', 1)[0]
            if edge_id not in detector_map:
                detector_map[edge_id] = []
            detector_map[edge_id].append({'id': area['id'], 'type': 'Zone Detector'})

    data_list = []
    for edge_id in sequential_order:
        if edge_id in edges:
            edge_data = edges[edge_id]
            detector_ids = []
            detector_types = []
            if edge_id in detector_map:
                for det in detector_map[edge_id]:
                    detector_ids.append(det['id'])
                    detector_types.append(det['type'])

            data_list.append({
                'Edge ID': edge_id,
                'Category': categorize_edge(edge_id),
                'Section': get_section(edge_id),
                'From -> To': f"{edge_data['from']} -> {edge_data['to']}",
                'Lanes': edge_data['num_lanes'],
                'Speed (m/s)': round(edge_data['speed_ms'], 2),
                'Speed (km/h)': round(edge_data['speed_kmh'], 1),
                'Length (m)': round(edge_data['length_m'], 2),
                'Priority': edge_data['priority'],
                'Detector ID': ', '.join(detector_ids) if detector_ids else '',
                'Detector Type': ', '.join(detector_types) if detector_types else '',
            })

    return pd.DataFrame(data_list)


def create_junction_dataframe(junctions: Dict, edges: Dict) -> pd.DataFrame:
    """Create a pandas DataFrame from junctions dictionary with internal lane data."""
    data_list = []

    for junc_id, junc_data in junctions.items():
        if junc_id.startswith(':'):
            continue

        incoming_edges = [e_id for e_id, e in edges.items() if e['to'] == junc_id]
        outgoing_edges = [e_id for e_id, e in edges.items() if e['from'] == junc_id]
        internal_edges = junc_data.get('internal_edges', [])

        if internal_edges:
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
                    'Incoming Edges': ', '.join(incoming_edges),
                    'Outgoing Edges': ', '.join(outgoing_edges),
                    'Total Connections': len(incoming_edges) + len(outgoing_edges),
                })
        else:
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
                'Incoming Edges': ', '.join(incoming_edges),
                'Outgoing Edges': ', '.join(outgoing_edges),
                'Total Connections': len(incoming_edges) + len(outgoing_edges),
            })

    df = pd.DataFrame(data_list)
    return df.sort_values(['Junction ID', 'Internal Lane']).reset_index(drop=True)


def get_junction_positions(junctions: Dict, edges: Dict, cumulative_pos: Dict) -> Dict:
    """Calculate junction positions on the linear cumulative axis."""
    junction_positions = {}
    for junc_id, junc_data in junctions.items():
        if junc_data['type'] == 'dead_end':
            continue
        for edge_id, edge_data in edges.items():
            if edge_data['to'] == junc_id and edge_id in cumulative_pos and cumulative_pos[edge_id]['start'] is not None:
                junction_positions[junc_id] = cumulative_pos[edge_id]['end']
                break
    return junction_positions


def get_junction_height(junction_id: str, edges: Dict, heights_map: Dict) -> float:
    """Calculate junction height as maximum of adjacent edges."""
    connected_edges = [e_id for e_id, e in edges.items() if e['from'] == junction_id or e['to'] == junction_id]
    return max([heights_map.get(e, 0.5) for e in connected_edges]) if connected_edges else 0.5


def plot_network_infrastructure(xml_file: str, detector_file: str, output_file: str, 
                                with_traffic_lights: bool = False) -> None:
    """
    Create comprehensive network visualization with linear positioning.
    
    Args:
        xml_file: Path to network XML file
        detector_file: Path to detectors XML file
        output_file: Output filename
        with_traffic_lights: Whether to parse and display traffic lights
    """
    # Parse data
    edges = parse_network_xml(xml_file)
    junctions = parse_junctions(xml_file)
    detectors = parse_detectors(detector_file)
    tl_junctions = parse_traffic_lights(xml_file) if with_traffic_lights else {}
    df = create_edge_dataframe(edges)

    # Calculate cumulative positions
    cumulative_pos = {}
    current_pos = 0
    for _, row in df.iterrows():
        edge_id = row['Edge ID']
        category = row['Category']
        if category in ['Mainline', 'Merge']:
            cumulative_pos[edge_id] = {
                'start': current_pos,
                'end': current_pos + HORIZONTAL_EDGE_WIDTH,
                'category': category
            }
            current_pos += HORIZONTAL_EDGE_WIDTH
        else:
            cumulative_pos[edge_id] = {'start': None, 'end': None, 'category': category}

    junction_pos = get_junction_positions(junctions, edges, cumulative_pos)

    plt.rcParams.update(PLOT_STYLE)
    fig, ax = plt.subplots(figsize=(18, 10))

    y_mainline = 5
    edge_heights = {}
    rm_junction_positions = {}
    rm_junction_widths = {}
    en_junction_widths = {}  # Track EN junction widths
    ex_junction_widths = {}  # Track EX junction widths

    # Calculate RM junction widths based on max of connected vertical edges
    for acc_id in ['E34_THA_ACC', 'E35_HOR_ACC', 'E36_WAED_ACC']:
        if acc_id in edges:
            from_junc = edges[acc_id]['from']
            acc_width = edges[acc_id]['num_lanes'] * LANE_WIDTH
            onramp_id = acc_id.replace('_ACC', '')
            onramp_width = edges[onramp_id]['num_lanes'] * LANE_WIDTH if onramp_id in edges else acc_width
            rm_junction_widths[from_junc] = max(acc_width, onramp_width)
            
            # Also store EN junction width (where acceleration lane connects to mainline)
            to_junc = edges[acc_id]['to']
            en_junction_widths[to_junc] = acc_width

    # Calculate EX junction widths based on off-ramp widths
    for _, row in df.iterrows():
        if row['Category'] == 'Off-ramp':
            edge_id = row['Edge ID']
            from_junc = edges[edge_id]['from']
            ex_junction_widths[from_junc] = edges[edge_id]['num_lanes'] * LANE_WIDTH

    # Draw mainline and merge segments (adjusted to not overlap with junctions on BOTH sides)
    for _, row in df.iterrows():
        edge_id = row['Edge ID']
        if row['Category'] in ['Mainline', 'Merge']:
            start = cumulative_pos[edge_id]['start']
            end = cumulative_pos[edge_id]['end']
            height = row['Lanes'] * LANE_HEIGHT
            edge_heights[edge_id] = height
            color = COLOR_SCHEME['mainline'] if row['Category'] == 'Mainline' else COLOR_SCHEME['merge']
            
            # Adjust start position if this edge starts from a junction with vertical connections
            from_junc = edges[edge_id]['from']
            if from_junc in rm_junction_widths:
                # Edge starts after RM junction
                start += rm_junction_widths[from_junc] / 2
            elif from_junc in en_junction_widths:
                # Edge starts after EN junction
                start += en_junction_widths[from_junc] / 2
            elif from_junc in ex_junction_widths:
                # Edge starts after EX junction
                start += ex_junction_widths[from_junc] / 2
            
            # Adjust end position if this edge ends at a junction with vertical connections
            to_junc = edges[edge_id]['to']
            if to_junc in ex_junction_widths:
                # Edge ends before EX junction
                end -= ex_junction_widths[to_junc] / 2
            elif to_junc in en_junction_widths:
                # Edge ends before EN junction
                end -= en_junction_widths[to_junc] / 2
            elif to_junc in rm_junction_widths:
                # Edge ends before RM junction
                end -= rm_junction_widths[to_junc] / 2
            
            # Draw the adjusted horizontal bar (aligned to bottom)
            adjusted_width = end - start
            ax.barh(y_mainline, adjusted_width, left=start, height=height,
                   color=color, edgecolor='black', linewidth=1.5, alpha=0.7, align='edge')
            
            mid_x = start + adjusted_width / 2
            mid_y = y_mainline + height / 2
            label = f"{edge_id}\n{row['Speed (km/h)']:.0f}km/h\n{row['Length (m)']:.0f}m | {row['Lanes']}L"
            ax.text(mid_x, mid_y, label, ha='center', va='center', fontsize=7, weight='bold')

    # Draw EN junctions first (at mainline level, where acceleration lanes connect)
    for junc_id, position in junction_pos.items():
        if junc_id in en_junction_widths and junc_id in junctions:
            junc_height = get_junction_height(junc_id, edges, edge_heights)
            junc_width = en_junction_widths[junc_id]
            
            junc_type = junctions[junc_id]['type']
            if junc_type == 'traffic_light':
                color, alpha = 'red', 0.9
            elif junc_type == 'priority':
                color, alpha = 'orange', 0.7
            elif junc_type == 'unregulated':
                color, alpha = 'gray', 0.6
            else:
                color, alpha = 'orange', 0.7

            rect = Rectangle((position - junc_width/2, y_mainline), junc_width, junc_height,
                           facecolor=color, edgecolor='black', linewidth=2, alpha=alpha, zorder=10)
            ax.add_patch(rect)
            ax.text(position, y_mainline + junc_height + 15, junc_id, fontsize=6, ha='center', va='bottom', weight='bold',
                   bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8))

    # Draw EX junctions (at mainline level, where off-ramps connect)
    for junc_id, position in junction_pos.items():
        if junc_id in ex_junction_widths and junc_id in junctions:
            junc_height = get_junction_height(junc_id, edges, edge_heights)
            junc_width = ex_junction_widths[junc_id]

            junc_type = junctions[junc_id]['type']
            if junc_type == 'traffic_light':
                color, alpha = 'red', 0.9
            elif junc_type == 'priority':
                color, alpha = 'orange', 0.7
            elif junc_type == 'unregulated':
                color, alpha = 'gray', 0.6
            else:
                color, alpha = 'orange', 0.7

            rect = Rectangle((position - junc_width/2, y_mainline), junc_width, junc_height,
                           facecolor=color, edgecolor='black', linewidth=2, alpha=alpha, zorder=10)
            ax.add_patch(rect)
            ax.text(position, y_mainline + junc_height + 15, junc_id, fontsize=6, ha='center', va='bottom', weight='bold',
                   bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8))

    # Draw acceleration lanes (connect to bottom of EN junction, avoid RM junction overlap)
    for acc_id in ['E34_THA_ACC', 'E35_HOR_ACC', 'E36_WAED_ACC']:
        if acc_id in edges and edges[acc_id]['to'] in junction_pos:
            en_x = junction_pos[edges[acc_id]['to']]
            
            # Start at bottom of EN junction
            acc_lane_top = y_mainline
            
            # Get width for spacing calculation
            acc_width = edges[acc_id]['num_lanes'] * LANE_WIDTH
            onramp_id = acc_id.replace('_ACC', '')
            onramp_width = edges[onramp_id]['num_lanes'] * LANE_WIDTH if onramp_id in edges else acc_width
            rm_junction_width = max(acc_width, onramp_width)
            
            # Calculate RM junction position with spacing
            rm_junction_height = LANE_HEIGHT / 2
            rm_y = acc_lane_top - VERTICAL_EDGE_HEIGHT - rm_junction_height
            rm_junction_positions[edges[acc_id]['from']] = {'x': en_x, 'y': rm_y}
            
            width = acc_width
            edge_heights[acc_id] = width
            
            # Draw acceleration lane from bottom of EN to top of RM junction
            acc_lane_bottom = rm_y + rm_junction_height / 2
            acc_lane_height = acc_lane_top - acc_lane_bottom
            
            rect = Rectangle((en_x - width/2, acc_lane_bottom), width, acc_lane_height,
                           facecolor=COLOR_SCHEME['acceleration'], edgecolor='black', linewidth=1.5, alpha=0.7)
            ax.add_patch(rect)
            
            label = f"{acc_id}\n{edges[acc_id]['speed_kmh']:.0f}km/h\n{edges[acc_id]['length_m']:.0f}m"
            ax.text(en_x, acc_lane_bottom + acc_lane_height/2, label, ha='center', va='center', fontsize=7, weight='bold',
                   bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))

    # Draw RM junctions (between acceleration lanes and on-ramps) - HALF HEIGHT
    rm_junction_height = LANE_HEIGHT / 2
    for rm_junc_id, pos_data in rm_junction_positions.items():
        width = rm_junction_widths.get(rm_junc_id, LANE_WIDTH)
        color = 'red' if rm_junc_id in tl_junctions else 'orange'
        alpha = 0.9 if rm_junc_id in tl_junctions else 0.7
        
        rect = Rectangle((pos_data['x'] - width/2, pos_data['y'] - rm_junction_height/2), width, rm_junction_height,
                       facecolor=color, edgecolor='black', linewidth=2, alpha=alpha, zorder=10)
        ax.add_patch(rect)
        ax.text(pos_data['x'], pos_data['y'] + rm_junction_height/2 + 15, rm_junc_id, fontsize=6, ha='center', va='bottom', weight='bold',
               bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8))

    # Draw on-ramps (connect to bottom of RM junction, leave space)
    for onramp_id in ['E34_THA', 'E35_HOR', 'E36_WAED']:
        if onramp_id in edges and edges[onramp_id]['to'] in rm_junction_positions:
            rm_pos = rm_junction_positions[edges[onramp_id]['to']]
            width = edges[onramp_id]['num_lanes'] * LANE_WIDTH
            edge_heights[onramp_id] = width
            
            # Start at bottom of RM junction with spacing
            onramp_top = rm_pos['y'] - rm_junction_height / 2
            bottom_y = onramp_top - VERTICAL_EDGE_HEIGHT
            
            rect = Rectangle((rm_pos['x'] - width/2, bottom_y), width, VERTICAL_EDGE_HEIGHT,
                           facecolor=COLOR_SCHEME['on_ramp'], edgecolor='black', linewidth=1.5, alpha=0.7)
            ax.add_patch(rect)
            
            label = f"{onramp_id}\n{edges[onramp_id]['speed_kmh']:.0f}km/h\n{edges[onramp_id]['length_m']:.0f}m"
            ax.text(rm_pos['x'], bottom_y + VERTICAL_EDGE_HEIGHT/2, label, ha='center', va='center', fontsize=7, weight='bold',
                   bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))

    # Draw off-ramps (connect to bottom of EX junction)
    for _, row in df.iterrows():
        if row['Category'] == 'Off-ramp' and edges[row['Edge ID']]['from'] in junction_pos:
            ex_x = junction_pos[edges[row['Edge ID']]['from']]
            
            width = edges[row['Edge ID']]['num_lanes'] * LANE_WIDTH
            edge_heights[row['Edge ID']] = width
            
            # Start at bottom of EX junction
            offramp_top = y_mainline
            bottom_y = offramp_top - VERTICAL_EDGE_HEIGHT
            
            rect = Rectangle((ex_x - width/2, bottom_y), width, VERTICAL_EDGE_HEIGHT,
                           facecolor=COLOR_SCHEME['off_ramp'], edgecolor='black', linewidth=1.5, alpha=0.7)
            ax.add_patch(rect)
            
            label = f"{row['Edge ID']}\n{row['Speed (km/h)']:.0f}km/h\n{row['Length (m)']:.0f}m"
            ax.text(ex_x, bottom_y + VERTICAL_EDGE_HEIGHT/2, label, ha='center', va='center', fontsize=7, weight='bold',
                   bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))

    # Draw traffic light icons (higher zorder)
    if with_traffic_lights:
        for tl_id in tl_junctions.keys():
            if tl_id in rm_junction_positions:
                pos = rm_junction_positions[tl_id]
                circle = plt.Circle((pos['x'], pos['y'] + rm_junction_height/2 + 30), 30, color='red',
                                  edgecolor='black', linewidth=2, alpha=0.9, zorder=15)
                ax.add_patch(circle)
                ax.text(pos['x'], pos['y'] + rm_junction_height/2 + 30, 'TL', fontsize=8,
                       ha='center', va='center', weight='bold', color='white', zorder=16)

    # Draw detectors (HIGHEST zorder to be on top of junctions)
    for loop in detectors['induction_loops']:
        parts = loop['lane'].rsplit('_', 1)
        edge_id = parts[0]
        lane_num = int(parts[1]) if len(parts) > 1 else 0

        if edge_id in cumulative_pos and cumulative_pos[edge_id]['start'] is not None:
            edge_length = edges[edge_id]['length_m']
            relative_pos = (edge_length + loop['pos']) / edge_length if loop['pos'] < 0 else loop['pos'] / edge_length
            detector_x = cumulative_pos[edge_id]['start'] + (relative_pos * HORIZONTAL_EDGE_WIDTH)
            
            num_lanes = edges[edge_id]['num_lanes']
            lane_offset = (lane_num + 0.5) * LANE_HEIGHT
            detector_y = y_mainline + lane_offset

            ax.plot(detector_x, detector_y, marker='D', markersize=8, color='cyan',
                   markeredgecolor='black', markeredgewidth=1.5, zorder=20)
            ax.text(detector_x, detector_y + LANE_HEIGHT * 0.6, loop['id'], fontsize=5, ha='center', va='bottom', rotation=90,
                   bbox=dict(boxstyle='round,pad=0.2', facecolor='cyan', alpha=0.6), zorder=21)

    # Zone detectors (HIGHEST zorder)
    for area in detectors['lane_area_detectors']:
        edge_id = area['lane'].rsplit('_', 1)[0]
        if edge_id in ['E34_THA', 'E35_HOR', 'E36_WAED'] and edges[edge_id]['to'] in rm_junction_positions:
            pos = rm_junction_positions[edges[edge_id]['to']]
            y_detector = pos['y'] - rm_junction_height / 2  # At bottom of RM junction
            ax.plot(pos['x'], y_detector, marker='^', markersize=10, color='lime',
                   markeredgecolor='black', markeredgewidth=1.5, zorder=20)
            ax.text(pos['x'] + LANE_WIDTH, y_detector, area['id'], fontsize=5, ha='left', va='center',
                   bbox=dict(boxstyle='round,pad=0.2', facecolor='lime', alpha=0.6), zorder=21)

    # Legend
    legend_elements = [
        mpatches.Patch(color=COLOR_SCHEME['mainline'], label='Mainline A3', alpha=0.7),
        mpatches.Patch(color=COLOR_SCHEME['merge'], label='Merge Sections', alpha=0.7),
        mpatches.Patch(color=COLOR_SCHEME['on_ramp'], label='On-ramps', alpha=0.7),
        mpatches.Patch(color=COLOR_SCHEME['acceleration'], label='Acceleration Lanes', alpha=0.7),
        mpatches.Patch(color=COLOR_SCHEME['off_ramp'], label='Off-ramps', alpha=0.7),
        mpatches.Patch(color='orange', label='Priority Junction', alpha=0.7),
        mpatches.Patch(color='gray', label='Unregulated Junction', alpha=0.6),
        plt.Line2D([0], [0], marker='D', color='w', markerfacecolor='cyan', markeredgecolor='black', markersize=8, label='Induction Loop'),
        plt.Line2D([0], [0], marker='^', color='w', markerfacecolor='lime', markeredgecolor='black', markersize=10, label='Zone Detector'),
    ]
    if with_traffic_lights:
        legend_elements.insert(5, mpatches.Patch(color='red', label='Traffic Light Junction', alpha=0.9))
        legend_elements.append(plt.Line2D([0], [0], marker='o', color='w', markerfacecolor='red', markersize=10, label='Traffic Light', markeredgecolor='black'))
    
    ax.legend(handles=legend_elements, loc='upper left', framealpha=0.9, fontsize=10)

    # Formatting
    max_pos = max([p['end'] for p in cumulative_pos.values() if p['end'] is not None])
    min_y = min(pos['y'] for pos in rm_junction_positions.values()) - LANE_HEIGHT / 2 - VERTICAL_EDGE_HEIGHT - 50 if rm_junction_positions else 0

    ax.set_xlim(-500, max_pos + 500)
    ax.set_ylim(min_y, y_mainline + 200)
    ax.set_xlabel('Network Topology', fontsize=12, weight='bold')
    ax.set_yticks([])
    title = 'Network Infrastructure with Traffic Lights' if with_traffic_lights else 'Network Infrastructure - Complete Visualization'
    ax.set_title(f'{title}\nSUMO Network: A3 Highway (WAE → HOR → THA)', fontsize=14, weight='bold', pad=20)
    ax.grid(axis='x', alpha=0.3)
    ax.set_axisbelow(True)

    plt.tight_layout()
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    plt.savefig(output_file, dpi=300, bbox_inches='tight')
    print(f"[OK] Network visualization saved: {output_file}")
    
    pdf_file = output_file.replace('.png', '.pdf')
    plt.savefig(pdf_file, format='pdf', bbox_inches='tight')
    print(f"[OK] Network visualization PDF saved: {pdf_file}")
    plt.close()


def main():
    """Main execution function."""
    print("\n" + "="*100)
    print("NETWORK INFRASTRUCTURE ANALYSIS")
    print("="*100)

    try:
        print("\n[1/4] Parsing network files...")
        edges = parse_network_xml('shared_simulation_files/Network.net.xml')
        junctions = parse_junctions('shared_simulation_files/Network.net.xml')
        detectors = parse_detectors('shared_simulation_files/detectors.add.xml')
        print(f"[OK] Parsed {len(edges)} edges, {len(junctions)} junctions")

        print("\n[2/4] Creating dataframes...")
        df_edges = create_edge_dataframe(edges, detectors)
        df_junctions = create_junction_dataframe(junctions, edges)
        print(f"[OK] Created edge list ({len(df_edges)} entries) and junction list ({len(df_junctions)} entries)")

        print("\n[3/4] Saving to Excel...")
        os.makedirs('infrastructure_data', exist_ok=True)
        excel_file = 'infrastructure_data/network_infrastructure_list.xlsx'
        with pd.ExcelWriter(excel_file, engine='openpyxl') as writer:
            df_edges.to_excel(writer, index=False, sheet_name='Edges')
            df_junctions.to_excel(writer, index=False, sheet_name='Junctions')
        print(f"[OK] Saved to {excel_file}")

        print("\n[4/4] Creating visualizations...")
        plot_network_infrastructure('shared_simulation_files/Network.net.xml', 
                                   'shared_simulation_files/detectors.add.xml',
                                   'infrastructure_data/network_with_junctions.png',
                                   with_traffic_lights=False)
        
        plot_network_infrastructure('shared_simulation_files/Network_TL.net.xml', 
                                   'shared_simulation_files/detectors.add.xml',
                                   'infrastructure_data/network_with_junctions_and_tl.png',
                                   with_traffic_lights=True)

        print("\n" + "="*100)
        print("ANALYSIS COMPLETE")
        print("="*100)
        print("\nGenerated files:")
        print("  - infrastructure_data/network_infrastructure_list.xlsx")
        print("  - infrastructure_data/network_with_junctions.png & .pdf")
        print("  - infrastructure_data/network_with_junctions_and_tl.png & .pdf")

    except Exception as e:
        print(f"[ERROR] {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()
