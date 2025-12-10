#!/usr/bin/env python3
"""
Utility script to map between Gazebo SDF world files and Isaac Sim USD world files.
Helps maintain compatibility between the two simulators.
"""

import os
import sys
from pathlib import Path


# World file mapping: SDF name -> USD name
WORLD_MAP = {
    'court.sdf': 'court.usd',
    'maze.sdf': 'maze.usd',
    'warehouse.sdf': 'warehouse.usd',
    'depot.sdf': 'depot.usd',
}


def get_isaac_world(gazebo_world: str) -> str:
    """
    Get corresponding Isaac Sim world file name for a Gazebo world file.
    
    Args:
        gazebo_world: Gazebo world file name (e.g., 'court.sdf')
    
    Returns:
        Corresponding USD world file name (e.g., 'court.usd')
    """
    if gazebo_world in WORLD_MAP:
        return WORLD_MAP[gazebo_world]
    
    # Default: replace .sdf with .usd
    if gazebo_world.endswith('.sdf'):
        return gazebo_world.replace('.sdf', '.usd')
    
    return gazebo_world


def get_gazebo_world(isaac_world: str) -> str:
    """
    Get corresponding Gazebo world file name for an Isaac Sim world file.
    
    Args:
        isaac_world: Isaac Sim world file name (e.g., 'court.usd')
    
    Returns:
        Corresponding SDF world file name (e.g., 'court.sdf')
    """
    # Reverse lookup
    for sdf_name, usd_name in WORLD_MAP.items():
        if isaac_world == usd_name:
            return sdf_name
    
    # Default: replace .usd with .sdf
    if isaac_world.endswith('.usd'):
        return isaac_world.replace('.usd', '.sdf')
    
    return isaac_world


if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("Usage: python3 world_file_mapper.py <direction> <world_file>")
        print("  direction: 'to_isaac' or 'to_gazebo'")
        print("  world_file: world file name (e.g., 'court.sdf' or 'court.usd')")
        sys.exit(1)
    
    direction = sys.argv[1]
    world_file = sys.argv[2]
    
    if direction == 'to_isaac':
        result = get_isaac_world(world_file)
        print(result)
    elif direction == 'to_gazebo':
        result = get_gazebo_world(world_file)
        print(result)
    else:
        print(f"Unknown direction: {direction}")
        sys.exit(1)
