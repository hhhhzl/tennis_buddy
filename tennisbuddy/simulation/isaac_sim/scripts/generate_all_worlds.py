#!/usr/bin/env python3
"""
Generate all USD world files for Isaac Sim from Gazebo SDF files.
This script generates basic USD templates for all worlds.
"""

import os
import sys
from pathlib import Path
import subprocess


def generate_basic_usd_world(output_path: str, world_name: str):
    """
    Generate a basic USD world file template.
    
    Args:
        output_path: Path to output USD file
        world_name: Name of the world
    """
    usd_content = f'''#usda 1.0
(
    defaultPrim = "World"
    endTimeCode = 100
    metersPerUnit = 1
    startTimeCode = 0
    timeCodesPerSecond = 60
    upAxis = "Z"
)

def Xform "World" (
    kind = "group"
)
{{
    def Xform "Environment" (
        kind = "group"
    )
    {{
        # Ground plane
        def Mesh "GroundPlane"
        {{
            float3[] extent = [(-50, -50, 0), (50, 50, 0)]
            int[] faceVertexCounts = [4]
            int[] faceVertexIndices = [0, 1, 2, 3]
            point3f[] points = [(-50, -50, 0), (50, -50, 0), (50, 50, 0), (-50, 50, 0)]
            color3f[] primvars:displayColor = [(0.8, 0.8, 0.8)]
            double3 xformOp:translate = (0, 0, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }}

        # Lighting
        def DistantLight "SunLight"
        {{
            float intensity = 1.0
            float3 xformOp:rotateXYZ = (45, 0, 0)
            uniform token[] xformOpOrder = ["xformOp:rotateXYZ"]
        }}
    }}

    # Physics settings
    def PhysicsScene "PhysicsScene"
    {{
        float3 gravity = (0, 0, -9.8)
        bool enabled = 1
    }}
}}
'''

    # Write the USD file
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    with open(output_path, 'w') as f:
        f.write(usd_content)
    
    print(f"Generated USD world file: {output_path}")


def main():
    """Generate all world files."""
    script_dir = Path(__file__).parent
    worlds_dir = script_dir.parent / "worlds"
    worlds_dir.mkdir(parents=True, exist_ok=True)
    
    # World names
    worlds = {
        'court': 'Tennis Court',
        'maze': 'Maze',
        'warehouse': 'Warehouse',
        'depot': 'Depot'
    }
    
    # Check if court.py exists (more detailed generator)
    court_script = script_dir / "generate_tennis_court_usd.py"
    if court_script.exists():
        print("Generating detailed court.usd...")
        try:
            subprocess.run([
                sys.executable,
                str(court_script)
            ], check=True)
        except subprocess.CalledProcessError:
            print("Warning: Failed to generate detailed court.usd, using template...")
            generate_basic_usd_world(
                str(worlds_dir / "court.usd"),
                "court"
            )
    else:
        generate_basic_usd_world(
            str(worlds_dir / "court.usd"),
            "court"
        )
    
    # Generate other worlds as basic templates
    for world_name, description in worlds.items():
        if world_name == 'court':
            continue  # Already generated
        
        output_path = worlds_dir / f"{world_name}.usd"
        if not output_path.exists():
            print(f"Generating {world_name}.usd ({description})...")
            generate_basic_usd_world(str(output_path), world_name)
        else:
            print(f"{world_name}.usd already exists, skipping...")
    
    print("\nAll world files generated!")
    print(f"World files location: {worlds_dir}")


if __name__ == "__main__":
    main()
