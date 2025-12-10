#!/usr/bin/env python3
"""
Generate USD world file for tennis court environment in Isaac Sim.
This creates a USD file equivalent to the Gazebo court.sdf world.
"""

import os
import sys
from pathlib import Path


def generate_court_usd(output_path: str):
    """
    Generate a USD world file for the tennis court environment.
    This matches the structure from court.sdf.
    """
    usd_content = '''#usda 1.0
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
{
    def Xform "Environment" (
        kind = "group"
    )
    {
        # Ground plane (100x100 meters)
        def Mesh "GroundPlane"
        {
            float3[] extent = [(-50, -50, 0), (50, 50, 0)]
            int[] faceVertexCounts = [4]
            int[] faceVertexIndices = [0, 1, 2, 3]
            point3f[] points = [(-50, -50, 0), (50, -50, 0), (50, 50, 0), (-50, 50, 0)]
            color3f[] primvars:displayColor = [(0.8, 0.8, 0.8)]
            double3 xformOp:translate = (0, 0, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        # Outer apron (green area around court) - 36.57m x 18.29m
        def Mesh "OuterApron"
        {
            float3[] extent = [(-18.285, -9.145, -0.05), (18.285, 9.145, 0.05)]
            int[] faceVertexCounts = [4]
            int[] faceVertexIndices = [0, 1, 2, 3]
            point3f[] points = [(-18.285, -9.145, -0.05), (18.285, -9.145, -0.05), (18.285, 9.145, 0.05), (-18.285, 9.145, 0.05)]
            color3f[] primvars:displayColor = [(0.02, 0.25, 0.02)]
            double3 xformOp:translate = (0, 0, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        # Main court surface - 23.77m x 10.97m
        def Mesh "CourtGround"
        {
            float3[] extent = [(-11.885, -5.485, -0.05), (11.885, 5.485, 0.05)]
            int[] faceVertexCounts = [4]
            int[] faceVertexIndices = [0, 1, 2, 3]
            point3f[] points = [(-11.885, -5.485, -0.05), (11.885, -5.485, -0.05), (11.885, 5.485, 0.05), (-11.885, 5.485, 0.05)]
            color3f[] primvars:displayColor = [(0.0, 0.60, 0.0)]
            double3 xformOp:translate = (0, 0, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        # Court lines - Baselines
        def Mesh "BaselinePositiveX"
        {
            float3[] extent = [(-0.025, -5.485, 0.005), (0.025, 5.485, 0.015)]
            int[] faceVertexCounts = [4]
            int[] faceVertexIndices = [0, 1, 2, 3]
            point3f[] points = [(-0.025, -5.485, 0.005), (0.025, -5.485, 0.005), (0.025, 5.485, 0.015), (-0.025, 5.485, 0.015)]
            color3f[] primvars:displayColor = [(1, 1, 1)]
            double3 xformOp:translate = (11.885, 0, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def Mesh "BaselineNegativeX"
        {
            float3[] extent = [(-0.025, -5.485, 0.005), (0.025, 5.485, 0.015)]
            int[] faceVertexCounts = [4]
            int[] faceVertexIndices = [0, 1, 2, 3]
            point3f[] points = [(-0.025, -5.485, 0.005), (0.025, -5.485, 0.005), (0.025, 5.485, 0.015), (-0.025, 5.485, 0.015)]
            color3f[] primvars:displayColor = [(1, 1, 1)]
            double3 xformOp:translate = (-11.885, 0, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        # Doubles sidelines
        def Mesh "DoublesSidelinePosY"
        {
            float3[] extent = [(-11.885, -0.025, 0.005), (11.885, 0.025, 0.015)]
            int[] faceVertexCounts = [4]
            int[] faceVertexIndices = [0, 1, 2, 3]
            point3f[] points = [(-11.885, -0.025, 0.005), (11.885, -0.025, 0.005), (11.885, 0.025, 0.015), (-11.885, 0.025, 0.015)]
            color3f[] primvars:displayColor = [(1, 1, 1)]
            double3 xformOp:translate = (0, 5.485, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def Mesh "DoublesSidelineNegY"
        {
            float3[] extent = [(-11.885, -0.025, 0.005), (11.885, 0.025, 0.015)]
            int[] faceVertexCounts = [4]
            int[] faceVertexIndices = [0, 1, 2, 3]
            point3f[] points = [(-11.885, -0.025, 0.005), (11.885, -0.025, 0.005), (11.885, 0.025, 0.015), (-11.885, 0.025, 0.015)]
            color3f[] primvars:displayColor = [(1, 1, 1)]
            double3 xformOp:translate = (0, -5.485, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        # Singles sidelines
        def Mesh "SinglesSidelinePosY"
        {
            float3[] extent = [(-11.885, -0.025, 0.005), (11.885, 0.025, 0.015)]
            int[] faceVertexCounts = [4]
            int[] faceVertexIndices = [0, 1, 2, 3]
            point3f[] points = [(-11.885, -0.025, 0.005), (11.885, -0.025, 0.005), (11.885, 0.025, 0.015), (-11.885, 0.025, 0.015)]
            color3f[] primvars:displayColor = [(1, 1, 1)]
            double3 xformOp:translate = (0, 4.115, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def Mesh "SinglesSidelineNegY"
        {
            float3[] extent = [(-11.885, -0.025, 0.005), (11.885, 0.025, 0.015)]
            int[] faceVertexCounts = [4]
            int[] faceVertexIndices = [0, 1, 2, 3]
            point3f[] points = [(-11.885, -0.025, 0.005), (11.885, -0.025, 0.005), (11.885, 0.025, 0.015), (-11.885, 0.025, 0.015)]
            color3f[] primvars:displayColor = [(1, 1, 1)]
            double3 xformOp:translate = (0, -4.115, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        # Service lines
        def Mesh "ServiceLinePosX"
        {
            float3[] extent = [(-0.025, -4.115, 0.005), (0.025, 4.115, 0.015)]
            int[] faceVertexCounts = [4]
            int[] faceVertexIndices = [0, 1, 2, 3]
            point3f[] points = [(-0.025, -4.115, 0.005), (0.025, -4.115, 0.005), (0.025, 4.115, 0.015), (-0.025, 4.115, 0.015)]
            color3f[] primvars:displayColor = [(1, 1, 1)]
            double3 xformOp:translate = (6.40, 0, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def Mesh "ServiceLineNegX"
        {
            float3[] extent = [(-0.025, -4.115, 0.005), (0.025, 4.115, 0.015)]
            int[] faceVertexCounts = [4]
            int[] faceVertexIndices = [0, 1, 2, 3]
            point3f[] points = [(-0.025, -4.115, 0.005), (0.025, -4.115, 0.005), (0.025, 4.115, 0.015), (-0.025, 4.115, 0.015)]
            color3f[] primvars:displayColor = [(1, 1, 1)]
            double3 xformOp:translate = (-6.40, 0, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        # Center service line
        def Mesh "CenterServiceLine"
        {
            float3[] extent = [(-6.40, -0.025, 0.005), (6.40, 0.025, 0.015)]
            int[] faceVertexCounts = [4]
            int[] faceVertexIndices = [0, 1, 2, 3]
            point3f[] points = [(-6.40, -0.025, 0.005), (6.40, -0.025, 0.005), (6.40, 0.025, 0.015), (-6.40, 0.025, 0.015)]
            color3f[] primvars:displayColor = [(1, 1, 1)]
            double3 xformOp:translate = (0, 0, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        # Net - 0.05m x 10.97m x 0.91m
        def Mesh "Net"
        {
            float3[] extent = [(-0.025, -5.485, 0), (0.025, 5.485, 0.91)]
            int[] faceVertexCounts = [4, 4, 4, 4, 4, 4]
            int[] faceVertexIndices = [0, 1, 2, 3, 4, 7, 6, 5, 0, 4, 5, 1, 2, 6, 7, 3, 1, 5, 6, 2, 0, 3, 7, 4]
            point3f[] points = [
                (-0.025, -5.485, 0), (0.025, -5.485, 0),
                (0.025, 5.485, 0.91), (-0.025, 5.485, 0.91),
                (-0.025, -5.485, 0.91), (0.025, -5.485, 0.91),
                (0.025, 5.485, 0), (-0.025, 5.485, 0)
            ]
            color3f[] primvars:displayColor = [(0.8, 0.8, 0.8)]
            double3 xformOp:translate = (0, 0, 0.455)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        # Benches
        def Mesh "BenchPosY"
        {
            float3[] extent = [(-1.0, -0.25, 0), (1.0, 0.25, 0.5)]
            int[] faceVertexCounts = [4, 4, 4, 4, 4, 4]
            int[] faceVertexIndices = [0, 1, 2, 3, 4, 7, 6, 5, 0, 4, 5, 1, 2, 6, 7, 3, 1, 5, 6, 2, 0, 3, 7, 4]
            point3f[] points = [
                (-1.0, -0.25, 0), (1.0, -0.25, 0),
                (1.0, 0.25, 0.5), (-1.0, 0.25, 0.5),
                (-1.0, -0.25, 0.5), (1.0, -0.25, 0.5),
                (1.0, 0.25, 0), (-1.0, 0.25, 0)
            ]
            color3f[] primvars:displayColor = [(0.5, 0.3, 0.1)]
            double3 xformOp:translate = (0, 6.985, 0.25)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def Mesh "BenchNegY"
        {
            float3[] extent = [(-1.0, -0.25, 0), (1.0, 0.25, 0.5)]
            int[] faceVertexCounts = [4, 4, 4, 4, 4, 4]
            int[] faceVertexIndices = [0, 1, 2, 3, 4, 7, 6, 5, 0, 4, 5, 1, 2, 6, 7, 3, 1, 5, 6, 2, 0, 3, 7, 4]
            point3f[] points = [
                (-1.0, -0.25, 0), (1.0, -0.25, 0),
                (1.0, 0.25, 0.5), (-1.0, 0.25, 0.5),
                (-1.0, -0.25, 0.5), (1.0, -0.25, 0.5),
                (1.0, 0.25, 0), (-1.0, 0.25, 0)
            ]
            color3f[] primvars:displayColor = [(0.5, 0.3, 0.1)]
            double3 xformOp:translate = (0, -6.985, 0.25)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        # Umpire chair
        def Mesh "UmpireChair"
        {
            float3[] extent = [(-0.3, -0.3, 0), (0.3, 0.3, 2.5)]
            int[] faceVertexCounts = [4, 4, 4, 4, 4, 4]
            int[] faceVertexIndices = [0, 1, 2, 3, 4, 7, 6, 5, 0, 4, 5, 1, 2, 6, 7, 3, 1, 5, 6, 2, 0, 3, 7, 4]
            point3f[] points = [
                (-0.3, -0.3, 0), (0.3, -0.3, 0),
                (0.3, 0.3, 2.5), (-0.3, 0.3, 2.5),
                (-0.3, -0.3, 2.5), (0.3, -0.3, 2.5),
                (0.3, 0.3, 0), (-0.3, 0.3, 0)
            ]
            color3f[] primvars:displayColor = [(0.2, 0.2, 0.2)]
            double3 xformOp:translate = (0, 6.2, 1.25)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        # Audience stands
        def Mesh "AudienceStandRow1"
        {
            float3[] extent = [(-13.0, -0.75, 0), (13.0, 0.75, 0.5)]
            int[] faceVertexCounts = [4, 4, 4, 4, 4, 4]
            int[] faceVertexIndices = [0, 1, 2, 3, 4, 7, 6, 5, 0, 4, 5, 1, 2, 6, 7, 3, 1, 5, 6, 2, 0, 3, 7, 4]
            point3f[] points = [
                (-13.0, -0.75, 0), (13.0, -0.75, 0),
                (13.0, 0.75, 0.5), (-13.0, 0.75, 0.5),
                (-13.0, -0.75, 0.5), (13.0, -0.75, 0.5),
                (13.0, 0.75, 0), (-13.0, 0.75, 0)
            ]
            color3f[] primvars:displayColor = [(0.4, 0.4, 0.4)]
            double3 xformOp:translate = (0, 8.2, 0.25)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def Mesh "AudienceStandRow2"
        {
            float3[] extent = [(-13.0, -0.75, 0), (13.0, 0.75, 0.5)]
            int[] faceVertexCounts = [4, 4, 4, 4, 4, 4]
            int[] faceVertexIndices = [0, 1, 2, 3, 4, 7, 6, 5, 0, 4, 5, 1, 2, 6, 7, 3, 1, 5, 6, 2, 0, 3, 7, 4]
            point3f[] points = [
                (-13.0, -0.75, 0), (13.0, -0.75, 0),
                (13.0, 0.75, 0.5), (-13.0, 0.75, 0.5),
                (-13.0, -0.75, 0.5), (13.0, -0.75, 0.5),
                (13.0, 0.75, 0), (-13.0, 0.75, 0)
            ]
            color3f[] primvars:displayColor = [(0.4, 0.4, 0.4)]
            double3 xformOp:translate = (0, 9.9, 0.75)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def Mesh "AudienceStandRow3"
        {
            float3[] extent = [(-13.0, -0.75, 0), (13.0, 0.75, 0.5)]
            int[] faceVertexCounts = [4, 4, 4, 4, 4, 4]
            int[] faceVertexIndices = [0, 1, 2, 3, 4, 7, 6, 5, 0, 4, 5, 1, 2, 6, 7, 3, 1, 5, 6, 2, 0, 3, 7, 4]
            point3f[] points = [
                (-13.0, -0.75, 0), (13.0, -0.75, 0),
                (13.0, 0.75, 0.5), (-13.0, 0.75, 0.5),
                (-13.0, -0.75, 0.5), (13.0, -0.75, 0.5),
                (13.0, 0.75, 0), (-13.0, 0.75, 0)
            ]
            color3f[] primvars:displayColor = [(0.4, 0.4, 0.4)]
            double3 xformOp:translate = (0, 11.6, 1.25)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        # Lighting
        def DistantLight "SunLight"
        {
            float intensity = 1.0
            float3 xformOp:rotateXYZ = (11.85, 0, 0)
            uniform token[] xformOpOrder = ["xformOp:rotateXYZ"]
        }
    }

    # Physics settings
    def PhysicsScene "PhysicsScene"
    {
        float3 gravity = (0, 0, -9.8)
        bool enabled = 1
    }
}
'''

    # Write the USD file
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    with open(output_path, 'w') as f:
        f.write(usd_content)
    
    print(f"Generated USD world file: {output_path}")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        output_path = sys.argv[1]
    else:
        script_dir = Path(__file__).parent
        output_path = script_dir.parent / "worlds" / "court.usd"
        output_path.parent.mkdir(parents=True, exist_ok=True)
    
    generate_court_usd(str(output_path))
