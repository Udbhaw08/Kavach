# Ultra-Realistic Tank Model for YOLO Detection

## Overview
This model has been designed specifically for YOLO object detection with realistic military tank features.

## Specifications
- **Overall Dimensions**: ~10m length × 12m width × 6.5m height
- **Total Components**: 25+ detailed parts
- **Color Scheme**: Military olive green with variations

## Key Features

### Main Structure
1. **Lower Hull** (9m × 11m × 2m) - Main chassis
2. **Upper Hull** (7m × 10m × 1m) - Superstructure
3. **Main Turret** (5.5m × 7.5m × 2.6m) - Rotating turret base
4. **Turret Mantlet** (1.8m × 4m × 2m) - Front armor

### Armament
5. **Main Gun Barrel** (7.5m long, 0.3m diameter)
6. **Muzzle Brake** (0.8m long, 0.4m diameter)

### Crew Positions
7. **Commander's Cupola** (1m radius cylinder)
8. **Driver's Hatch** (0.5m radius)

### Running Gear
9-16. **8 Road Wheels** (0.7m radius, 4 per side)
17-18. **Track Assemblies** (9.5m × 1.2m, left and right)

### Armor & Protection
19. **Front Glacis** (angled at 20°)
20-21. **Side Skirt Armor** (left and right)

### Engine & Exhaust
22. **Engine Deck** (1.5m × 10m)
23-24. **Exhaust Grilles** (left and right)

### Storage
25-26. **Turret Storage Boxes** (left and right)

## Color Palette (for YOLO training)
- **Main Hull**: RGB(0.25, 0.32, 0.20) - Olive green
- **Turret**: RGB(0.23, 0.30, 0.18) - Darker green
- **Gun Barrel**: RGB(0.16, 0.16, 0.16) - Dark gray/black
- **Tracks**: RGB(0.12, 0.12, 0.12) - Black
- **Wheels**: RGB(0.10, 0.10, 0.10) - Dark black

## YOLO Detection Features
✓ High contrast between components
✓ Distinctive silhouette (turret + gun barrel)
✓ Realistic proportions matching real tanks
✓ Multiple detection points (turret, barrel, tracks, wheels)
✓ Proper specular highlights for depth perception

## Usage
Load this model in your Gazebo world to test YOLO detection from the VTOL camera.
