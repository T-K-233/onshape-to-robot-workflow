# Onshape to Robot Workflow

This repository contains an example workflow to convert an Onshape CAD model to a URDF and MJCF file.

## Quick Start

```bash
sudo apt install openscad
```

```bash
uv sync
```

```bash
uv run ./scripts/export_onshape_to_urdf.py
```

```bash
cd ./data/miku/urdf/assets/
uv run onshape-to-robot-edit-shape ./chest.stl
```

```bash
uv run ./scripts/convert_urdf_to_mjcf.py ./data/miku/urdf/miku.urdf ./data/miku/mjcf/miku.xml 
```