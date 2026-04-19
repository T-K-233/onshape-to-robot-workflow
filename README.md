# Robot Descriptions

URDF and MuJoCo (MJCF) descriptions for humanoid robots and other assets, plus CLI tools to regenerate them from Onshape using [onshape-to-robot](https://github.com/Rhoban/onshape-to-robot) and to convert URDF to MJCF.

CLI implementations live under `robot_descriptions/workflow/`.

## Contents

Collision geometry in the shipped URDF uses primitives and/or merged meshes from the export pipeline; **visual meshes** are loaded from `robots/<robot>/meshes/` (paths relative to the URDF).

## Example Usage

Add to your `uv`-managed project:

```bash
uv add git+https://github.com/Berkeley-Humanoids/Robot-Descriptions.git
```

Loading a robot asset:

```python
from robot_descriptions import load_asset

# load_asset() method will automatically fetch
# the robot description files from Github repository
urdf_path = load_asset("robots/miku/urdf/miku.urdf")
```

You can also specify the remote repository and the local directory:

```python
urdf_path = load_asset(
    repo_url="https://github.com/<Your-Name>/<Your-Repository>",
    cache_dir="<directory-path>/",
)
```

## Re-generating Robot Assets

1. Set up dependency.

    ```bash
    uv sync
    ```

    We need to install OpenSCAD to perform collider editing.

    ```bash
    sudo apt install openscad
    ```

2. Export files from Onshape (New).

    Onshape is posing a usage limit on API access. To work around that, we can use the [onshape-to-robot application](https://cad.onshape.com/appstore/apps/File%20Export/698b9abb84adb494ca5d5d5a).

    Go to the assembly tab, and copy paste the `config.json` in the application panel.

    After conversion, download the result and extract to `./robots/<robot>/urdf/`.

3. Perform post processing.

    ```bash
    uv run robot-descriptions-onshape-to-urdf ./robots/<robot>/urdf/config.json --convert
    ```

    For the previous API access version, run without the `--convert` flag to directly fetch from Onshape:

    ```bash
    uv run robot-descriptions-onshape-to-urdf ./robots/<robot>/urdf/config.json
    ```

    Note that API access could be expensive depending on the complexity of the robot. Typically it consumes ~1000 requests per conversion.

4. Edit collision shapes (OpenSCAD)

    First, use `--keep-assets` when running the onshape export script. This argument will preserve the intermediary `urdf/assets/` folder and the `robot.pkl` file.

    ```bash
    uv run robot-descriptions-onshape-to-urdf ./robots/<robot>/urdf/config.json --keep-assets
    ```

    To edit the collider for specific STL body, run the following command:

    ```bash
    cd ./robots/<robot>/urdf/assets/
    uv run onshape-to-robot-edit-shape ./chest.stl
    ```

5. URDF -> MJCF

    Joint actuator and MJCF joint tuning are loaded from `robots/<robot>/urdf/joint_properties.json`.

    ```bash
    uv run robot-descriptions-urdf-to-mjcf ./robots/<robot>/urdf/<robot_name>.urdf ./robots/<robot>/mjcf/<robot_name>.xml
    ```

    For floating base robots:

    ```bash
    uv run robot-descriptions-urdf-to-mjcf ./robots/<robot>/urdf/<robot_name>.urdf ./robots/<robot>/mjcf/<robot_name>.xml --freejoint
    ```
