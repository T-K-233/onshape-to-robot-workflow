# Copyright 2023 The RoboPianist Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Programatically build a piano MJCF model."""

from pathlib import Path

from dm_control import mjcf
from mujoco_utils import types

try:
    from . import consts
except ImportError:
    import consts  # type: ignore[no-redef]


def build() -> types.MjcfRootElement:
    """Programatically build a piano MJCF."""
    root = mjcf.RootElement()
    root.model = "piano"

    root.compiler.autolimits = True
    root.compiler.angle = "radian"

    # Add materials.
    root.asset.add("material", name="white", rgba=consts.WHITE_KEY_COLOR)
    root.asset.add("material", name="black", rgba=consts.BLACK_KEY_COLOR)

    root.default.geom.type = "box"
    root.default.joint.type = "hinge"
    root.default.joint.axis = [0, 1, 0]
    root.default.site.type = "box"
    root.default.site.group = 4
    root.default.site.rgba = [1, 0, 0, 1]

    # This effectively disables key-key collisions but still allows hand-key collisions,
    # assuming we've kept the default hand contype = conaffinity = 1.
    # See https://mujoco.readthedocs.io/en/latest/computation.html#selection for more
    # details.
    root.default.geom.contype = 0
    root.default.geom.conaffinity = 1

    # White key defaults.
    white_default = root.default.add("default", dclass="white_key")
    white_default.geom.material = "white"
    white_default.geom.size = [
        consts.WHITE_KEY_LENGTH / 2,
        consts.WHITE_KEY_WIDTH / 2,
        consts.WHITE_KEY_HEIGHT / 2,
    ]
    white_default.geom.mass = consts.WHITE_KEY_MASS
    white_default.site.size = white_default.geom.size
    white_default.joint.pos = [consts.WHITE_KEY_LENGTH / 2 - consts.WHITE_KEY_TOTAL_LENGTH, 0, 0]
    white_default.joint.damping = consts.KEY_DAMPING
    white_default.joint.armature = consts.KEY_ARMATURE
    white_default.joint.stiffness = consts.KEY_STIFFNESS
    white_default.joint.springref = consts.KEY_SPRINGREF
    white_default.joint.range = [0, consts.WHITE_KEY_JOINT_MAX_ANGLE]

    # Black key defaults.
    black_default = root.default.add("default", dclass="black_key")
    black_default.geom.material = "black"
    black_default.geom.size = [
        consts.BLACK_KEY_LENGTH / 2,
        consts.BLACK_KEY_WIDTH / 2,
        consts.BLACK_KEY_HEIGHT / 2,
    ]
    black_default.site.size = black_default.geom.size
    black_default.geom.mass = consts.BLACK_KEY_MASS
    black_default.joint.pos = [consts.BLACK_KEY_LENGTH / 2 - consts.BLACK_KEY_TOTAL_LENGTH, 0, 0]
    black_default.joint.damping = consts.KEY_DAMPING
    black_default.joint.armature = consts.KEY_ARMATURE
    black_default.joint.stiffness = consts.KEY_STIFFNESS
    black_default.joint.springref = consts.KEY_SPRINGREF
    black_default.joint.range = [0, consts.BLACK_KEY_JOINT_MAX_ANGLE]

    # Add parent piano body.
    piano_body = root.worldbody.add("body", name="piano")

    # Add base inside piano body.
    base_body = piano_body.add("body", name="base")
    base_body.add(
        "inertial",
        pos=consts.BASE_1_POS,
        mass=10.0,
        diaginertia=[0.1, 0.1, 0.1],
    )
    base_body.add("geom", name="base_1", type="box", pos=consts.BASE_1_POS, size=consts.BASE_1_SIZE, rgba=consts.BASE_COLOR)
    base_body.add("geom", name="base_2", type="box", pos=consts.BASE_2_POS, size=consts.BASE_2_SIZE, rgba=consts.BASE_COLOR)

    # These will hold kwargs. We'll subsequently use them to create the actual objects.
    geoms = []
    bodies = []
    joints = []
    sites = []

    for i in range(consts.NUM_WHITE_KEYS):
        y_coord = (
            -consts.PIANO_KEY_TOTAL_WIDTH * 0.5
            + consts.WHITE_KEY_WIDTH * 0.5
            + i * (consts.WHITE_KEY_WIDTH + consts.SPACING_BETWEEN_WHITE_KEYS)
        )
        bodies.append(
            {
                "name": f"white_key_{consts.WHITE_KEY_INDICES[i]}",
                "pos": [consts.WHITE_KEY_X_OFFSET, y_coord, consts.WHITE_KEY_Z_OFFSET],
            }
        )
        geoms.append(
            {
                "name": f"white_key_geom_{consts.WHITE_KEY_INDICES[i]}",
                "dclass": "white_key",
            }
        )
        joints.append(
            {
                "name": f"white_joint_{consts.WHITE_KEY_INDICES[i]}",
                "dclass": "white_key",
            }
        )
        sites.append(
            {
                "name": f"white_key_site_{consts.WHITE_KEY_INDICES[i]}",
                "dclass": "white_key",
            }
        )

    # Place the lone black key on the far left.
    y_coord = consts.WHITE_KEY_WIDTH + 0.5 * (
        -consts.PIANO_KEY_TOTAL_WIDTH + consts.SPACING_BETWEEN_WHITE_KEYS
    )
    bodies.append(
        {
            "name": f"black_key_{consts.BLACK_TRIPLET_KEY_INDICES[0]}",
            "pos": [consts.BLACK_KEY_X_OFFSET, y_coord, consts.BLACK_KEY_Z_OFFSET],
        }
    )
    geoms.append(
        {
            "name": f"black_key_geom_{consts.BLACK_TRIPLET_KEY_INDICES[0]}",
            "dclass": "black_key",
        }
    )
    joints.append(
        {
            "name": f"black_joint_{consts.BLACK_TRIPLET_KEY_INDICES[0]}",
            "dclass": "black_key",
        }
    )
    sites.append(
        {
            "name": f"black_key_site_{consts.BLACK_TRIPLET_KEY_INDICES[0]}",
            "dclass": "black_key",
        }
    )

    # Place the twin black keys.
    n = 0
    for twin_index in consts.TWIN_GROUP_INDICES:
        for j in range(2):
            y_coord = (
                -consts.PIANO_KEY_TOTAL_WIDTH * 0.5
                + (j + 1) * (consts.WHITE_KEY_WIDTH + consts.SPACING_BETWEEN_WHITE_KEYS)
                + twin_index
                * (consts.WHITE_KEY_WIDTH + consts.SPACING_BETWEEN_WHITE_KEYS)
            )
            bodies.append(
                {
                    "name": f"black_key_{consts.BLACK_TWIN_KEY_INDICES[n]}",
                    "pos": [
                        consts.BLACK_KEY_X_OFFSET,
                        y_coord,
                        consts.BLACK_KEY_Z_OFFSET,
                    ],
                }
            )
            geoms.append(
                {
                    "name": f"black_key_geom_{consts.BLACK_TWIN_KEY_INDICES[n]}",
                    "dclass": "black_key",
                }
            )
            joints.append(
                {
                    "name": f"black_joint_{consts.BLACK_TWIN_KEY_INDICES[n]}",
                    "dclass": "black_key",
                }
            )
            sites.append(
                {
                    "name": f"black_key_site_{consts.BLACK_TWIN_KEY_INDICES[n]}",
                    "dclass": "black_key",
                }
            )
            n += 1

    # Place the triplet black keys.
    n = 1  # Skip the lone black key.
    for triplet_index in consts.TRIPLET_GROUP_INDICES:
        for j in range(3):
            y_coord = (
                -consts.PIANO_KEY_TOTAL_WIDTH * 0.5
                + (j + 1) * (consts.WHITE_KEY_WIDTH + consts.SPACING_BETWEEN_WHITE_KEYS)
                + triplet_index
                * (consts.WHITE_KEY_WIDTH + consts.SPACING_BETWEEN_WHITE_KEYS)
            )
            bodies.append(
                {
                    "name": f"black_key_{consts.BLACK_TRIPLET_KEY_INDICES[n]}",
                    "pos": [
                        consts.BLACK_KEY_X_OFFSET,
                        y_coord,
                        consts.BLACK_KEY_Z_OFFSET,
                    ],
                }
            )
            geoms.append(
                {
                    "name": f"black_key_geom_{consts.BLACK_TRIPLET_KEY_INDICES[n]}",
                    "dclass": "black_key",
                }
            )
            joints.append(
                {
                    "name": f"black_joint_{consts.BLACK_TRIPLET_KEY_INDICES[n]}",
                    "dclass": "black_key",
                }
            )
            sites.append(
                {
                    "name": f"black_key_site_{consts.BLACK_TRIPLET_KEY_INDICES[n]}",
                    "dclass": "black_key",
                }
            )
            n += 1

    # Sort the elements based on the key number.
    names: list[str] = [body["name"] for body in bodies]  # type: ignore
    indices = sorted(range(len(names)), key=lambda k: int(names[k].split("_")[-1]))
    bodies = [bodies[i] for i in indices]
    geoms = [geoms[i] for i in indices]
    joints = [joints[i] for i in indices]
    sites = [sites[i] for i in indices]

    # Now create the corresponding MJCF elements and add them to the piano body.
    for i in range(len(bodies)):
        body = piano_body.add("body", **bodies[i])
        body.add("geom", **geoms[i])
        body.add("joint", **joints[i])
        body.add("site", **sites[i])

    return root


def main() -> None:
    root = build()
    xml = root.to_xml_string()

    save_path = Path("./robots/piano/mjcf/piano_dep20.xml")
    # create directory if it doesn't exist
    save_path.parent.mkdir(parents=True, exist_ok=True)

    with open(save_path, "w") as f:
        f.write(xml)


if __name__ == "__main__":
    main()
