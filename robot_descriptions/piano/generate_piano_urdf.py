"""
python -m robot_descriptions.piano.generate_piano_urdf
"""

from pathlib import Path

try:
    from . import consts
except ImportError:
    import consts  # type: ignore[no-redef]


def _key_xml(
    key_name: str,
    joint_name: str,
    color: str,
    mass: float,
    lx: float,
    ly: float,
    lz: float,
    joint_origin: tuple,
    visual_origin: tuple,
    max_angle: float,
) -> list:
    ixx = mass * (ly ** 2 + lz ** 2) / 12
    iyy = mass * (lx ** 2 + lz ** 2) / 12
    izz = mass * (lx ** 2 + ly ** 2) / 12
    vx, vy, vz = visual_origin
    jx, jy, jz = joint_origin
    return [
        f'  <link name="{key_name}">',
        '    <visual>',
        f'      <origin xyz="{vx} {vy} {vz}" rpy="0 0 0" />',
        '      <geometry>',
        f'        <box size="{lx} {ly} {lz}" />',
        '      </geometry>',
        f'      <material name="{color}" />',
        '    </visual>',
        '    <collision>',
        f'      <origin xyz="{vx} {vy} {vz}" rpy="0 0 0" />',
        '      <geometry>',
        f'        <box size="{lx} {ly} {lz}" />',
        '      </geometry>',
        '    </collision>',
        '    <inertial>',
        f'      <origin xyz="{vx} {vy} {vz}" rpy="0 0 0" />',
        f'      <mass value="{mass}" />',
        f'      <inertia ixx="{ixx}" ixy="0.0" ixz="0.0" iyy="{iyy}" iyz="0.0" izz="{izz}" />',
        '    </inertial>',
        '  </link>',
        '',
        f'  <joint name="{joint_name}" type="revolute">',
        '    <parent link="base" />',
        f'    <child link="{key_name}" />',
        f'    <origin xyz="{jx} {jy} {jz}" rpy="0 0 0" />',
        '    <axis xyz="0 1 0" />',
        f'    <limit lower="0" upper="{max_angle}" effort="{consts.KEY_MAX_TORQUE}" velocity="6.67" />',
        '  </joint>',
        '',
    ]


def build() -> str:
    """Programatically build a piano URDF.

    Returns:
        URDF XML string.
    """
    urdf_lines = [
        '<?xml version="1.0" encoding="utf-8"?>',
        '<robot name="piano">',
        '  <material name="white">',
        f'    <color rgba="{consts.WHITE_KEY_COLOR[0]} {consts.WHITE_KEY_COLOR[1]} {consts.WHITE_KEY_COLOR[2]} {consts.WHITE_KEY_COLOR[3]}" />',
        '  </material>',
        '  <material name="black">',
        f'    <color rgba="{consts.BLACK_KEY_COLOR[0]} {consts.BLACK_KEY_COLOR[1]} {consts.BLACK_KEY_COLOR[2]} {consts.BLACK_KEY_COLOR[3]}" />',
        '  </material>',
        '',
        '  <!-- Base link -->',
        '  <link name="base">',
        '  </link>',
        '',
        '  <!-- Piano body -->',
        '  <link name="body">',
        '    <visual>',
        f'      <origin xyz="{consts.BASE_1_POS[0]} {consts.BASE_1_POS[1]} {consts.BASE_1_POS[2]}" rpy="0 0 0" />',
        '      <geometry>',
        f'        <box size="{consts.BASE_1_SIZE[0]*2} {consts.BASE_1_SIZE[1]*2} {consts.BASE_1_SIZE[2]*2}" />',
        '      </geometry>',
        '    </visual>',
        '    <visual>',
        f'      <origin xyz="{consts.BASE_2_POS[0]} {consts.BASE_2_POS[1]} {consts.BASE_2_POS[2]}" rpy="0 0 0" />',
        '      <geometry>',
        f'         <box size="{consts.BASE_2_SIZE[0]*2} {consts.BASE_2_SIZE[1]*2} {consts.BASE_2_SIZE[2]*2}" />',
        '      </geometry>',
        '    </visual>',
        '    <collision>',
        f'      <origin xyz="{consts.BASE_1_POS[0]} {consts.BASE_1_POS[1]} {consts.BASE_1_POS[2]}" rpy="0 0 0" />',
        '      <geometry>',
        f'         <box size="{consts.BASE_1_SIZE[0]*2} {consts.BASE_1_SIZE[1]*2} {consts.BASE_1_SIZE[2]*2}" />',
        '      </geometry>',
        '    </collision>',
        '    <collision>',
        f'      <origin xyz="{consts.BASE_2_POS[0]} {consts.BASE_2_POS[1]} {consts.BASE_2_POS[2]}" rpy="0 0 0" />',
        '      <geometry>',
        f'         <box size="{consts.BASE_2_SIZE[0]*2} {consts.BASE_2_SIZE[1]*2} {consts.BASE_2_SIZE[2]*2}" />',
        '      </geometry>',
        '    </collision>',
        '    <inertial>',
        f'      <origin xyz="{consts.BASE_1_POS[0]} {consts.BASE_1_POS[1]} {consts.BASE_1_POS[2]}" rpy="0 0 0" />',
        '      <mass value="10.0" />',
        '      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1" />',
        '    </inertial>',
        '  </link>',
        '',
        '  <joint name="base_body_joint" type="fixed">',
        '    <parent link="base" />',
        '    <child link="body" />',
        '    <origin xyz="0 0 0" rpy="0 0 0" />',
        '  </joint>',
        '',
    ]

    # Collect every key with its index so we can emit in sorted order (matches MJCF).
    keys: list[tuple[int, str, float]] = []

    # White keys.
    for i in range(consts.NUM_WHITE_KEYS):
        y_coord = (
            -consts.PIANO_KEY_TOTAL_WIDTH * 0.5
            + consts.WHITE_KEY_WIDTH * 0.5
            + i * (consts.WHITE_KEY_WIDTH + consts.SPACING_BETWEEN_WHITE_KEYS)
        )
        keys.append((consts.WHITE_KEY_INDICES[i], "white", y_coord))

    # Lone black key on the far left.
    y_coord = consts.WHITE_KEY_WIDTH + 0.5 * (
        -consts.PIANO_KEY_TOTAL_WIDTH + consts.SPACING_BETWEEN_WHITE_KEYS
    )
    keys.append((consts.BLACK_TRIPLET_KEY_INDICES[0], "black", y_coord))

    # Twin black keys.
    n = 0
    for twin_index in consts.TWIN_GROUP_INDICES:
        for j in range(2):
            y_coord = (
                -consts.PIANO_KEY_TOTAL_WIDTH * 0.5
                + (j + 1) * (consts.WHITE_KEY_WIDTH + consts.SPACING_BETWEEN_WHITE_KEYS)
                + twin_index
                * (consts.WHITE_KEY_WIDTH + consts.SPACING_BETWEEN_WHITE_KEYS)
            )
            keys.append((consts.BLACK_TWIN_KEY_INDICES[n], "black", y_coord))
            n += 1

    # Triplet black keys.
    n = 1  # Skip the lone black key.
    for triplet_index in consts.TRIPLET_GROUP_INDICES:
        for j in range(3):
            y_coord = (
                -consts.PIANO_KEY_TOTAL_WIDTH * 0.5
                + (j + 1) * (consts.WHITE_KEY_WIDTH + consts.SPACING_BETWEEN_WHITE_KEYS)
                + triplet_index
                * (consts.WHITE_KEY_WIDTH + consts.SPACING_BETWEEN_WHITE_KEYS)
            )
            keys.append((consts.BLACK_TRIPLET_KEY_INDICES[n], "black", y_coord))
            n += 1

    keys.sort(key=lambda k: k[0])

    for index, color, y_coord in keys:
        key_name = f"{color}_key_{index}"
        joint_name = f"{color}_joint_{index}"
        if color == "white":
            joint_origin = (
                consts.WHITE_KEY_X_OFFSET
                + consts.WHITE_KEY_LENGTH / 2
                - consts.WHITE_KEY_TOTAL_LENGTH,
                y_coord,
                consts.WHITE_KEY_Z_OFFSET,
            )
            visual_origin = (
                consts.WHITE_KEY_TOTAL_LENGTH
                - consts.WHITE_KEY_LENGTH
                + consts.WHITE_KEY_LENGTH / 2,
                0,
                0,
            )
            mass = consts.WHITE_KEY_MASS
            lx, ly, lz = (
                consts.WHITE_KEY_LENGTH,
                consts.WHITE_KEY_WIDTH,
                consts.WHITE_KEY_HEIGHT,
            )
            max_angle = consts.WHITE_KEY_JOINT_MAX_ANGLE
        else:
            joint_origin = (
                consts.BLACK_KEY_X_OFFSET
                + consts.BLACK_KEY_LENGTH / 2
                - consts.BLACK_KEY_TOTAL_LENGTH,
                y_coord,
                consts.BLACK_KEY_Z_OFFSET,
            )
            visual_origin = (
                consts.BLACK_KEY_TOTAL_LENGTH
                - consts.BLACK_KEY_LENGTH
                + consts.BLACK_KEY_LENGTH / 2,
                0,
                0,
            )
            mass = consts.BLACK_KEY_MASS
            lx, ly, lz = (
                consts.BLACK_KEY_LENGTH,
                consts.BLACK_KEY_WIDTH,
                consts.BLACK_KEY_HEIGHT,
            )
            max_angle = consts.BLACK_KEY_JOINT_MAX_ANGLE
        urdf_lines.extend(
            _key_xml(
                key_name=key_name,
                joint_name=joint_name,
                color=color,
                mass=mass,
                lx=lx,
                ly=ly,
                lz=lz,
                joint_origin=joint_origin,
                visual_origin=visual_origin,
                max_angle=max_angle,
            )
        )

    # Close the robot tag
    urdf_lines.append('</robot>')

    return '\n'.join(urdf_lines)


def main() -> None:
    urdf_content = build()

    save_path = Path("./robots/piano/urdf/piano_dep20.urdf")
    # create directory if it doesn't exist
    save_path.parent.mkdir(parents=True, exist_ok=True)

    with open(save_path, "w") as f:
        f.write(urdf_content)


if __name__ == "__main__":
    main()
