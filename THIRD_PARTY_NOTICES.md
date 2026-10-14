# Third-party model assets

The Reach Alpha 5 robot description, DAE meshes and related configuration in
`vendor/reach_description/` are copied from:

- Repository: https://github.com/Robotic-Decision-Making-Lab/reach
- Revision: `5e37b9df49fb1a47fb2c0b02ba641ef854d552f9`
- Original directory: `reach_description/`
- Copyright: Evan Palmer, 2025
- License: [original notice](vendor/reach_description/LICENSE)

The generated URDF/MJCF files and OBJ assets in `models/` derive from this
description. A copy of the upstream notice is retained in `models/assets/LICENSE`.
The upstream license limits use to Reach Robotics products and prohibits use in
other contexts without the specified authorization. This repository does not
relicense those assets or grant broader rights.

When adding a different robot, supply that robot's own appropriately licensed
description, geometry and parameters. Python dependencies retain their respective
licenses; installation versions are listed in `requirements-lock.txt`.
