# Project Instructions

# Agent Behavior
- **Keep responses concise:** Provide short, direct answers without overly
  verbose explanations.

# Project Scope
- **Directory restrictions:** ONLY modify files within this git repo's
  directory. You can read files from outside this project's directory.

## Build Commands
When modifying code in this repository and asked to build, you must run the
build inside our Docker container. Use `docker exec` to run the build in the
workspace directory:

```bash
# Replace <package_name> with the name of the package that was modified.
docker exec --workdir /home/ros/workspaces/jazzy_ws --user ros rtp_jazzy_nvidia /bin/bash -ic 'colcon-build --packages-up-to <package_name>'
```

## Testing
Do not run `colcon test` ever.

<!-- ## Linting -->
<!-- This repository uses `pre-commit`. To run it, execute it inside the Docker container: -->
<!-- ```bash -->
<!-- docker exec --workdir /home/ros/workspaces/jazzy_ws/src/rif_ros --user ros rtp_jazzy_nvidia /bin/bash -ic 'pre-commit run --all-files' -->
<!-- ``` -->
