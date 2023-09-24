## Mistakes

- I didn't run the `source catkin_ws/devel/setup.bash` command since following the MuSHR Foxglove docks didn't build it that way

## Running muSHR Visualization
- Run `mushr_noetic` in your shell
- Run `source ~/.bashrc` in the container shell
- Run `roslaunch mushr_sim teleop.launch` to start the SIM

## Common commands

`roslaunch cse478 teleop.launch`
`roslaunch cse478 foxgloveteleop.launch`
`rostopic list` Lists all topics currently running (requires roscore/roslaunch to be running)