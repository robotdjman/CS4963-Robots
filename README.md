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

## X11 XQuartz Setup
For more info check this out: https://gist.github.com/sorny/969fe55d85c9b0035b0109a31cbcb088

- Make sure you have XQuarts running
- Make sure you have typed `xhost +localhost` every time you open xquartz
- Run `ps aux | grep Xquartz` on your host machine, if it says "nolisten" anywhere make sure to enable network settings
- Run `export DISPLAY=host.docker.internal:0`
under XQuartz Preferences (You can get to this via the top mac nav bar with xquarts open) -> Security -> Allow network clients
## Useful Links
https://github.com/prl-mushr/mushr_pf/blob/master/src/mushr_pf/motion_model.py