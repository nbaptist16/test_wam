RVIZ INTERACTABLE SIM [working]:
nckmlb@xpso:~/test_wam$ roslaunch urdf_tutorial display.launch model:=src/trial0/urdf/barrett_model/robots/wam7_bhand.urdf.xacro gui:=true
- make sure fixed frame (edit @ global options) and grid reference frame both set to /wam/base_link
- can probably sub rviz (see launch params) to test_joint_sliders.rviz


GAZEBO TODO SIM [working]:
roslaunch trial0 wam0.launch
- todo: fix z-coord at spawn (currently floating)


MOVEIT @ RVIZ:
- nckmlb@xpso:~/test_wam$ roslaunch barrett_wam_moveit_config demo.launch


RVIZ + GAZEBO:
nckmlb@xpso:~/test_wam$ roslaunch trial0 wam0.launch rviz:=true
- rviz sim initially looks angry. Just play the simulation in gazebo (default at start is paused) and it should be fine



ACKNOWLEDGEMENTS:
Thank you to JHU for the [barrett libraries](https://github.com/jhu-lcsr?q=barrett&type=&language=&sort=)