gnome-terminal -x roslaunch rbx1_nav fake_nav_test.launch
gnome-terminal -x rosrun rviz rviz -d `rospack find tiffer_panel`/launch/develope_version.rviz
