# ROBO-FEUP

Directory structure:

docs: documentation used

include: header files of the program

src: source code of the program

stdr_files: contains resource files(robot + map) and launchers of the v and w maps

To run:

move this project in ~/catkin_ws/src/robo-feup (or another catkin workspace)

install stdr from its git repository

run 'sh prepare.sh' to move the resource files to the stdr directory(change workspace directory in the script if needed)

run 'catkin_make' on base directory to build project

run 'roslaunch stdr_launchers standart_env_v.launch' to launch the v map(or w)

run 'rosrun robo_feup robo_wall' to run the robot controller 

