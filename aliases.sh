alias build="colcon build --symlink-install && source install/setup.bash"
clean_build(){
    rm -rf build install log
    touch src/planner/COLCON_IGNORE
    build

    rm src/planner/COLCON_IGNORE
    build
}
run_test(){
    refresh
    echo"ros2 launch projects evacuation.launch.py"
    ros2 launch projects evacuation.launch.py
}


rebuild_graph_generator(){
    rm -r build/planner
    build
}


source /opt/ros/humble/setup.bash
alias refresh="source install/setup.bash"
#export CMAKE_MODULE_PATH=$CMAKE_MODULE_PATH:"/opt/ros/rolling/share/"
#export PYTHONPATH=$PYTHONPATH:"/usr/lib/python3/dist-packages"
