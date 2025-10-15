Instalar Python 3.10: Requisito para que MATLAB pueda construir custom messages

    sudo apt install python3.10 python3.10-venv python3.10-dev

Para leer transformaciones entre marcos de referencia

    ros2 run tf2_ros tf2_echo ur3e_base_link ee_link

Set GCC compiler to use (quizás necesario para Matlab)
    sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-12 60
    sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-12 60

Limpiar workspace
    cd ~/DefectsExp_UR/ur_ws
    rm -rf build/ install/ log/

restringir colcon a src
    colcon build --base-paths src

Habilitar puerto para el sensor Anyskin
    sudo chmod a+rw /dev/ttyACM0

ROS2 record bags
    ros2 bag record -o BagName /ee_pose_fast /anyskin_measurements

Reset baseline del Anyskin
    ros2 service call /reset_baseline std_srvs/srv/Trigger

Test a target pose
    ros2 run testing pub_pose 

# LANZAR UR:

Construir controller workspace (~/DefectsExp_UR/controller_ws):
    colcon build --packages-skip cartesian_controller_simulation cartesian_controller_tests --cmake-args -DCMAKE_BUILD_TYPE=Release

Sourcear controlador en una capa:
    source install/local_setup.zsh

Añadir path del ambiente python para que ROS lo encuentre (añadido a .zshrc)
    export PYTHONPATH=/home/gustavo-fuentevilla/DefectsExp_UR/pyenv/lib/python3.12/site-packages:$PYTHONPATH

Construir paquetes para el UR: [Opción segura --base-paths src]
    colcon build --symlink-install 

Sourcear xd, una capa encima:
    source install/local_setup.zsh

Ejecutar el launch:
    ros2 launch easy_ur_control easy_ur_launcher.launch.py robot_ip:=192.168.100.10 ur_type:=ur3e ctrl:=cartesian_compliance_controller

Esto último ya ejecuta el nodo del Anyskin y el nodo sincronizador
--- alternativamente se pueden ejecutar por separado con:

Ejecutar el nodo del anyskin:
	ros2 run anyskin_sensor_publisher anyskin_sensor_node

Ejecutar el nodo sincronizador:
	ros2 run sync_data sync_data
	
Ejecutar plotjuggler y cargar el layout
    ros2 run plotjuggler plotjuggler

Ejecutar el launch de rutina inicial ur_motion_routines
    ros2 launch ur_motion_routines ur_motion_routines.launch.py
