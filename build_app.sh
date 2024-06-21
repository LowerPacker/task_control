#!/bin/bash  

CUR_DIR=$(pwd)
COMM_INTERFACE_DIR=${CUR_DIR}/task_control_interface
APP_DIR=${CUR_DIR}

build_common_interface()
{
    cd ${COMM_INTERFACE_DIR} || { echo "can not access in ${COMM_INTERFACE_DIR}."; exit 1; }  
    # 检查并删除log、install、build目录  
    for dir in ${COMM_INTERFACE_DIR}/log ${COMM_INTERFACE_DIR}/install ${COMM_INTERFACE_DIR}/build; do  
        if [ -d "${dir}" ]; then  
            rm -rf "${dir}"  
            echo "delete ${dir}."  
        fi
    done
    # 执行colcon build命令  
    colcon build --packages-select task_control_interface
    # 检查colcon build命令的退出状态  
    if [ $? -ne 0 ]; then
        echo "colcon build fail."  
        exit 1  
    fi
    source ${COMM_INTERFACE_DIR}/install/local_setup.bash
    # 退出${COMM_INTERFACE_DIR}目录  
    cd - 
}

build_app()
{
    # source ${COMM_INTERFACE_DIR}/install/local_setup.bash
    # 检查并删除log、install、build目录  
    # for dir in ${APP_DIR}/log ${APP_DIR}/install ${APP_DIR}/build; do  
    for dir in ${APP_DIR}/log ${APP_DIR}/build; do  
        if [ -d "${dir}" ]; then  
            rm -rf "${dir}"  
            echo "delete ${dir}."  
        fi
    done
    # 执行colcon build命令
    colcon build --packages-select task_control
    # 检查colcon build命令的退出状态  
    if [ $? -ne 0 ]; then
        echo "colcon build fail."  
        exit 1  
    fi
    # source ${APP_DIR}/install/local_setup.bash
}

main()
{
    build_common_interface
    build_app
    source ${APP_DIR}/install/local_setup.bash
    source ${COMM_INTERFACE_DIR}/install/local_setup.bash
}

main