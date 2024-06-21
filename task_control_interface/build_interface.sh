#!/bin/bash  

CUR_DIR=$(pwd)
INTERFACE_DIR=${CUR_DIR}

build_interface()
{
    # cd ${INTERFACE_DIR} || { echo "can not access in ${INTERFACE_DIR}."; exit 1; }  
    # 检查并删除log、install、build目录  
    for dir in ${INTERFACE_DIR}/log ${INTERFACE_DIR}/install ${INTERFACE_DIR}/build; do  
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
    source ${INTERFACE_DIR}/install/local_setup.bash
    # 退出${INTERFACE_DIR}目录  
    # cd - 
}
  
main()
{
    build_interface
}

main
