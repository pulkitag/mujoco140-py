#!/bin/sh
parent_path=$( cd "$(dirname "${BASH_SOURCE}")" ; pwd -P )
echo $parent_path
#mujoco_path=$MUJOCO_PY_BUNDLE_PATH/osx/mujoco
mujoco_path=/work4/pulkitag-code/pkgs/mujoco/mjpro131/include
echo $mujoco_path
rm /tmp/code_gen_mujoco.h
cat $mujoco_path/mjdata.h >> /tmp/code_gen_mujoco.h && \
  cat $mujoco_path/mjmodel.h >> /tmp/code_gen_mujoco.h && \
  cat $mujoco_path/mjrender.h >> /tmp/code_gen_mujoco.h && \
  cat $mujoco_path/mjvisualize.h >> /tmp/code_gen_mujoco.h && \
  ruby $parent_path/codegen.rb /tmp/code_gen_mujoco.h $mujoco_path/mjxmacro.h > $parent_path/mjtypes.py
  #ruby $parent_path/codegen.rb /tmp/code_gen_mujoco.h $mujoco_path/mjxmacro.h
