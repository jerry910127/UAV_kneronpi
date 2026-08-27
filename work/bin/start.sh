#!/bin/sh -ex
(
cd $(dirname $(realpath $0))

EXEC_PATH=.	#../ai_application/nnm/build/bin

#export LD_LIBRARY_PATH=$PWD/lib

#./example_nnm_sensor_venc1280x720f30 2>&1 | tee /tmp/kneopi.log &
#${EXEC_PATH}/rgbcam &
#${EXEC_PATH}/rgbir &

#${EXEC_PATH}/venc_receiver &

)
