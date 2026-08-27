#!/bin/sh


EXEC_PATH=/work/utils/peripheral
#PERM_RIGHT=sudo
UART_PORT=_ttyS3


#
# -o, Data Output Mode, 0x41
# -m, Measurement Mode, 0x42
# -f, Frame Time/Rate, 0x43
#
# -t, Measurement: Single Shot, 0x10
#

${PERM_RIGHT} ${EXEC_PATH}/uart_tof${UART_PORT} -o
#${PERM_RIGHT} ${EXEC_PATH}/uart_tof${UART_PORT} -f
${PERM_RIGHT} ${EXEC_PATH}/uart_tof${UART_PORT} 02 43 00 01 86 A0 73 03

${PERM_RIGHT} ${EXEC_PATH}/uart_tof${UART_PORT} -m
${PERM_RIGHT} ${EXEC_PATH}/uart_tof${UART_PORT} 02 44 01 DA 03
#${PERM_RIGHT} ${EXEC_PATH}/uart_tof${UART_PORT} 02 44 02 FD 03

while true;  do
    ${PERM_RIGHT} ${EXEC_PATH}/uart_tof${UART_PORT} -t
    sleep 0.5
done
