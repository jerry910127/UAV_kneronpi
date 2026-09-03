#!/bin/sh -ex
(
cd $(dirname $(realpath $0))

#./uvc-gadget_mount.sh install

cd /work/scripts
./udpServ.py 54321 &

)
