#!/usr/bin/env bash

##source export-javi.txt
export APP_REPO=nexus-firmware

export PLATFORMIO_BUILD_FLAGS='-DHAS_NEXUS=1 -DNEXUS_ENABLE_ADV -DAPP_REPO=$APP_REPO -DNEXUS_TRUSTED_CHANNEL_1=\"Nexus\" -DNEXUS_TRUSTED_CHANNEL_2=\"Nexus2\"'



export TARGETS="nrf52_promicro_diy_tcxo station-g2 rak3401-1watt"

for a in $TARGETS
do
	echo Bulding $a
	time pio run -e $a -t clean
	time pio run -e $a
	date
done
