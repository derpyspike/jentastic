#!/usr/bin/env bash

##source export-javi.txt
# Nexus-28006 (2026-09-16): APP_REPO for admins builder is
# `jt-firmware` (short for jentastic-firmware), not `jentastic-firmware`.
# Matches the BUILDER_ADMINS line in build-nexus.sh:69.
export APP_REPO=jt-firmware

export PLATFORMIO_BUILD_FLAGS='-DHAS_NEXUS=1 -DAPP_REPO=$APP_REPO -DNEXUS_TRUSTED_CHANNEL_1=\"Nexus\" -DNEXUS_TRUSTED_CHANNEL_2=\"JenT\"'



export TARGETS="nrf52_promicro_diy_tcxo xiaowa-esp32c3 t-deck-tft heltec-v4 rak_wismeshtag seeed_solar_node"

for a in $TARGETS
do
	echo Bulding $a
	time pio run -e $a -t clean
	time pio run -e $a
	date
done
