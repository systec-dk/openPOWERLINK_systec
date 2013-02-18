setMode -bscan
setCable -p auto
identify
identifyMPM
attachflash -position 1 -spi "N25Q128"
assignfiletoattachedflash -position 1 -file flash_image.mcs
Program -p 1 -dataWidth 1 -spionly -e -v -loadfpga
quit
