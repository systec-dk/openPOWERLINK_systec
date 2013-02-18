setMode -bscan
setCable -p auto
identify
identifyMPM
assignFile -p 1 -file flash_image_0.mcs
setAttribute -position 1 -attr packageName -value ""
assignFile -p 2 -file flash_image_1.mcs
setAttribute -position 2 -attr packageName -value ""
Program -p 1 -e -v
Program -p 2 -e -v
quit
