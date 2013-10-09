#!/bin/sh

#***************************************************************************#
#                                                                           #
#  SYSTEC electronic GmbH, D-08468 Heinsdorfergrund, Am Windrad 2              #
#  www.systec-electronic.com                                                #
#                                                                           #
#  File:         install.sh                                                 #
#  Description:  install script for ECUcore-5484 firmware                   #
#                                                                           #
#  -----------------------------------------------------------------------  #
#                                                                           #
#  Revision History:                                                        #
#                                                                           #
#  2008/10/07 d.k.:  adapted for openPOWERLINK                              #
#                                                                           #
#****************************************************************************

# This script must be executed from the installation directory

chmod +x  etc/rc.usr
chmod +x  etc/autostart


echo ""
echo " Installation has been finished."
echo " Going to restart the system to activate the new firmware."
echo ""

reboot
