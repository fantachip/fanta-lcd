#!/bin/bash

USB0=/dev/ttyUSB0

# reset the display
python -c 'print "\\c\\x0"' > $USB0

# splash 
python -c 'print "\\c FANTACHIP-LCD\\g\x01\x00  CONTROLLER"' > $USB0
sleep 2
python -c 'print "\\g\x01\x00     Basic      "' > $USB0
sleep 2 
python -c 'print "\\g\x01\x00Brought to you..  "' > $USB0
sleep 2
python -c 'print "\\g\x01\x00by FANTA-CHIP.  "' > $USB0
sleep 2
# write welcome
python -c 'print "\\c\\x3\\g\x00\x10Welcome!\\g\x01\x10PROGRAMMABLE LCD"' > $USB0
for i in {0..15}
do
	python -c 'print "\\h"' > $USB0
done
sleep 3
python -c 'print "\\cNO CURSOR:\\x0"' > $USB0
sleep 3
python -c 'print "\\cBLINK:\\x1"' > $USB0
sleep 3
python -c 'print "\\cJUST CURSOR:\\x2"' > $USB0
sleep 3
python -c 'print "\\cBLINK+CURSOR:\\x3"' > $USB0
sleep 3
python -c 'print "\\c\\g\x00\x00   CLEARING..   "' > $USB0
sleep 3
python -c 'print "\\c"' > $USB0
sleep 3
python -c 'print "STO \"rabbit\"\\g\x01\x00in adr 0x01.."' > $USB0
python -c 'print "\\s\x01rabbit\x00"' > $USB0
sleep 3
python -c 'print "\\cUSE 0x01.."' > $USB0
sleep 3
python -c 'print "\\cStored string:\\g\x01\x00\\r\x01"' > $USB0
sleep 3
python -c 'print "\\c FANTACHIP-LCD\\g\x01\x00   CONTROLLER"' > $USB0
sleep 2
python -c 'print "\\g\x01\x00      Basic      "' > $USB0
sleep 2 
python -c 'print "\\g\x01\x00  Brought to you  "' > $USB0
sleep 2
python -c 'print "\\g\x01\x00 by FANTA-CHIP."' > $USB0
sleep 3
for i in {1..32}
do
	python -c 'print "\\h"' > $USB0
done

# testing invalid operations
python -c 'print "\\cTest invalid r"' > $USB0
sleep 3
python -c 'print "\\r10\\r02"' > $USB0
sleep 3
python -c 'print "\\cTest invalid adr"' > $USB0
sleep 3
python -c 'print "\\g\xff\xff"' > $USB0
sleep 3
python -c 'print "\\cTest overflow"' > $USB0
sleep 3
python -c 'print "\\chasdfyuasydiufyoiauhoisdbfabsodiufgoaiusydfoibasoidufoiausydoifuabsoidufoaisudyfoiahsdoiufyaoisdbfoiuasydoifuhasidufhoiuasydoifuhasoiudhfoiausydoifbasoidufyoaisudhbfoiausydoifuabhsoidufhyaoisudfoaiusbdiofuasoidufyaoisdbfoiuahsoiduyfoaisudbfoaiusbdfoiuasydofiuabsodiufbaoisudhfoauisydofiubasoidufboaisudgfoiuabsdoiufaosiudbfoiausbdoifubasoiudbfoaisubdoifubasoidfboaiusbdfiausbodiufbasdf"' > $USB0
sleep 3
python -c 'print "\\c  FANTACHIP-LCD\\g\x01\x00   CONTROLLER"' > $USB0

