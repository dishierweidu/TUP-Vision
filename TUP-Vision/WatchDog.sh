#!/bin/bash 
 
sec=1 
cnt=0 
name=RP_Infantry_Plus 
Thread=ps -ef | grep $name | grep -v "grep" 
cd /home/s305-nuc5/kevin/build-$name-Desktop-Debug/ 
make clean && make -j 
while [ 1 ] 
do 
count=ps -ef | grep $name | grep -v "grep" | wc -l 
echo "Thread count: $count" 
echo "Expection count: $cnt" 
if [ $count -gt 1 ]; then 
 echo "The $name is still alive!" 
 sleep $sec 
else  
 echo "Starting $name..." 
 cd ~ && ./ttyUSB.sh 
    cd /home/s305-nuc5/kevin/build-$name-Desktop-Debug/ 
    gnome-terminal -x bash -c "./$name;exec bash;" 
    echo "$name has started!"   
 sleep $sec 
 ((cnt=cnt+1)) 
 if [ $cnt -gt 9 ]; then 
  reboot 
 fi 
fi 
done
