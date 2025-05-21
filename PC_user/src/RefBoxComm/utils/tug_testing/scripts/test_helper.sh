#!/bin/bash

catkin_make run_tests | grep " run_tests.py: verify result" > tmp.txt

totalRes=0
for i in $(awk '{print $5}' tmp.txt)
do
	res=$(cat ${i:1:-1} | grep -c 'failures="0"')
	#echo $i
	#echo $res
	if [ $res -le 0 ]
	then
		echo ${i:1:-1}
		totalRes=1
	fi
done

exit $totalRes

