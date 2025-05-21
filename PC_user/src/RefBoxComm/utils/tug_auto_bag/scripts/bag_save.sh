#!/bin/bash

save_duration="3" # define minutes that should be saved at least

log_path="/logfiles"
log_filename="log_autobag"
log_type="txt"
log_file="${log_path}/${log_filename}.${log_type}"

bag_path="/bagfiles"

savetime=$(date +%Y-%m-%d_%H-%M-%S)
savetime_s=$(date +%s)
savetime_n=$(date +%s.%2N)

# Waiting for auto_bag to start recording
stop=10 # wait for auto_bag to come up. timeout=stop*sleeptime
counter=0
while [ ! -f ${bag_path}/tmp/*.active ]
do
  sleep 1
  counter=$((counter+1))
  # stop this retry after 10 seconds
  if [ "$counter" -gt "$stop" ]
  then
    echo "No active bagfile! Is auto_bag running?"
    exit 1
  fi
done
echo "Waiting for active file..."
active_bag=$(find ${bag_path}/tmp/*.active)
echo $active_bag

# Waiting for auto_bag to finished current used bagfile.
stop=36 # wait for auto_bag to finish current bagfile. timeout=stop*sleeptime
counter=0
while [ -f $active_bag ]
do
  sleep 5
  counter=$((counter+1))
  
  # stop this retry after 180 seconds
  if [ "$counter" -gt "$stop" ]
  then
    echo "auto_bag did not finish the current bagfile in time. Maybe recording duration is longer than this timeout!"
    exit 1
  fi
done

# Extract informations of bagfile to do some calculations
bagfile=${active_bag%.*}
bagfilebase=$(basename "$bagfile")
bagstarttime=$(rosbag info ${bagfile}  | awk '$1 ~ /^ *start/ ' | awk -F '[(._)]' '{print $(NF-2)}')
difference=$(( savetime_s - bagstarttime ))

# Moving the non active files (create hardlink)
find ${bag_path}/tmp/* -mmin -${save_duration} -type f -name "*[^{active}]" -exec ln "{}" ${bag_path}/save/ \;

(
  # Wait for lock on /var/lock/.myscript.exclusivelock (fd 200) for 10 seconds
  flock -x -w 10 200 || exit 1
  # write into log_file
  echo "write into log_file $log_file"
  echo -e "${savetime}\t${savetime_n}\tSave bagfile request for '${bagfilebase}' at second '${difference}'" >> $log_file

) 200>/var/lock/.myscript.exclusivelock

