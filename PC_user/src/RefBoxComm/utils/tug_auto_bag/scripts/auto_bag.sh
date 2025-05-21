#!/bin/bash

# get all artibutes
log_path=$1
bag_path=$2
bag_name_prefix=$3
bag_duration=$4
arg_list=$(echo $5)

# check existens of given folder for bagfiles and logfiles
if [ ! -d "$log_path" ]; then
  echo "$log_path does not exist! Please create the folder or create a symlink ('ln -s <your_logfile_location> $log_path')."
  exit -1
fi

if [ ! -d "$bag_path" ]; then
  echo "$bag_path does not exist! Please create the folder or create a symlink ('ln -s <your_bagfile_location> $bag_path')."
  exit -1
fi

if [ ! -d "${bag_path}/tmp" ]; then
  mkdir -p "${bag_path}/tmp"
fi

if [ ! -d "${bag_path}/save" ]; then
  mkdir -p "${bag_path}/save"
fi

# define logfile parameters
log_time=$(date +%Y-%m-%d_%H-%M-%S)
log_file="log_file"
log_autobag="log_autobag"
log_type="txt"

# real file names with timestamp
log_autobag_real="${log_path}/${log_autobag}_${log_time}.${log_type}"
log_autobag_virtual="${log_path}/${log_autobag}.${log_type}"

# virtual file names without timestamp which are used by the system for logging
log_file_real="${log_path}/${log_file}_${log_time}.${log_type}"
log_file_virtual="${log_path}/${log_file}.${log_type}"

# create logfiles
touch "${log_autobag_real}"
touch "${log_file_real}"

# set symlink for logfiles
ln -sf "${log_autobag_real}" "${log_autobag_virtual}"
ln -sf "${log_file_real}" "${log_file_virtual}"

rosrun tug_auto_bag write_repo_states.sh ${log_file_real}
rosrun tug_auto_bag write_repo_states.sh ${log_autobag_real}

# delete temporary bag files
# if rosbag crashes and a bag_save is running, it is possible that the following
# command removes the bagfile before it is stored in ${bag_path}/save
find "${bag_path}/tmp" -type f -delete

# start recording
if [ ! "$arg_list" ];then
  rosbag record -a --split --duration=${bag_duration} -o ${bag_path}/tmp/${bag_name_prefix}
else
  rosbag record -a --split --duration=${bag_duration} -o ${bag_path}/tmp/${bag_name_prefix} -x ${arg_list// /|}
fi