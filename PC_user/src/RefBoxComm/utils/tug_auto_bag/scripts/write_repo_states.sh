#!/bin/bash

# execute this script before you start to write into a logfile.
# This script will write all current git commit ids as header into the log file.
#
# example:
#
#     write_repo_states.sh                                      # will write a logfile at /logfiles/log.txt
#
#     write_repo_states.sh /path/to/logfile/                    # will write a logfile at /path/to/logfile/log.txt if it exists already
#
#     write_repo_states.sh /path/to/logfile/log_file_name.txt   # will write a logfile at /path/to/logfile/log_file_name.txt but the 
#                                                               # file has to created first, e.g. touch /path/to/logfile/log_file_name.txt
#     

logpath=$1
echo "${logpath}"
if   [ -d "${logpath}" ]
    then 
    export logfile="${logpath}/log.txt"
    echo "${logpath} is used as logging directory";
elif [ -f ${logpath} ]
    then
    export logfile=$logpath
    echo "${logfile} is used as logging file";
else
    logpath="/logfiles"
    export logfile="${logpath}/log.txt"

    echo "${logpath} is used as logging directory";
fi

echo -n "" > $logfile


function getgitid
{
    directory=$1
    timestamp=$2
    cd $directory
    reponame=`basename $directory`
    gitid=`git rev-parse HEAD`
    echo -e ${timestamp}"\t"${reponame}"\t"${gitid} >> $logfile
}
export -f getgitid


function logit
{
    for path in ${ROS_PACKAGE_PATH//:/ }; do
        find "$path" -maxdepth 2 -mindepth 2 -type d -iname ".git" -exec bash -c 'getgitid `dirname $0` "`date +%Y-%m-%d_%H-%M-%S`\t`date -d now "+%s.%2N"`"' {} \;
    done

    echo -e ${timestamp}"------------------------------------------------------------------------------------------------------------------------" >> $logfile
}

logit