#!/bin/bash

bag_file="$1"
filtered_topics=(${@:2})

topics=$(rosbag info -y --key=topics bag/2023-08-19-05-31-51.bag | awk '{print $3}')
echo $topics
filtered_topics_string=""
for topic in $topics; do
    skip_topic=false
    for filtered_topic in "${filtered_topics[@]}"; do
        if [[ "$topic" == "$filtered_topic" ]]; then
            skip_topic=true
            break
        fi
    done
    if [[ $skip_topic == false ]]; then
        filtered_topics_string+=" $topic"
    fi
done

echo "Playing bag file '$bag_file' with filtered topics: $filtered_topics_string"
rosbag play "$bag_file" --topics$filtered_topics_string
