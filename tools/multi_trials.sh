#! /bin/sh
if [ $# -lt 3 ]
then
    echo "not enough params"
    exit 2
fi

config=$1
bag=$2
runs=$3

for i in {1..${runs}}
do
echo "the ${i} time run"
tmux new -d "rosrun vins vins_node $config"

tmux new -d "rosrun prior_locate prior_locate_node $config"

tmux new -d "rosrun prior_fusion prior_fusion_node $config"

# wait for all nodes to start
sleep 8

rosbag play ${bag} && rosnode kill /prior_locate_node /prior_opt_node /vins_estimator
sleep 5
done
