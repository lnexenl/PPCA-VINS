#! /bin/sh
if [ $# -lt 3 ]
then
    echo "not enough params"
    exit 2
fi

config=$1
bag=$2
runs=$3
bagtime=$(printf "%.0f" $(rosbag info -y -k duration $bag))
echo "bag time: $bagtime s"
cnt=$4
result_dir=$5
end_tuns=$(($cnt+$runs-1))

while [ $cnt -le $end_tuns ]
do
echo "the ${cnt} time run"
echo "starting vins node"
tmux new-session -d -s vins rosrun vins vins_node $config
sleep 2
echo "starting locating node"
tmux new-session -d -s locate rosrun prior_locate prior_locate_node $config
sleep 2
echo "starting fusion node"
tmux new-session -d -s fusion rosrun prior_fusion prior_fusion_node $config
sleep 4


tmux new-session -d -s bag rosbag play ${bag}
sleep 10

rosnode info /prior_locate_node >/dev/null 2>&1
if [ $? -ne 0 ]
then
    echo "locating node failed"
    rosnode kill /prior_locate_node /prior_opt_node /vins_estimator
    rosnode kill `rosnode list | grep play`
    tmux kill-server
#     killall screen
    rm -r "$result_dir/$cnt"
    continue
fi

echo "waiting for bag end"
sleep $((30+$bagtime))

rosnode info /prior_locate_node >/dev/null 2>&1
if [ $? -ne 0 ]
then
    echo "locating node failed"
    rosnode kill /prior_locate_node /prior_opt_node /vins_estimator
    rosnode kill `rosnode list | grep play`
    tmux kill-server
#     killall screen
    rm -r "$result_dir/$cnt"
    continue
fi

tmux kill-server
# killall screen
sleep 5
cnt=$(($cnt+1))
done
