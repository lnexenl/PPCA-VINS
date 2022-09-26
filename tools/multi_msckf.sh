#! /bin/sh
if [ $# -lt 3 ]
then
    echo "not enough params"
    exit 2
fi

bag=$1
runs=$2
bagtime=$(printf "%.0f" $(rosbag info -y -k duration $bag))
echo "bag time: $bagtime s"
cnt=$3
result_dir=$4
end_tuns=$(($cnt+$runs-1))

while [ $cnt -le $end_tuns ]
do

filename_est="$result_dir/$cnt/msckf_est.txt"
echo "the ${cnt} time run"
echo "starting msckf"
tmux new-session -d -s msckf roslaunch ov_msckf subscribe.launch config:="cdd" path_est:="$filename_est"

echo "starting noise node"
tmux new-session -d -s noise_adder roslaunch noise_adder big_noise.launch
sleep 4

tmux new-session -d -s bag rosbag play ${bag} --start=4
sleep 5

echo "waiting for bag end"
sleep $((5+$bagtime))

tmux kill-server
# killall screen
sleep 5
cnt=$(($cnt+1))
done
