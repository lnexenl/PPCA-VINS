#! /bin/zsh
# Copyright (c) 2022. Lin Zihan
#

if [ ! -d "plot" ]; then
    mkdir plot
fi
if [ ! -d "plot/ape" ]; then
    mkdir plot/ape
fi
if [ ! -d "plot/rpe" ]; then
    mkdir plot/rpe
fi
if [ ! -d "result" ]; then
    mkdir result
fi
if [ ! -d "result/ape" ]; then
    mkdir result/ape
fi
if [ ! -d "result/rpe" ]; then
    mkdir result/rpe
fi
rm result/ape/*.zip
rm result/rpe/*.zip
for i in {1..5}
do
    echo "ape_${i}"
    evo_ape tum --align -r full --pose_relation full --plot_mode xz ./groundtruth vio_$i.txt --save_results result/ape/vio_$i.zip --plot_mode xy --save_plot ./plot/ape/vio_ --plot_x_dimension seconds --plot_full_ref
    evo_ape tum --align -r full --pose_relation full --plot_mode xz ./groundtruth prior_$i.txt --save_results result/ape/prior_$i.zip  --plot_mode xy --save_plot ./plot/ape/prior_ --plot_x_dimension seconds --plot_full_ref
done

for i in {1..5}
do
    echo "rpe_${i}"
    evo_rpe tum --align -r full ./groundtruth vio_$i.txt -d 50 -u f --save_results result/rpe/vio_$i.zip --plot_mode xy --save_plot ./plot/rpe/vio_ --plot_x_dimension seconds --plot_full_ref
    evo_rpe tum --align -r full ./groundtruth prior_$i.txt -d 50 -u f --save_results result/rpe/prior_$i.zip  --plot_mode xy --save_plot ./plot/rpe/prior_ --plot_x_dimension seconds --plot_full_ref
done


# evo_ape tum --align -r full --pose_relation angle_deg --plot_mode xz ./gt.txt vio.txt --save_results vio_rre.zip --plot_mode xy --save_plot ./plot/rre_vio --plot_x_dimension seconds --plot_full_ref
# evo_ape tum --align -r full --pose_relation angle_deg --plot_mode xz ./gt.txt prior.txt --save_results prior_rre.zip  --plot_mode xy --save_plot ./plot/rre_prior --plot_x_dimension seconds --plot_full_ref
evo_res result/ape/* --use_filenames --use_rel_time -p --save_plot ./plot/ape
evo_res result/rpe/* --use_filenames --use_rel_time -p --save_plot ./plot/rpe
