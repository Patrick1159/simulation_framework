
# 查看S1静态走廊场景
gazebo /home/patrick/sim_dev/simulation_framework/exp/worlds/S1_static_corridor.world

# full pipeline
./src/exp/scripts/run_avoidance_exp.sh --subscene random_multi --method ours_no_sam  --duration 20 --repeats 1