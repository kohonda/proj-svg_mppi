#!/bin/bash

# check yq installed
if ! type "yq" > /dev/null 2>&1; then
    echo "[ERROR] yq is not installed. Please install yq."
    exit 1
fi

MAP_NAME="berlin"
IS_VISUALIZE=false

CURRENT_DIR=$(cd $(dirname $0) && pwd)
SUZ_WS=$(cd $(dirname $0) && cd .. && pwd)

# parse template yaml
default_yaml=$SUZ_WS/src/mppi_controller/config/mppi_controller.yaml
tmp_yaml=$SUZ_WS/src/eval/tmp/tmp.yaml

###########################
# Compare path tracking (pt) performance with baseline methos 
###########################
NUM_STATIC_OBSTACLES=0 # No obstacle avoidance
NUM_TRIALS=30
###### svg_mppi (Proposed method) ######
echo "[INFO] Evaluating svg_mppi..."
cp $default_yaml $tmp_yaml
yq eval '.mpc_mode = "svg_mppi"' -i $tmp_yaml
yq eval '.is_visualize_mppi = false' -i $tmp_yaml
./eval.sh "pt_svg_mppi" $tmp_yaml $NUM_TRIALS $NUM_STATIC_OBSTACLES $IS_VISUALIZE
echo "[INFO] Finished evaluating svg_mppi"

###### sv_mpc ######
echo "[INFO] Evaluating sv_mpc..."
cp $default_yaml $tmp_yaml
yq eval '.mpc_mode = "sv_mpc"' -i $tmp_yaml
yq eval '.is_visualize_mppi = false' -i $tmp_yaml
./eval.sh "pt_sv_mpc" $tmp_yaml $NUM_TRIALS $NUM_STATIC_OBSTACLES $IS_VISUALIZE
echo "[INFO] Finished evaluating sv_mpc"

###### reverse_mppi ######
echo "[INFO] Evaluating reverse_mppi..."
cp $default_yaml $tmp_yaml
yq eval '.mpc_mode = "reverse_mppi"' -i $tmp_yaml
yq eval '.is_visualize_mppi = false' -i $tmp_yaml
./eval.sh "pt_reverse_mppi" $tmp_yaml $NUM_TRIALS $NUM_STATIC_OBSTACLES $IS_VISUALIZE
echo "[INFO] Finished evaluating reverse_mppi"

###### forward_mppi ######
STEER_COVS=(0.1 0.075 0.05 0.025)
for steer_cov in ${STEER_COVS[@]}; do
    echo "[INFO] Evaluating forward_mppi (steer_cov: $steer_cov)..."
    cp $default_yaml $tmp_yaml
    yq eval '.mpc_mode = "forward_mppi"' -i $tmp_yaml
    yq eval '.is_visualize_mppi = false' -i $tmp_yaml
    yq eval ".forward_mppi.steer_cov = $steer_cov" -i $tmp_yaml
    ./eval.sh "pt_forward_mppi(cov:${steer_cov})" $tmp_yaml $NUM_TRIALS $NUM_STATIC_OBSTACLES $IS_VISUALIZE
    echo "[INFO] Finished evaluating forward_mppi (steer_cov: $steer_cov)"
done

###########################
# Compare obstacle avoidance (oa) performance with baseline methos 
###########################

NUM_STATIC_OBSTACLES=5 # With obstacle avoidance
NUM_TRIALS=100
###### svg_mppi (Proposed method) ######
echo "[INFO] Evaluating svg_mppi..."
cp $default_yaml $tmp_yaml
yq eval '.mpc_mode = "svg_mppi"' -i $tmp_yaml
yq eval '.is_visualize_mppi = false' -i $tmp_yaml
./eval.sh "oa_svg_mppi" $tmp_yaml $NUM_TRIALS $NUM_STATIC_OBSTACLES $IS_VISUALIZE
echo "[INFO] Finished evaluating svg_mppi"

###### sv_mpc ######
echo "[INFO] Evaluating sv_mpc..."
cp $default_yaml $tmp_yaml
yq eval '.mpc_mode = "sv_mpc"' -i $tmp_yaml
yq eval '.is_visualize_mppi = false' -i $tmp_yaml
./eval.sh "oa_sv_mpc" $tmp_yaml $NUM_TRIALS $NUM_STATIC_OBSTACLES $IS_VISUALIZE
echo "[INFO] Finished evaluating sv_mpc"

###### reverse_mppi ######
echo "[INFO] Evaluating reverse_mppi..."
cp $default_yaml $tmp_yaml
yq eval '.mpc_mode = "reverse_mppi"' -i $tmp_yaml
yq eval '.is_visualize_mppi = false' -i $tmp_yaml
./eval.sh "oa_reverse_mppi" $tmp_yaml $NUM_TRIALS $NUM_STATIC_OBSTACLES $IS_VISUALIZE
echo "[INFO] Finished evaluating reverse_mppi"

###### forward_mppi ######
STEER_COVS=(0.1 0.075 0.05 0.025)
for steer_cov in ${STEER_COVS[@]}; do
    echo "[INFO] Evaluating forward_mppi (steer_cov: $steer_cov)..."
    cp $default_yaml $tmp_yaml
    yq eval '.mpc_mode = "forward_mppi"' -i $tmp_yaml
    yq eval '.is_visualize_mppi = false' -i $tmp_yaml
    yq eval ".forward_mppi.steer_cov = $steer_cov" -i $tmp_yaml
    ./eval.sh "oa_forward_mppi(cov:${steer_cov})" $tmp_yaml $NUM_TRIALS $NUM_STATIC_OBSTACLES $IS_VISUALIZE
    echo "[INFO] Finished evaluating forward_mppi (steer_cov: $steer_cov)"
done

###########################
# Abalation study: without covariance adaptation
###########################

STEER_COVS=(0.1 0.075 0.05 0.025)

###### Path tracking ######
NUM_STATIC_OBSTACLES=0 # No obstacle avoidance
NUM_TRIALS=30
for steer_cov in ${STEER_COVS[@]}; do
    echo "[INFO] Evaluating svg_mppi (steer_cov: $steer_cov)..."
    cp $default_yaml $tmp_yaml
    yq eval '.mpc_mode = "svg_mppi"' -i $tmp_yaml
    yq eval '.is_visualize_mppi = false' -i $tmp_yaml
    yq eval ".svg_mppi.steer_cov = $steer_cov" -i $tmp_yaml
    yq eval ".svg_mppi.is_covariance_adaptation = false" -i $tmp_yaml
    ./eval.sh "pt_svg_mppi(cov:${steer_cov})" $tmp_yaml $NUM_TRIALS $NUM_STATIC_OBSTACLES $IS_VISUALIZE
    echo "[INFO] Finished evaluating svg_mppi (steer_cov: $steer_cov)"
done

###### Obstacle avoidance ######

NUM_STATIC_OBSTACLES=5 # With obstacle avoidance
NUM_TRIALS=100
for steer_cov in ${STEER_COVS[@]}; do
    echo "[INFO] Evaluating svg_mppi (steer_cov: $steer_cov)..."
    cp $default_yaml $tmp_yaml
    yq eval '.is_visualize_mppi = false' -i $tmp_yaml
    yq eval '.mpc_mode = "svg_mppi"' -i $tmp_yaml
    yq eval ".svg_mppi.steer_cov = $steer_cov" -i $tmp_yaml
    yq eval ".svg_mppi.is_covariance_adaptation = false" -i $tmp_yaml
    ./eval.sh "oa_svg_mppi(cov:${steer_cov})" $tmp_yaml $NUM_TRIALS $NUM_STATIC_OBSTACLES $IS_VISUALIZE
    echo "[INFO] Finished evaluating svg_mppi (steer_cov: $steer_cov)"
done

###########################
# Abalation study: without nominal solution
###########################

###### Path tracking ######
NUM_STATIC_OBSTACLES=0 # No obstacle avoidance
NUM_TRIALS=30
echo "[INFO] Evaluating svg_mppi without nominal solution..."
cp $default_yaml $tmp_yaml
yq eval '.mpc_mode = "svg_mppi"' -i $tmp_yaml
yq eval '.is_visualize_mppi = false' -i $tmp_yaml
yq eval ".svg_mppi.is_use_nominal_solution = false" -i $tmp_yaml
./eval.sh "pt_svg_mppi_wo_nominal" $tmp_yaml $NUM_TRIALS $NUM_STATIC_OBSTACLES $IS_VISUALIZE
echo "[INFO] Finished evaluating svg_mppi without nominal solution"

###### Obstacle avoidance ######
NUM_STATIC_OBSTACLES=5 # With obstacle avoidance
NUM_TRIALS=100
echo "[INFO] Evaluating svg_mppi without nominal solution..."
cp $default_yaml $tmp_yaml
yq eval '.mpc_mode = "svg_mppi"' -i $tmp_yaml
yq eval '.is_visualize_mppi = false' -i $tmp_yaml
yq eval ".svg_mppi.is_use_nominal_solution = false" -i $tmp_yaml
./eval.sh "oa_svg_mppi_wo_nominal" $tmp_yaml $NUM_TRIALS $NUM_STATIC_OBSTACLES $IS_VISUALIZE
echo "[INFO] Finished evaluating svg_mppi without nominal solution"
