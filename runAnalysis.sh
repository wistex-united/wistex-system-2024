#!/bin/bash

# Update these before running
BadgerRLSystemPath="/Users/jkelle/BadgerRLSystem"
AbstractSimPath="/Users/jkelle/AbstractSim"

set -e

if [[ "$(uname)" == "Darwin" ]]; then
  echo "Running on macOS"

  # Policy names
  policy_names=(
    # "KEEP_SimRobot_13_06_55_11_policy"
    "2M_noshaping_SimRobot_15_22_44_12_curriculum_close"
  )

  for policy_name in "${policy_names[@]}"; do
    echo "Running for policy: $policy_name"

    current_dir=$(pwd)
    cd $AbstractSimPath

    # Change this before running (soccer vs staticDef)
    python ./abstractSim/export.py policies/$policy_name
    cp exported/policy.onnx ${BadgerRLSystemPath}/Config/Policies/NearOppGoal/

    cd $current_dir
    # Number of iterations
    iterations=1
    parallel=1 # Mac seems to only work with parallel=1?
    # Application names
    application_names=(
      "NearOppGoal000"
      "NearOppGoal001"
      "NearOppGoal002"
      "NearOppGoal003"
      "NearOppGoal004"
      "NearOppGoal005"
      "NearOppGoal006"
      "NearOppGoal007"
      "NearOppGoal008"
      "NearOppGoal009"
      "NearOppGoal010"
      "NearOppGoal011"
      "NearOppGoal012"
      "NearOppGoal013"
      "NearOppGoal014"
      "NearOppGoal015"
      "NearOppGoal016"
      "NearOppGoal017"
      "NearOppGoal018"
      "NearOppGoal019"
    )

    mkdir -p analysisData

    for application_name in "${application_names[@]}"; do
      echo "Running for application: $application_name"

      rm -f ./Config/Scenes/tempdata.txt

      # Create an empty data.txt file (or overwrite an existing one)
      echo "" > analysisData/${application_name}_data.txt

      for (( i=1; i<=$iterations/$parallel; i++ )); do

        for (( j=1; j<=$parallel; j++ )); do
        (
          open -g ./Config/Scenes/$application_name.ros2 &> /dev/null &
        )
        done

        # Wait for tempdata.txt to have more than parallel lines
        while [[ $(wc -l < ./Config/Scenes/tempdata.txt 2>/dev/null) -lt $parallel ]]; do
          sleep 1  # Wait for 1 second before checking again
        done > /dev/null 2>&1

        # Append info from tempdata.txt to data.txt
        cat ./Config/Scenes/tempdata.txt >> analysisData/${application_name}_data.txt
        # Delete tempdata.txt
        rm ./Config/Scenes/tempdata.txt

        # Print a message for clarity
        echo "Iteration $i completed."
        done

      echo "All iterations for $application_name finished."
      done

      # Rename the data files to include the policy name
      for application_name in "${application_names[@]}"; do
        mv analysisData/${application_name}_data.txt analysisData/${application_name}_${policy_name}_data.txt
      done

    echo "All applications for $policy_name finished."
    done

elif [[ "$(uname)" == "Linux" ]]; then
  echo "Running on Linux"

  # Policy names
  policy_names=("displacements.large_dis_policy" "displacements.small_dis_policy" "noise.no_ball_noise_policy" "noise.observation_noise_policy" "realismFailure.goalSIze_policy" "realismFailure.kickingTime_policy" "shape.small_agents_policy")

  for policy_name in "${policy_names[@]}"; do
    echo "Running for policy: $policy_name"

    current_dir=$(pwd)
    cd $AbstractSimPath/multi_agent/curriculum

    # Change this before running (soccer vs staticDef)
    python ./utils/export_model.py policies/defender_ablations.$policy_name
    cp ./utils/exported_model/policy.onnx ${path_prefix}/Config/Policies/staticDef/

    cd $current_dir
    # Number of iterations
    iterations=100
    parallel=5
    # Application names
    application_names=("analysis_staticDef1" "analysis_staticDef2" "analysis_staticDef3")

    for application_name in "${application_names[@]}"; do
      echo "Running for application: $application_name"

      rm ./Config/Scenes/tempdata.txt

      # Create an empty data.txt file (or overwrite an existing one)
      echo "" > analysisData/${application_name}_data.txt

      for (( i=1; i<=$iterations/$parallel; i++ )); do

        for (( j=1; j<=$parallel; j++ )); do
        (
          ./Build/Linux/SimRobot/Develop/SimRobot Config/Scenes/$application_name.ros2 &> /dev/null &
        )
        done

        # Wait for tempdata.txt to have more than parallel lines
        while [[ $(wc -l < ./Config/Scenes/tempdata.txt 2>/dev/null) -lt $parallel ]]; do
          sleep 1  # Wait for 1 second before checking again
        done > /dev/null 2>&1

        # Append info from tempdata.txt to data.txt
        cat ./Config/Scenes/tempdata.txt >> analysisData/${application_name}_data.txt
        # Delete tempdata.txt
        rm ./Config/Scenes/tempdata.txt

        # Print a message for clarity
        echo "Iteration $i completed."
        done

      echo "All iterations for $application_name finished."
      done

      # Rename the data files to include the policy name
      for application_name in "${application_names[@]}"; do
        mv analysisData/${application_name}_data.txt analysisData/${application_name}_${policy_name}_data.txt
      done

    echo "All applications for $policy_name finished."
    done

else
  echo "Unknown operating system"
fi
