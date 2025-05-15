#!/bin/bash
caffeinate -d & #MAC
CAFFEINATE_PID=$! #MAC

# Number of iterations
iterations=2
# Application sname
# application_name="analysis_staticDef1"
application_name="josh"

# Get first command line argument (application name)
if [[ $1 != "" ]]; then
  application_name=$2
fi

# Get second command line argument (default = 100)
if [[ $2 != "" ]]; then
  iterations=$1
fi

rm -f ./Config/Scenes/tempdata.txt

# Create an empty data.txt file (or overwrite an existing one)
mkdir -p analysisData
echo "" > analysisData/data.txt

for (( i=1; i<=$iterations; i++ )); do
  # Open the application
  open -g ./Config/Scenes/$application_name.ros2 #MAC
  # open -g ./Config/Scenes/BHFast.ros2 #MAC
  app_pid=$!  # Store the process ID of the opened application
  echo "app_pid: $app_pid"

  # Wait for tempInfo.txt to be created
  while [[ ! -f ./Config/Scenes/tempdata.txt ]]; do
    echo "Waiting for tempdata.txt to be created..."
    sleep 1  # Wait for 1 second before checking again
  done

  sleep 1

  # Close the application (assuming it responds to SIGINT)
#   kill -SIGINT $app_pid
  echo "stopping SimRobot"
  osascript -e 'quit app "SimRobot"' #MAC

  # Append info from tempInfo.txt to data.txt
  echo "Writing to analysisData/data.txt"
  cat ./Config/Scenes/tempdata.txt >> analysisData/data.txt

  # Delete tempInfo.txt
  echo "Deleting tempdata.txt"
  rm ./Config/Scenes/tempdata.txt

  # Print a message for clarity
  echo "Iteration $i completed."

  sleep 1
done

pkill caffeinate #MAC

echo "All iterations finished."