#!/usr/bin/env bash

. ~/figment/install/setup.bash

. ~/figment/devel/setup.bash

# Run the example node
echo "Launching ARIAC example nodes for example_team2"
rosrun figment_ariac scheduler_plan
