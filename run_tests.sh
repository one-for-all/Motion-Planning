#! /bin/bash
cd /Motion-Planning/

# Launch a fake X-server in the background
Xvfb :100 -ac -screen 0 800x600x24 &

# Give that a sec to take effect
sleep 1

# Launch a complete robot context and execute some canned movement.
DISPLAY=:100 python run_tests.py
exit_status=$?
if [ ! $exit_status -eq 0 ]; then
  echo "Error code in running unit tests: " $exit_status
  exit $exit_status
fi