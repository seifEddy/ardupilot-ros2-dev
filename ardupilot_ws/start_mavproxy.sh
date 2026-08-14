# Start mavproxy in the background
set -ex

# Start mavproxy with the master set to mcast: and the console output gui
mavproxy.py --master mcast: --console