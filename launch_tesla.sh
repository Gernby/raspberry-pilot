!/bin/bash
cd ~/raspilot
export PYTHONPATH="$PWD"
pkill -f GernbyMode

# Try both versions... the one that matches the hardware will win
python3 ~/raspilot/controls_PICAN.py
python3 ~/raspilot/controls_Panda.py
bin/python3 ~/raspilot/controls_PICAN.py
bin/python3 ~/raspilot/controls_Panda.py
