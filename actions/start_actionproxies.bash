#!/bin/bash

python wait_actionproxy.py &
python turn_actionproxy.py &
python movebase_actionproxy.py &

