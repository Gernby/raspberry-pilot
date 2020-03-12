#!/bin/bash
cd ~/raspilot
~/.local/bin/pipenv run bash model.sh &
~/.local/bin/pipenv run bash controls.sh &
