#!/bin/bash
cd raspilot
~/.local/bin/pipenv run bash controls.sh
~/.local/bin/pipenv run bash model.sh
cd