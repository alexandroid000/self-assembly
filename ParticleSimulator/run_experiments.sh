#! /usr/bin/sh

# export PYTHONPATH="${PYTHONPATH}":"${PWD}"

for start in {0..4}; do
    for action in {0..81}; do
    	python3 run_sim.py "$start" "$action"
    done
done
