#!/bin/bash

mv build/results.csv $(printf "results_%s_%s.csv" $(date +%H_%M_%S__%d%b%Y) $1)
