#!/bin/bash
echo "Time           RSS(MB)    VSZ(MB)"
echo "--------------------------------"
while true; do
  RSS=$(ps -o rss= -p $1)
  VSZ=$(ps -o vsz= -p $1)
  printf "%s   %.2f      %.2f\n" "$(date +%H:%M:%S)" "$((RSS/1024))" "$((VSZ/1024))"
  sleep 1
done