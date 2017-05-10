#!/bin/bash
rsync -chavzP --stats --delete alarm@192.168.3.1:/home/alarm/logs/ ../logs