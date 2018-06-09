#!/bin/bash

rsync -av --delete-after deploy/ crozet@ssh.cluster003.ovh.net:/home/crozet/nphysics_demo/
