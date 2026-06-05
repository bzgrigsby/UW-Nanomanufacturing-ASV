#!/bin/bash

cd /home/admin
source env/bin/activate
cd "/home/admin/GroundStation/Web Project"
python -m flask --app app run --host=0.0.0.0
