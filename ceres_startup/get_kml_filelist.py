# Matt Butler. Harper Adams University 2022
# Utility to scan for kml files in folder to list in node-red dropdown

import os
import sys
import json

results = []
results += [each for each in os.listdir("/home/pi/Downloads") if each.endswith('.kml')]

results = json.dumps(results)

sys.stderr.write(str(results))


