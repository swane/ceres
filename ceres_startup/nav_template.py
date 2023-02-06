# recieves mission as a  list of lists on argv[1], and a starting point on argv[2].
# to test from a terminal:
# python nav.py '[["-3.0953963", "52.4604236", "start"], ["-1.9692977", "53.0455683", "middle"], ["-1.0519393", "52.5941015", "end"]]' 0

import sys
import json

try:
    mission = sys.argv[1]
    #print(mission)
    start_point = sys.argv[2]
    #print (start_point)
except: # default values for testing. WARNING: Thonny uses Python 3!!!!!!!!!
    mission = '[["-3.0953963", "52.4604236", "start"], ["-1.9692977", "53.0455683", "middle"], ["-1.0519393", "52.5941015", "end"]]'
    start_point = '0'
    print('no args given')


# if you want to use Python dictionry

m_dict = json.loads(mission)


for i in range(int(start_point), len(m_dict)):

    print(m_dict[i][1]) # lat
    print(m_dict[i][0]) # lon
    print(m_dict[i][2]) # info

print()
print("The route has " + str(len(m_dict)) + " points")


sys.stderr.write(str(mission) + ' ' + start_point)