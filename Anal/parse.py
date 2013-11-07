import re
import fileinput
#read line by line

totalKey = 0;
totalNode = 0;
for line in fileinput.input() :
    #find ? key
    index = line.find(" key,")
    if (index > 0) :
        sKey = line[index-1: index]
        print sKey
        nKey = int(sKey)
        totalKey += nKey
        totalNode += 1

    index = line.find(" keys,")
    if (index > 0) :
        sKey = line[index-2: index]
        print sKey
        nKey = int(sKey)
        totalKey += nKey
        totalNode += 1

print "%d %d" % (totalKey, totalNode)
print totalKey/ 15.0 / totalNode
