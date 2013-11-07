import re
import fileinput
#read line by line

totalKey = 0
totalNode = 0
urlsTotalLength = 0

f_url_input = open("../hyw_url_init.dat")
f_tree_output = open("../output.txt")

for line in f_url_input.readlines():
   tokens = line.split(' ')
   urlsTotalLength  += len(tokens[1])

for line in f_tree_output:
    #find ? key
    index = line.find(" key,")
    if (index > 0) :
        sKey = line[index-1: index]
        #print sKey
        nKey = int(sKey)
        totalKey += nKey
        totalNode += 1

    index = line.find(" keys,")
    if (index > 0) :
        sKey = line[index-2: index]
        #print sKey
        nKey = int(sKey)
        totalKey += nKey
        totalNode += 1

print "number of key slices: %d, number of nodes: %d" % (totalKey, totalNode)
print "memory usage: %f" % (totalKey/ 15.0 / totalNode)

print "key compression: %f" % (float(totalKey * 8) / urlsTotalLength)

f_url_input.close()
f_tree_output.close()


