import re
import fileinput

data = "leaf 0x7f8ded16a040: 1 key, version c0000000, permutation 0:edcba987654321, parent (nil), prev (nil), next (nil) [ksuf x2]\n  http://1 = SUBTREE #0/137\n    leaf 0x7f8dec568040: 5 keys, version c0000000, permutation 04312:edcba98765, parent (nil), prev (nil), next (nil) [ksuf i2x4]\n      -hydropo = SUBTREE #0/137\n        leaf 0x7f8dec367040: 1 key, version c0000000, permutation 0:edcba987654321, parent (nil), prev (nil), next (nil) [ksuf x2]\n"

#abandoning regular expression
'''
p = re.compile('[a-z]')
m = p.match(data)
print m
'''

#read line by line

totalKey = 0;
totalNode = 0;
for line in fileinput.input() :
    #find ? key
    index = line.find("key")
    if (index > 0) :
        sKey = line[index-2: index]
	print sKey
	nKey = int(sKey)
	totalKey += nKey
	totalNode += 1
print "%d %d" % (totalKey, totalNode)
print totalKey/ 15.0 / totalNode

