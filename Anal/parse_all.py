# argv[1]: masstree printout to be parsed
# argv[2]: workload file

import re
import sys

class node:

    def __init__ (self) :
        self.nodetype = 0 # 0 is leaf, 1 is interior
        self.numKeys = 0
        self.mLevel = 0
        self.bLevel = 0
        self.keyValue = {}
        self.keylen = 0
        self.isComplete = False

    def displayNode (self) :
        print "nodetype: ", self.nodetype, ", numKeys: ", self.numKeys, ", mLevel: ", self.mLevel, ", bLevel: ", self.bLevel, ", keyValue: ", self.keyValue, ", keylen: ", self.keylen, ", isComplete: ", self.isComplete


node_list = []
node_stack = []

f_tree = open (sys.argv[1])

for line in f_tree.readlines() :
    tokens = line.split ()
    #print tokens
    if (tokens[0] == "set") :
        break
    if len (tokens) == 0 :
        continue
    #leading_space = len(line) - len(line.lstrip(' '))
    new_node = node()

    if (tokens[0] == "internode") or (tokens[0] == "leaf") :
        if (tokens[0] == "internode") :
            new_node.nodetype = 1

        new_node.numKeys = int (tokens[2])
        if node_stack :
            parent = node_stack.pop()
            if (parent.nodetype == 1) :
                new_node.mLevel = parent.mLevel
                new_node.bLevel = parent.bLevel + 1
            if (parent.nodetype == 0) :
                new_node.mLevel = parent.mLevel + 1
                new_node.bLevel = 0
            node_stack.append (parent)
        node_stack.append (new_node)
    elif (len(tokens) == 1) :
        cur_node = node_stack.pop()
        while (cur_node.keylen == cur_node.numKeys) :
            cur_node.isComplete = True
            node_list.append (cur_node)
            cur_node = node_stack.pop()
        if (cur_node.nodetype == 1) :
            cur_node.keyValue[tokens[0]] = "node"
        cur_node.keylen += 1
        node_stack.append (cur_node)
    else :
        if (tokens[2] == "[]") :
            continue
        cur_node = node_stack.pop()
        while (cur_node.keylen == cur_node.numKeys) :
            cur_node.isComplete = True
            node_list.append (cur_node)
            cur_node = node_stack.pop()
        if (cur_node.nodetype == 0) :
            cur_node.keyValue[tokens[0]] = tokens[2]
            cur_node.keylen += 1
            if (cur_node.keylen == cur_node.numKeys) and (tokens[2] != "SUBTREE") :
                cur_node.isComplete = True
                node_list.append (cur_node)
            else :
                node_stack.append (cur_node)
        else :
            cur_node.displayNode()
            print "error"
            exit(1)

while (node_stack) :
    node_list.append (node_stack.pop())

for n in node_list :
    n.displayNode()

INTERNODE_FIXED_SIZE = 261
BORDERNODE_FIXED_SIZE = 292
POINTER_SIZE = 8
VALUE_SIZE = 8

totalNode = 0
totalKeysliceLen = 0
totalKeyslice = 0
totalmLevel = 0
totalbLevel = 0
maxmLevel = 0
maxbLevel = 0
leafLinks = 0
leafValues = 0
totalSpace = 0
suffixSize = 0

for n in node_list :
    keys = n.keyValue.keys()
    for k in keys :
        totalKeysliceLen += len(k)
    totalKeyslice += n.numKeys
    totalmLevel += n.mLevel
    totalbLevel += n.bLevel
    totalNode += 1
    if (maxmLevel < n.mLevel) :
        maxmLevel = n.mLevel
    if (maxbLevel < n.bLevel) :
        maxbLevel = n.bLevel
    if (n.nodetype == 1) :
        totalSpace += INTERNODE_FIXED_SIZE
    elif (n.nodetype == 0) :
        totalSpace += BORDERNODE_FIXED_SIZE
        values = n.keyValue.values()
        for v in values :
           if (v == "SUBTREE") :
               leafLinks += POINTER_SIZE
           else :
               leafValues += VALUE_SIZE
               totalSpace += VALUE_SIZE
    else :
        exit(1)

avgmLevel = totalmLevel * 1.0 / totalNode
avgbLevel = totalbLevel * 1.0 / totalNode
avgDepth = avgmLevel + avgbLevel
maxDepth = maxmLevel + maxbLevel

totalKeyLen = 0
f_workload = open (sys.argv[2])
for line in f_workload.readlines() :
    tokens = line.split()
    totalKeyLen += len (tokens[1])

keyslice_array_usage = totalKeyslice / 15.0 / totalNode
key_compress = totalKeysliceLen / 1.0 / totalKeyLen

suffixSize = totalKeysliceLen - totalKeyslice * 8
totalSpace += suffixSize

struct_overhead = (totalSpace - totalKeysliceLen - leafValues) / 1.0 / totalSpace

print "\nkeyslice_array_usage = ", keyslice_array_usage, "\nkey_compress = ", key_compress, "\navgmLevel = ", avgmLevel, "\navgbLevel = ", avgbLevel, "\navgDepth = ", avgDepth, "\nmaxmLevel = ", maxmLevel, "\nmaxbLevel = ", maxbLevel, "\nmaxDepth = ", maxDepth, "\ntotalKeysliceLen: ", totalKeysliceLen, "\ntotalKeySize: ", totalKeyLen, "\ntotalValueSize: ", leafValues, "\ntotalSpace: ", totalSpace, "\nstruct_overhead: ", struct_overhead, "\ntotalNode = ", totalNode, "\nsuffixSize = ", suffixSize


