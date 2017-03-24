#pragma once

#include<iostream>
#include <vector>
#include<RRTNode.h>
#include <limits>
using namespace std;

class NodeTree
{
public:
    std::vector<RRTNode *> vecNodes;
    NodeTree();
    void addNode(const std::vector<double>& _config,long _self_id,long _parent_id);
    bool deleteNode(long self_id);
    void printNodetree();
    int getNodeSize();
    void nearestNeighbor(const vector<double> _config, std::vector<double> &returnConfig, long &returnID);
};
