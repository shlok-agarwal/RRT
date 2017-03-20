#pragma once

#include<iostream>
#include <vector>
#include<RRTNode.h>
using namespace std;

class NodeTree
{
public:

    std::vector<RRTNode> Nodes;
    void addNode(std::vector<float> _config,int _self_id,int _parent_id)
    {
        Nodes.push_back(RRTNode(_config,_self_id,_parent_id));
    }
    bool deleteNode(int self_id)
    {
        // TODO - Not needed in code at the moment
        return false;

    }
    void printNodetree()
    {
        for (uint i = 0; i < Nodes.size(); ++i) {

            cout<<Nodes.at(i).self_id<<"    ";
            cout<<Nodes.at(i).parent_id<<endl;
        }
    }

};
