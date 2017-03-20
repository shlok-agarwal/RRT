#pragma once

#include<iostream>
#include <vector>
#include<RRTNode.h>
#include <limits>
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
    int nearestNeighbor(std::vector<float> _config)
    {
        int ID=0;
        float dist=0;
        float closest=std::numeric_limits<float>::max(); // max value of float
        for (uint i = 0; i < Nodes.size(); ++i) {
            dist=Nodes.at(i).calcDistance(_config);
            if(dist<closest)
            {
                closest=dist;
                ID=Nodes.at(i).self_id;
            }
        }
        return ID;
    }

};
