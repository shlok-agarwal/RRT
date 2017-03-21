#pragma once

#include<iostream>
#include <vector>
#include<RRTNode.h>
#include <limits>
using namespace std;

class NodeTree
{
public:

    NodeTree()
    {
        cout<<"object created"<<endl;
    }
    std::vector<RRTNode> Nodes;
    void addNode(std::vector<double> _config,long _self_id,long _parent_id)
    {

        Nodes.push_back(RRTNode(_config,_self_id,_parent_id));
    }
    bool deleteNode(long self_id)
    {
        // TODO - Not needed in code at the moment
        return false;

    }
    void printNodetree()
    {
        for (ulong i = 0; i < Nodes.size(); ++i) {

            cout<<Nodes.at(i).self_id<<"    ";
            cout<<Nodes.at(i).parent_id<<endl;
        }
    }
    int getNodeSize()
    {
        return Nodes.size();
    }

    pair<long,vector<double>> nearestNeighbor(std::vector<double> _config)
    {
                              std::vector<double> nearconfig;
                              long ID=0;
                              double dist=0;
                              double closest=std::numeric_limits<double>::max(); // max value of double
                              for (ulong i = 0; i < Nodes.size(); ++i) {
        dist=Nodes.at(i).calcDistance(_config);
        if(dist<closest)
        {
            closest=dist;
            nearconfig=Nodes.at(i).config;
            ID=Nodes.at(i).self_id;
        }
    }

    return make_pair(ID,nearconfig);
}

};
