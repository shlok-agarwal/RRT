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

    NodeTree()
    {
        vecNodes={}; // do i need this?
        cout<<"object created"<<endl;
    }

    void addNode(std::vector<double>& _config,long _self_id,long _parent_id)
    {

        vecNodes.push_back(new RRTNode(_config,_self_id,_parent_id));
    }
    bool deleteNode(long self_id)
    {
        // TODO - Not needed in code at the moment
        return false;

    }
    void printNodetree()
    {
        for (ulong i = 0; i < vecNodes.size(); ++i) {

            cout<<vecNodes.at(i)->self_id<<"    ";
            cout<<vecNodes.at(i)->parent_id<<endl;
        }
    }
    int getNodeSize()
    {
        return vecNodes.size();
    }

    pair<long,vector<double>> nearestNeighbor(std::vector<double> _config)
    {
                              std::vector<double> nearconfig;
                              long ID=0;
                              double dist=0;
                              double closest=std::numeric_limits<double>::max(); // max value of double
                              for (ulong i = 0; i < vecNodes.size(); ++i) {
        dist=vecNodes.at(i)->calcDistance(_config);
        if(dist<closest)
        {
            closest=dist;
            nearconfig=vecNodes.at(i)->config;
            ID=vecNodes.at(i)->self_id;
        }
    }

    return make_pair(ID,nearconfig);
}

};
