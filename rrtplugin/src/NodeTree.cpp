#include <iostream>
#include<NodeTree.h>

using namespace std;

NodeTree::NodeTree()
{
    vecNodes={};
}
void NodeTree::addNode(const std::vector<double>& _config,long _self_id,long _parent_id)
{

    vecNodes.push_back(new RRTNode(_config,_self_id,_parent_id));
}
bool NodeTree::deleteNode(long self_id)
{
    // TODO - Not needed in code at the moment
    return false;

}
void NodeTree::printNodetree()
{
    for (ulong i = 0; i < vecNodes.size(); ++i) {

        cout<<vecNodes.at(i)->self_id<<"    ";
        cout<<vecNodes.at(i)->parent_id<<endl;
        cout<<"Config :";
        for (size_t j=0; j< vecNodes[i]->config.size(); j++)
            cout<<" "<<vecNodes[i]->config[j];
        cout<<std::endl;
    }
}
int NodeTree::getNodeSize()
{
    return vecNodes.size();
}

void NodeTree::nearestNeighbor(const vector<double> _config, std::vector<double> &returnConfig, long &returnID)
{
    double dist=0;
    double closest=std::numeric_limits<float>::max(); // max value of float
    for (size_t i = 0; i < vecNodes.size(); ++i) {
        dist=vecNodes.at(i)->calcDistance(_config);
        if(dist<closest)
        {
            closest=dist;
            returnConfig=vecNodes.at(i)->config;
            returnID=vecNodes.at(i)->self_id;
        }
    }

    return;
}


