#ifndef RRT_NODE
#define RRT_NODE

#include<iostream>
#include <vector>

class RRTNode
{
public:

    typedef std::vector<float> configv;
    typedef std::vector<configv> Tree;
    configv config;
    int self_id;
    int parent_id;
    int dimension;

    RRTNode(configv _config, int _self_id, int _parent_id)
    {
        config=_config;
        self_id=_self_id;
        parent_id=_parent_id;

    }
    void setConfig(configv _config)
    {
        _config=_config;
    }
    configv getconfig()
    {
        return config;
    }
    double getJoint(int jindex)
    {
        return config[jindex];
    }
    void setSelfID(int _self_id)
    {
        self_id=_self_id;
    }
    void setParentID(int _parent_id)
    {
        parent_id=_parent_id;
    }
    int getSelfID()
    {
        return self_id;
    }
    int getParentID()
    {
        return parent_id;
    }

};





#endif
