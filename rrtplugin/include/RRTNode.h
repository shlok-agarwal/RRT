#pragma once

#include<iostream>
#include <vector>
#include <math.h>

class RRTNode
{
public:

    typedef std::vector<float> configv;
    typedef std::vector<configv> Tree;
    configv config;
    configv weights;
    int self_id;
    int parent_id;
    int dimension;

    RRTNode(configv _config, int _self_id, int _parent_id, configv weights={3.17104, 2.75674, 2.2325,1.78948,1.42903,0.809013,0.593084})
    {
        config=_config;
        self_id=_self_id;
        parent_id=_parent_id;

    }
    // hard coded weight values because of discrepancy of results obtained c++ and python API of openrave
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
    float calcDistance(configv sample)
    {
        float dist=0;
        for (uint i = 0; i < config.size(); ++i) {
            dist+=weights.at(i)*pow(config.at(i)-sample.at(i),2);
        }
        return sqrt(dist);
    }

};

