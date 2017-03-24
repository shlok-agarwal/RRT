#pragma once

#include<iostream>
#include <vector>
#include <math.h>

using namespace std;

class RRTNode
{
public:

    typedef std::vector<double> configv;
    typedef std::vector<configv> Tree;
    configv config;
    configv weights;
    long self_id;
    long parent_id;
    long dimension;

    RRTNode(configv _config, int _self_id, int _parent_id);
    // hard coded weight values because of discrepancy of results obtained c++ and python API of openrave
    void setConfig(configv _config);
    configv getconfig();
    double getJoint(int jindex);
    void setSelfID(int _self_id);
    void setParentID(int _parent_id);
    int getSelfID();
    int getParentID();
    double calcDistance(configv sample);

};

