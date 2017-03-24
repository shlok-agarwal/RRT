#pragma once

#include<iostream>
#include <vector>
#include<NodeTree.h>
#include <openrave/openrave.h>
#include <openrave/planningutils.h>
#include <chrono>

using namespace std;
using namespace OpenRAVE;
using  ns = chrono::seconds;
using get_time = chrono::steady_clock ;

class RRT
{
public:
    vector<double> startConfig;
    vector<double> goalConfig;
    EnvironmentBasePtr env;
    RobotBasePtr robot;
    std::vector<RobotBasePtr> n; // gettting all the robots in the environment

    std::vector<dReal> qmin, qmax;
    std::vector<int> jindex,jcircular;
    std::uniform_real_distribution<double> random;
    std::random_device rand_dev;
    float goalBias;


    RRT(vector<double> _startConfig, vector<double> _goalConfig,float _goalBias, EnvironmentBasePtr _env);
    void sampleRandomConfig(vector<double> &config, float goalBias);
    bool isCollision(const vector<dReal> &config) ;
    pair<vector<vector<double> >, vector<vector<double> > > buildRRT(NodeTree &NTree);
    void buildBiRRT(NodeTree &NTree1,NodeTree &NTree2);
    short extendRRT(NodeTree &NTree, const vector<double> &configRand,bool mode=true);
    void executeTraj(vector<vector<double>> &trajConfig);
    void findPath(NodeTree &NTree, vector<vector<double>> &trajConfig, bool reverseTree=true);
    void smoothPath(vector<vector<double>> &trajConfig);
    bool connectPath(vector<vector<double>> &trajConfig,int r1,int r2);
};


