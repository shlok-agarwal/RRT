#pragma once

#include<iostream>
#include <vector>
#include<NodeTree.h>
#include <openrave/openrave.h>

using namespace std;
using namespace OpenRAVE;

class RRT
{
public:
    vector<float> startConfig;
    vector<float> goalConfig;
    EnvironmentBasePtr env;
    RobotBasePtr robot;
    std::vector<RobotBasePtr> n; // gettting all the robots in the environment

    std::vector<dReal> qmin, qmax;
    std::vector<int> jindex,jcircular;


    RRT(vector<float> _startConfig, vector<float> _goalConfig, EnvironmentBasePtr _env)
    {
        startConfig=_startConfig;
        goalConfig=_goalConfig;
        env=_env;

        // Get robot from environment
        env->GetRobots(n);
        robot=n.at(0); //selecting the first robot

        // Unqiue Joint ID for each joint
        jindex=robot->GetActiveDOFIndices();
        for (uint i = 0; i < jindex.size(); ++i) {
            if(robot->GetJointFromDOFIndex(jindex.at(i))->IsCircular(0)==true)
            {
                jcircular.push_back(1);
            }
            else jcircular.push_back(0);
        }

        // Calculating joint limits for non-circular joints
        robot->GetDOFLimits(qmin, qmax, jindex);

        for (uint i = 0; i < jindex.size(); ++i) {
            if(jcircular.at(i)==1)
            {
                qmin.at(i)=-2*3.14; qmax.at(i)=2*3.14;
            }
        }
        for (int i = 0; i < 7; ++i) {
           cout<<qmin.at(i)<<"    "<<qmax.at(i)<<endl;
        }
    }


};
