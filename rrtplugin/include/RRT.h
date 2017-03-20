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
    std::uniform_real_distribution<double> random;
    std::random_device rand_dev;


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
        for (uint i = 0; i < jindex.size(); ++i) {
            cout<<qmin.at(i)<<"     "<<qmax.at(i)<<endl;
        }
    }
    vector<float> sampleRandomConfig()
    {
        vector<float> config;
        for (uint i = 0; i < jindex.size(); ++i) {
            // First joint
            random=std::uniform_real_distribution<double>(qmin.at(i),qmax.at(i));
            std::mt19937_64 init_generator(rand_dev());
            config.push_back(random(init_generator));
        }
        for (uint i = 0; i < jindex.size(); ++i) {
            cout<<config.at(i)<<endl;
        }
        return config;
    }
    bool isCollision(vector<double> config)
    {
         robot->SetActiveDOFValues(config);
         if(robot->CheckSelfCollision())
         {
             return true;
         }
         else if(env->CheckCollision(RobotBaseConstPtr(robot)))
         {
             return true;
         }
         else return false;
    }


};
