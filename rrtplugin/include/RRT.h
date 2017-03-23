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


    RRT(vector<double> _startConfig, vector<double> _goalConfig, EnvironmentBasePtr _env)
    {
        startConfig=_startConfig;
        goalConfig=_goalConfig;
        env=_env;

        // Get robot from environment
        env->GetRobots(n);
        robot=n.at(0); //selecting the first robot

        // Unqiue Joint ID for each joint
        jindex=robot->GetActiveDOFIndices();
        for (size_t i = 0; i < jindex.size(); ++i) {
            if(robot->GetJointFromDOFIndex(jindex.at(i))->IsCircular(0))
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
                qmin.at(i)=-0.1; qmax.at(i)=0.1;
            }
        }/*
        for (uint i = 0; i < jindex.size(); ++i) {
            cout<<qmin.at(i)<<"     "<<qmax.at(i)<<endl;
        }*/
        // cout<<"Constructor Initialized"<<endl;
    }
    void sampleRandomConfig(vector<double> &config, float goalbias=0.25 )
    {
        random=std::uniform_real_distribution<double>(0.0,1.0);
        std::mt19937_64 init_generator(rand_dev());

        // Accounting for goal bias
        if(random(init_generator)<goalbias)
        {
            if(random(init_generator)<0.5)
            {
                for (size_t i = 0; i < jindex.size(); ++i) {
                    // First joint
                    random=std::uniform_real_distribution<double>(goalConfig.at(i)-0.1,goalConfig.at(i));
                    std::mt19937_64 init_generator(rand_dev());
                    config.push_back(random(init_generator));

                }
              //  cout<<"-* * * *-CLOSE GOAL POSITION TAKEN -*  * *  * *- "<<endl;
            }
            else
            {config=goalConfig;

             //   cout<<"- - - - - - - -GOAL POSITION TAKEN - - - - - - - "<<endl;
            }
            return ;
        }
        // Generating sample within joint limits
        for (size_t i = 0; i < jindex.size(); ++i) {
            // First joint
            random=std::uniform_real_distribution<double>(qmin.at(i),qmax.at(i));
            std::mt19937_64 init_generator(rand_dev());
            config.push_back(random(init_generator));
        }
        // cout<<"Random Sample Gen"<<endl;
        return ;
    }
    bool isCollision(const vector<dReal> &config)  // change this function. use collision checker online example :: IMPORTANT
    {
        OpenRAVE::EnvironmentMutex::scoped_lock lock(env->GetMutex());

        robot->SetActiveDOFValues(config);
        return (robot->CheckSelfCollision() || env->CheckCollision(RobotBaseConstPtr(robot)));
        /*
        static OpenRAVE::CollisionCheckerBasePtr pChecker;
        static std::vector<OpenRAVE::KinBodyPtr> vBodies;
        std::vector<OpenRAVE::dReal> values;

        OpenRAVE::EnvironmentMutex::scoped_lock lock(env->GetMutex());

        for (size_t i = 0; i < config.size(); i++)
        {  values.push_back(config.at(i));}

        robot->SetActiveDOFValues(values);
        if (vBodies.empty())
            env->GetBodies(vBodies);

        pChecker = env->GetCollisionChecker();
        if (!pChecker->SetCollisionOptions(OpenRAVE::CO_Contacts))
        {
            pChecker = OpenRAVE::RaveCreateCollisionChecker(env, "ode");
            pChecker->SetCollisionOptions(OpenRAVE::CO_Contacts);
            env->SetCollisionChecker(pChecker);
        }

        if (env->CheckStandaloneSelfCollision(robot))
        {
            for (size_t i = 0; i < vBodies.size(); i++)
                if (vBodies[i] != robot)
                    if (env->CheckCollision(robot, vBodies[i]))
                        return true;

            return false;
        }
         return true;*/
    }
    bool buildRRT(NodeTree &NTree);
    short extend(NodeTree &NTree, const vector<double> configRand);
    void executeTraj(vector<vector<double>> &trajConfig);
    void findPath(NodeTree &NTree,vector<vector<double>> &trajConfig);
    void smoothPath(vector<vector<double>> &trajConfig);
    bool connectPath(vector<vector<double>> &trajConfig,int r1,int r2);
};


