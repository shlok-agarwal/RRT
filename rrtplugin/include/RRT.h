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
                qmin.at(i)=-3.14; qmax.at(i)=3.14;
            }
        }
        /*for (uint i = 0; i < jindex.size(); ++i) {
            cout<<qmin.at(i)<<"     "<<qmax.at(i)<<endl;
        }*/
        cout<<"Constructor Initialized"<<endl;
    }
    vector<double> sampleRandomConfig()
    {
        vector<double> config;
        for (uint i = 0; i < jindex.size(); ++i) {
            // First joint
            random=std::uniform_real_distribution<double>(qmin.at(i),qmax.at(i));
            std::mt19937_64 init_generator(rand_dev());
            config.push_back(random(init_generator));
        }
        /*
        for (uint i = 0; i < jindex.size(); ++i) {
            cout<<config.at(i)<<endl;
        }*/
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
    bool buildRRT(NodeTree Nodes) // change return type later
    {
        long maxiterations=5;
        vector<double> config;
        Nodes.addNode(startConfig,1,0);
        Nodes.printNodetree();
        int state=0;
        for (long i = 0; i < maxiterations; ++i) {
            config=sampleRandomConfig();
            state=extend(Nodes,config); // maybe change this to pointer
            cout<<"iteration"<<endl;
            if(state==1)
            {
                break;
            }

        }
        if(state==1)
        {
            cout<<"Success!!";
            return true;
        }
        else return false;
    }
    int extend(NodeTree Nodes,vector<double> configrand)
    {

        cout<<"extend entered"<<endl;
        Nodes.printNodetree();
        vector<double> confignear;
        long tempID;
        confignear=Nodes.nearestNeighbor(configrand).second;
        tempID=Nodes.nearestNeighbor(configrand).first;
        vector<double> configvector;
        for (uint i = 0; i < confignear.size(); ++i) {
            configvector.push_back(configrand[i]-confignear[i]);
        }

        double uvector=0;
        // find unit vector
        for (uint i = 0; i < configvector.size(); ++i) {
            uvector+=pow(configvector.at(i),2);

        }
        uvector=int(sqrt(uvector));
        for (uint i = 0; i < configvector.size(); ++i) {
            configvector[i]=configvector[i]/uvector;

        }
        // uvector - number of steps
        // configvector - step increments

        for (int i = 0; i < uvector; ++i) {

            vector<double> confignew;
            uint k=0;
            for (uint j = 0; j < configvector.size(); ++j) {
                confignew.push_back(confignear[j]+configvector[j]);
                if(confignew[j]==goalConfig[j]) {k++;}
            }
            if(k==goalConfig.size())
            {
                cout<<"Goal Reached"<<endl;
                return 1;

            }
            else if(isCollision(confignew))
            {
                cout<<"collision"<<endl;
                return 0;
            }
            else
            {
                Nodes.addNode(confignew,Nodes.getNodeSize(),tempID);
                //cout<<"\r"<<"Added node: "<<confignew.at(0)<<"\t"<<confignew.at(1)<<"\t"<<confignew.at(2)<<"\t"<<confignew.at(3)<<"\t"<<confignew.at(4)<<"\t"<<confignew.at(5)<<"\t"<<confignew.at(6)<<"\t"<<Nodes.getNodeSize()<<flush;
                cout<<"Added node: "<<confignew.at(0)<<"\t"<<confignew.at(1)<<"\t"<<confignew.at(2)<<"\t"<<confignew.at(3)<<"\t"<<confignew.at(4)<<"\t"<<confignew.at(5)<<"\t"<<confignew.at(6)<<"\t"<<Nodes.getNodeSize()<<endl;
                Nodes.printNodetree();
                tempID=Nodes.getNodeSize();
            }

        }
        return 0;

    }

};
