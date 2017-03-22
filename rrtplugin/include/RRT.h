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
                qmin.at(i)=-2*3.14; qmax.at(i)=2*3.14;
            }
        }
        cout<<"index"<<endl;
        for (uint i = 0; i < jindex.size(); ++i) {
            cout<<"     "<<jindex.at(i)<<endl;
        }
        cout<<"circular"<<endl;
        for (uint i = 0; i < jcircular.size(); ++i) {
            cout<<"     "<<jcircular.at(i)<<endl;
        }
        cout<<"ranges"<<endl;
        for (uint i = 0; i < jindex.size(); ++i) {
            cout<<qmin.at(i)<<"     "<<qmax.at(i)<<endl;
        }
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
    bool isCollision(vector<double> config)  // change this function. use collision checker online example :: IMPORTANT
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
    bool buildRRT(NodeTree &NTree) // ask
    {
        long maxiterations=50000;
        vector<double> config;

        NTree.addNode(startConfig,1,0); // ask
        //NTree.printNodetree();
        cout<<"about to print tree \n";
        int state=0;

        for (long i = 0; i < maxiterations; ++i) {
            config=sampleRandomConfig();
            state=extend(NTree,config);  //ask

            //  cout<<"state:   "<<state<<endl;

            //   cout<<"iteration"<<endl;
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
    int extend(NodeTree &NTree,vector<double> configrand)
    {

        // cout<<"extend entered"<<endl;
        //NTree.printNodetree();
        vector<double> confignear;
        long tempID;
        confignear=NTree.nearestNeighbor(configrand).second; // make change here ASK
        tempID=NTree.nearestNeighbor(configrand).first;
        vector<double> configvector;
        for (uint i = 0; i < confignear.size(); ++i) {
            configvector.push_back(configrand[i]-confignear[i]);
        }
        /*          Old Method
        double uvector=0;
        // find unit vector
        for (uint i = 0; i < configvector.size(); ++i) {
            uvector+=pow(configvector.at(i),2);

        }
        uvector=int(sqrt(uvector));
        for (uint i = 0; i < configvector.size(); ++i) {
            configvector[i]=configvector[i]/uvector;
            // cout<<"config vector at :"<<i<<"  = "<<configvector[i]<<"   "<<confignear[i]<<"   "<<configrand[i]<<endl;

        }
        // uvector - number of steps
        // configvector - step increments

        cout<<"closest id   "<<tempID<<endl;


        for (int i = 0; i < uvector; ++i) {

            vector<double> confignew;
            uint k=0;
            for (uint j = 0; j < configvector.size(); ++j) {
                confignew.push_back(confignear[j]+configvector[j]);
                if(confignew[j]==goalConfig[j]) {k++;}
            }


        // New Method

        float resolution=0.1;
        float r;
        vector<vector<double>> s;
        s.resize(confignear.size());
        for (int i = 0; i < confignear.size(); ++i) {
            cout<<"here1"<<endl;
            cout<<int(fabs(configvector[i]/resolution))<<endl;
            if(int(fabs(configvector[i]/resolution))>=1)
            {
            if(configvector[i]>0)
            {
                r=resolution;
            }
            else r=-resolution;
            cout<<"here2"<<endl;
            s.at(i).assign(int(fabs(configvector[i]/resolution)),r);
            s.at(i).assign(7,0.4);
            cout<<"here3"<<endl;
            s.at(i).push_back(fmod(configvector[i],resolution));
            cout<<"here4"<<endl;
            }
            else s.at(i).push_back(configvector[i]);

        }
        // find largest number
*/
        // determine number of times to run each step
        float resolution=0.1;
        vector<int> n;
        for (uint i = 0; i < confignear.size(); ++i) {
            if(fmod(configvector[i],resolution)==0)
                n.push_back(int(fabs(configvector[i]/resolution)));
            else
                n.push_back(int(fabs(configvector[i]/resolution))+1);
        }
        int big = 0;

        for(uint i=0;i<confignear.size();i++)
        {
            if(n.at(i)>big)
                big=n.at(i);
        }
        //  cout << "The biggest number is: " << big << endl;


        vector<int> dir;
        for (uint i = 0; i < confignear.size(); ++i) {
            if(configvector[i]>0)
            {
                dir.push_back(1);
            }
            else
                dir.push_back(-1);
        }
        uint k=0;
        for (int i = 0; i < big; ++i) {
            vector<double> confignew;
            k=0;

            // joint 1 conditions
            if(i<n.at(0)-1)
            {
                confignew.push_back(confignear[0]+dir[0]*resolution);
            }
            else if(i==n.at(0))
            {
                if(fmod(configvector[i],resolution)==0)
                {
                    confignew.push_back(confignear[0]+resolution);
                }
                else confignew.push_back(fmod(configvector[i],resolution));
            }
            else
            {
                confignew.push_back(configrand[0]);
            }


            // joint 2 conditions
            if(i<n.at(1)-1)
            {
                confignew.push_back(confignear[1]+dir[1]*resolution);
            }
            else if(i==n.at(1))
            {
                if(fmod(configvector[i],resolution)==0)
                {
                    confignew.push_back(confignear[1]+resolution);
                }
                else confignew.push_back(fmod(configvector[i],resolution));
            }
            else
            {
                confignew.push_back(configrand[1]);
            }

            // joint 3 conditions
            if(i<n.at(2)-1)
            {
                confignew.push_back(confignear[2]+dir[2]*resolution);
            }
            else if(i==n.at(2))
            {
                if(fmod(configvector[i],resolution)==0)
                {
                    confignew.push_back(confignear[2]+resolution);
                }
                else confignew.push_back(fmod(configvector[i],resolution));
            }
            else
            {
                confignew.push_back(configrand[2]);
            }

            // joint 4 conditions
            if(i<n.at(3)-1)
            {
                confignew.push_back(confignear[3]+dir[3]*resolution);
            }
            else if(i==n.at(3))
            {
                if(fmod(configvector[i],resolution)==0)
                {
                    confignew.push_back(confignear[3]+resolution);
                }
                else confignew.push_back(fmod(configvector[i],resolution));
            }
            else
            {
                confignew.push_back(configrand[3]);
            }

            // joint 5 conditions
            if(i<n.at(4)-1)
            {
                confignew.push_back(confignear[4]+dir[4]*resolution);
            }
            else if(i==n.at(4))
            {
                if(fmod(configvector[i],resolution)==0)
                {
                    confignew.push_back(confignear[4]+resolution);
                }
                else confignew.push_back(fmod(configvector[i],resolution));
            }
            else
            {
                confignew.push_back(configrand[4]);
            }

            // joint 6 conditions
            if(i<n.at(5)-1)
            {
                confignew.push_back(confignear[5]+dir[5]*resolution);
            }
            else if(i==n.at(5))
            {
                if(fmod(configvector[i],resolution)==0)
                {
                    confignew.push_back(confignear[5]+resolution);
                }
                else confignew.push_back(fmod(configvector[i],resolution));
            }
            else
            {
                confignew.push_back(configrand[5]);
            }

            // joint 7 conditions
            if(i<n.at(6)-1)
            {
                confignew.push_back(confignear[6]+dir[6]*resolution);
            }
            else if(i==n.at(6))
            {
                if(fmod(configvector[i],resolution)==0)
                {
                    confignew.push_back(confignear[6]+resolution);
                }
                else confignew.push_back(fmod(configvector[i],resolution));
            }
            else
            {
                confignew.push_back(configrand[6]);
            }

            for (uint j = 0; j < confignear.size(); ++j) {
                if(confignew[i]==goalConfig[i]) k++;
            }

            if(k==goalConfig.size())
            {
                cout<<"Goal Reached"<<endl;
                return 1;

            }
            else if(isCollision(confignew))
            {
                //     cout<<"collision"<<endl;
                return 0;
            }
            else
            {
                NTree.addNode(confignew,NTree.getNodeSize()+1,tempID); // ask
                cout<<"\r"<<"Added node: "<<confignew.at(0)<<"\t"<<confignew.at(1)<<"\t"<<confignew.at(2)<<"\t"<<confignew.at(3)<<"\t"<<confignew.at(4)<<"\t"<<confignew.at(5)<<"\t"<<confignew.at(6)<<"\t"<<NTree.getNodeSize()<<flush;
                //cout<<"Added node: "<<confignew.at(0)<<"\t"<<confignew.at(1)<<"\t"<<confignew.at(2)<<"\t"<<confignew.at(3)<<"\t"<<confignew.at(4)<<"\t"<<confignew.at(5)<<"\t"<<confignew.at(6)<<"\t"<<NTree.getNodeSize()<<endl;
                //NTree.printNodetree(); // ask
                tempID=NTree.getNodeSize();
                confignear=confignew;
            }

        }
        return 0;

    }

};
