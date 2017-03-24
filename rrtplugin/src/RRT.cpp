#include <iostream>
#include<NodeTree.h>
#include<RRT.h>

using namespace std;
RRT::RRT(vector<double> _startConfig, vector<double> _goalConfig, float _goalBias, EnvironmentBasePtr _env)
{
    startConfig=_startConfig;
    goalConfig=_goalConfig;
    goalBias=_goalBias;
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
    }
}
void RRT::sampleRandomConfig(vector<double> &config, float goalBias)
{
    random=std::uniform_real_distribution<double>(0.0,1.0);
    std::mt19937_64 init_generator(rand_dev());

    // Accounting for goal bias
    if(random(init_generator)<goalBias)
    {
        if(random(init_generator)<0.5)
        {
            for (size_t i = 0; i < jindex.size(); ++i) {
                // First joint
                random=std::uniform_real_distribution<double>(goalConfig.at(i)-0.1,goalConfig.at(i));
                std::mt19937_64 init_generator(rand_dev());
                config.push_back(random(init_generator));

            }
        }
        else
        {config=goalConfig;
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
bool RRT::isCollision(const vector<dReal> &config)  // change this function. use collision checker online example :: IMPORTANT
{
    OpenRAVE::EnvironmentMutex::scoped_lock lock(env->GetMutex());

    robot->SetActiveDOFValues(config);
    return (robot->CheckSelfCollision() || env->CheckCollision(RobotBaseConstPtr(robot)));
}
pair<vector<vector<double>>,vector<vector<double>>> RRT::buildRRT(NodeTree &NTree)
{
                                                  // Maximum number of iterations. This could also be a parameter of time
                                                  long maxiterations=100000;
                                                  vector<double> config;
                                                  vector <vector<double>> trajConfig,trajConfigRaw,trajConfigSmooth;
                                                  trajConfigRaw={};
                                                  trajConfigSmooth={};

                                                  NTree.addNode(startConfig,1,0);
                                                  short state=0;
                                                  auto start = get_time::now();
                                                  for (long i = 0; i < maxiterations; ++i) {
    config.clear();
    sampleRandomConfig(config,goalBias);
    state=extendRRT(NTree,config);
    if(state)
    {
        break;
    }
}
if(state)
{
    auto end = get_time::now();
    auto diff = end - start;
    cout<<"Success!!    Time taken to find Goal:   "<<chrono::duration_cast<ns>(diff).count()<<" seconds"<<endl;
    cout<<"Number of Nodes explored:    "<<NTree.getNodeSize()<<endl;

    findPath(NTree,trajConfig);
    trajConfigRaw=trajConfig;
    cout<<"Length of Path:  "<<trajConfigRaw.size()<<endl;
    start = get_time::now();
    smoothPath(trajConfig);
    trajConfigSmooth=trajConfig;
    end = get_time::now();
    diff = end - start;
    cout<<"Length of Path after Smoothening:    "<<trajConfigSmooth.size()<<endl;
    cout<<"Time taken to smoothen the trajectory:   "<<chrono::duration_cast<ns>(diff).count()<<" seconds"<<endl;
    executeTraj(trajConfig);
    return make_pair(trajConfigRaw,trajConfigSmooth);
}
cout<<"No solution"<<endl;
return make_pair(trajConfigRaw,trajConfigSmooth);

}
// mode - true RRT, mode -false BiRRT
short RRT::extendRRT(NodeTree &NTree, const vector<double> &configRand, bool mode) // configRand should receive an address?? wait to see if other things work well
{

    //cout<<"Rand config  "<<configRand.at(0)<<"\t"<<configRand.at(1)<<"\t"<<configRand.at(2)<<"\t"<<configRand.at(3)<<"\t"<<configRand.at(4)<<"\t"<<configRand.at(5)<<"\t"<<configRand.at(6)<<endl;
    //cout<<"extend entered"<<endl;
    //NTree.printNodetree();
    vector<double> configNear;
    long tempID;
    NTree.nearestNeighbor(configRand, configNear, tempID);
    double uvector=0.0;

    /*
    for (int i = 0; i < configRand.size(); ++i)
    {
        cout<<"random config : "<<configRand[i]<<endl;
    }

    for (int i = 0; i < configRand.size(); ++i) {
        cout<<"near config : "<<configNear[i]<<endl;
    }

*/

    //  cout<<"closest id   "<<tempID<<endl;

    float parameter=0.0;
    long tempstop=0;
    double element;
    vector<double> configNew;
    bool goalReached = true;
    size_t k=0;

    while (true)
    {

        goalReached=true;
        configNew.clear();
        tempstop++;

        if(tempstop>200)
        {
            // This step is a hack to not get stuck in a branch for too long
            //  cout<<"interrupt"<<endl;
            // Once called, it would go back to buildRRT function to get another random configuration
            return 0;
        }

        uvector=0.0;
        for (uint i = 0; i < configRand.size(); ++i) {
            uvector+=pow(configRand[i]-configNear[i],2);
        }

        //   cout<<"euclidean:   "<<uvector<<endl;

        if(uvector<0.01)
        {
            // This is to ensure that we stop moving further when we are tooooo close to the goal point
            NTree.addNode(configRand,NTree.getNodeSize()+1,tempID);
            goalReached=true;
            for (size_t i = 0; i < configRand.size(); ++i) {
                if(configRand[i]!=goalConfig[i]) {
                    goalReached=false;
                }
                if(goalReached)
                {
                    if(mode)
                    {
                        cout<<endl<<"Goal Reached"<<endl;
                        return 1;
                    }
                }
            }
            //      cout<<"     local goal reached"<<endl;
            return 0;
        }

        for (size_t j = 0; j < configRand.size(); ++j) {


            if(uvector<0.20)
            {
                parameter=uvector;
            }
            else parameter=0.20;

            element=configNear[j]+parameter*((configRand[j]-configNear[j])/uvector);
            configNew.push_back(element);
            element=roundf(element*100.0)/100.0;

            if(element!=goalConfig[j]) {
                goalReached=false;
            }

        }
        //confignew

        // to check limits
        k=0;
        for (size_t i = 0; i < configRand.size(); ++i) {
            if(configNew[i]>qmin[i] && configNew[i]<qmax[i]) k++;
        }
        if(goalReached)
        {
            if(mode)
            {
                cout<<endl<<"Goal Reached"<<endl;
                return 1;
            }

        }
        else if(k!=goalConfig.size())
        {
            return 0;
        }
        else if(isCollision(configNew))
        {
            // cout<<"collision"<<endl;
            return 0;
        }
        else
        {
            NTree.addNode(configNew,NTree.getNodeSize()+1,tempID);
            //cout<<"\r"<<"Added node: "<<configNew.at(0)<<"\t"<<configNew.at(1)<<"\t"<<configNew.at(2)<<"\t"<<configNew.at(3)<<"\t"<<configNew.at(4)<<"\t"<<configNew.at(5)<<"\t"<<configNew.at(6)<<"\t"<<NTree.getNodeSize()<<flush;
            //cout<<"Added node: "<<configNew.at(0)<<"\t"<<configNew.at(1)<<"\t"<<configNew.at(2)<<"\t"<<configNew.at(3)<<"\t"<<configNew.at(4)<<"\t"<<configNew.at(5)<<"\t"<<configNew.at(6)<<"\t"<<NTree.getNodeSize()<<endl;
            //NTree.printNodetree(); // ask
            cout<<"\r"<<"Added Node Number # "<<NTree.getNodeSize()<<flush;
            tempID=NTree.getNodeSize();
            configNear=configNew;
        }

    }
    return 0;

}
//pair<int,vector<double>>
void RRT::executeTraj(vector<vector<double>> &trajConfig)
{
    TrajectoryBasePtr traj = RaveCreateTrajectory(env,"");
    traj->Init(robot->GetActiveConfigurationSpecification());
    for (size_t i = 0; i < trajConfig.size(); ++i) {
        //traj->Insert(i,trajConfig.at(trajConfig.size()-1-i),true);
        traj->Insert(i,trajConfig.at(i),true);
    }
    planningutils::RetimeActiveDOFTrajectory(traj,robot);
    robot->GetController()->SetPath(traj);
    cout<<"Trajectory Succesfully Executed !!! \n";
}

void RRT::findPath(NodeTree &NTree,vector<vector<double>> &trajConfig,bool reverseTree)
{
    long ID=0;
    // push the goal id first
    trajConfig.push_back(NTree.vecNodes.at(NTree.getNodeSize()-1)->config);

    ID=NTree.vecNodes.at(NTree.getNodeSize()-1)->parent_id;


    // find parent by id and push it
    while(ID!=0)
    {

        for (int i = 0; i < NTree.getNodeSize(); ++i) {

            if(NTree.vecNodes.at(i)->self_id==ID)
            {
                trajConfig.push_back(NTree.vecNodes.at(i)->config);
                ID=NTree.vecNodes.at(i)->parent_id;

                break;
            }
        }
    }
    if(reverseTree)    reverse(trajConfig.begin(),trajConfig.end());
}

void RRT::smoothPath(vector<vector<double>> &trajConfig)
{
    random=std::uniform_real_distribution<double>(0.0,trajConfig.size());
    std::mt19937_64 init_generator(rand_dev());
    int r1,r2;

    for (int i = 0; i < 200; ++i) {

        r1=int(random(init_generator))%trajConfig.size();
        r2=int(random(init_generator))%trajConfig.size();
        if(r1!=r2 && abs(r1-r2)>1)
        {
            if(r1<r2)
            {
                if(connectPath(trajConfig,r1,r2))
                {

                    trajConfig.erase(trajConfig.begin()+r1+1,trajConfig.begin()+r2);

                }
            }
            else
            {
                if(connectPath(trajConfig,r2,r1))
                {

                    trajConfig.erase(trajConfig.begin()+r2+1,trajConfig.begin()+r1);


                }
            }


        }
    }


}

bool RRT::connectPath(vector<vector<double>> &trajConfig,int r1,int r2)
{

    vector<double> configStart,configGoal;
    double uvector=0.0;
    float parameter=0.0;
    long tempstop=0;
    double element;
    size_t k=0;
    vector<double> configNew;
    configStart=trajConfig.at(r1);
    configGoal=trajConfig.at(r2);

    while (true)
    {
        configNew.clear();
        tempstop++;

        if(tempstop>200)
        {
            //cout<<"too many iterations"<<endl;
            return false;
        }

        uvector=0.0;
        for (uint i = 0; i < configGoal.size(); ++i) {
            uvector+=pow(configGoal[i]-configStart[i],2);
        }

        //   cout<<"euclidean:   "<<uvector<<endl;

        if(uvector<0.01)
        {
            // This is to ensure that we stop moving further when we are tooooo close to the goal point
            return true;
        }

        for (size_t j = 0; j < configGoal.size(); ++j) {


            if(uvector<0.20)
            {
                parameter=uvector;
            }
            else parameter=0.20;

            element=configStart[j]+parameter*((configGoal[j]-configStart[j])/uvector);
            configNew.push_back(element);
            element=roundf(element*100.0)/100.0;
        }
        //confignew
        k=0;
        for (size_t i = 0; i < configGoal.size(); ++i) {
            if(configNew[i]>qmin[i] && configNew[i]<qmax[i]) k++;
        }
        if(k!=goalConfig.size())
        {
            return false;
        }
        else if(isCollision(configNew))
        {
            // cout<<"collision"<<endl;
            return false;
        }
        else
        {
            configStart=configNew;
        }


    }

}

void RRT::buildBiRRT(NodeTree &NTree1,NodeTree &NTree2)
{

    vector<double> config,configBR;
    NTree1.addNode(startConfig,1,0);
    NTree2.addNode(goalConfig,1,0);
    int state=0;
    int j=0;

    while(1) // give some condition
    {
        config.clear();
        sampleRandomConfig(config,0);
        if(j%2)
        {
            state=extendRRT(NTree1,config,false);
            state=extendRRT(NTree2,NTree1.vecNodes.at(NTree1.getNodeSize()-1)->config,false);
        }
        else
        {
            state=extendRRT(NTree2,config,false);
            state=extendRRT(NTree1,NTree2.vecNodes.at(NTree2.getNodeSize()-1)->config,false);
        }
        if(NTree2.vecNodes.at(NTree2.getNodeSize()-1)->config==NTree1.vecNodes.at(NTree1.getNodeSize()-1)->config)
        {
            cout<<endl;
            cout<<" Path found   "<<endl;
            break;
        }
        j++;
    }
    vector<vector<double>> trajTree1,trajTree2;
    findPath(NTree1,trajTree1);
    findPath(NTree2,trajTree2,false);
    for (size_t i = 0; i < trajTree2.size(); ++i) {
        trajTree1.push_back(trajTree2.at(i));
    }
    cout<<"Length of trajectory path    "<<trajTree1.size()<<endl;
    executeTraj(trajTree1);

}
