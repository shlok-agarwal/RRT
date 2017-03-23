#include <iostream>
#include<NodeTree.h>
#include<RRT.h>

using namespace std;
bool RRT::buildRRT(NodeTree &NTree)
{
    // Maximum number of iterations. This could also be a parameter of time
    long maxiterations=100000;
    vector<double> config;

    NTree.addNode(startConfig,1,0);
    short state=0;
    auto start = get_time::now();
    for (long i = 0; i < maxiterations; ++i) {
        config.clear();
        sampleRandomConfig(config);
        state=extend(NTree,config);

        //  cout<<"state:   "<<state<<endl;

        //   cout<<"iteration"<<endl;
        if(state==1)
        {
            break;
        }

    }
    if(state==1)
    {
        auto end = get_time::now();
        auto diff = end - start;
        cout<<"Success!!    Time Elapsed:   "<<chrono::duration_cast<ns>(diff).count()<<" seconds"<<endl;
        //NTree.printNodetree();
        vector <vector<double>> trajconfig;
        executeTraj(NTree,trajconfig);
        return true;
    }
    else
        return false;
}
short RRT::extend(NodeTree &NTree,const vector<double> configRand) // configRand should receive an address?? wait to see if other things work well
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
                    cout<<"Goal Reached"<<endl;
                    return 1;
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

        if(goalReached)
        {
            cout<<"Goal Reached"<<endl;
            return 1;

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
void RRT::executeTraj(NodeTree &NTree,vector<vector<double>> &trajConfig)
{
    long ID=0;
    // push the goal id first
    trajConfig.push_back(NTree.vecNodes.at(NTree.getNodeSize()-1)->config);

    ID=NTree.vecNodes.at(NTree.getNodeSize()-1)->parent_id;
    //cout<<"goal id:"<<ID<<endl;

    // find parent by id and push it
    while(ID!=0)
    {
        // cout<<"node size"<<NTree.getNodeSize()<< "\n";
        for (int i = 0; i < NTree.getNodeSize(); ++i) {

            if(NTree.vecNodes.at(i)->self_id==ID)
            {
                trajConfig.push_back(NTree.vecNodes.at(i)->config);
                ID=NTree.vecNodes.at(i)->parent_id;

                break;
            }
        }
    }
    //std::cout<<"Traj Config size:"<<trajConfig.size()<<std::endl;
    /*  for (size_t i = 0; i < trajConfig.size(); ++i) {
        for (int j = 0; j < 7; ++j) {
            cout<<"config "<<i<<"    "<<trajConfig.at(i)[j]<<endl;
        }
    }*/
    cout<<"start planning \n";
    // Creating trajectory : used from openrave example
    TrajectoryBasePtr traj = RaveCreateTrajectory(env,"");
    traj->Init(robot->GetActiveConfigurationSpecification());
    for (size_t i = 0; i < trajConfig.size(); ++i) {
        traj->Insert(i,trajConfig.at(trajConfig.size()-1-i),true);
    }
    cout<<"inserted points \n";
    //    planningutils::ReverseTrajectory(traj);
    planningutils::RetimeActiveDOFTrajectory(traj,robot);
    //smoothPath(trajConfig);

    robot->GetController()->SetPath(traj);
    cout<<"trajectory done \n";
}
void RRT::smoothPath(vector<vector<double>> &trajConfig)
{
    random=std::uniform_real_distribution<double>(0.0,trajConfig.size());
    std::mt19937_64 init_generator(rand_dev());
    int r1=int(random(init_generator));
    int r2=int(random(init_generator));

    for (int i = 0; i < 200; ++i) {
        r1=int(random(init_generator));
        r2=int(random(init_generator));
        if(r1!=r2)
        {
            if(connectPath(trajConfig,r1,r2))
            {
                if(r1<r2) trajConfig.erase(trajConfig.begin()+r1+1,trajConfig.begin()+r2);
                        else trajConfig.erase(trajConfig.begin()+r2+1,trajConfig.begin()+r1);
            }
        }
    }


}
bool RRT::connectPath(vector<vector<double>> &trajConfig,int r1,int r2)
{
    // calculate step size

    vector<double> configNew;
    vector<double> configStart;
    vector<double> configGoal;

    configStart=trajConfig.at(r1);
    configGoal=trajConfig.at(r2);
    float uvector=0.0;

    float parameter=0.0,element=0.0;
    bool goalReached=true;
    while(1)
    {
        for (size_t j = 0; j < configStart.size(); ++j) {
            uvector=0.0;
            vector<double> configNew;
            bool goalReached = true;
            for (uint i = 0; i < configStart.size(); ++i) {
                uvector+=pow(configGoal[i]-configStart[i],2);
            }
            if(uvector<0.001)
            {
                // delete points between the two
                return 1;
            }
            if(uvector)

                if(uvector<0.30)
                {
                    parameter=uvector;
                }
                else parameter=0.30;
            element=configStart[j]+parameter*((configGoal[j]-configStart[j])/uvector);
            configNew.push_back(element);
            element=roundf(element*100.0)/100.0;

            if(element!=configGoal[j]) {
                goalReached=false;
            }

        }
        //confignew

        if(goalReached)
        {
            // delete points between the two
            return 1;

        }
        else if(isCollision(configNew))
        {
            // cout<<"collision"<<endl;
            return 0;
        }
    }



}

