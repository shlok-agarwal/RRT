#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include <string>
#include<RRTNode.h>
#include<NodeTree.h>
#include<RRT.h>


using namespace OpenRAVE;
using namespace std;

class RRTPlugin : public ModuleBase
{
public:

    EnvironmentBasePtr env;
    RobotBasePtr robot;
    std::vector<RobotBasePtr> n; // gettting all the robots in the environment

    RRTPlugin(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {

        RegisterCommand("Test",boost::bind(&RRTPlugin::Test,this,_1,_2),
                        "This is a test command");
        RegisterCommand("StartRRT",boost::bind(&RRTPlugin::StartRRT,this,_1,_2),
                        "This command runs the RRT algorithm");
        RegisterCommand("StartBiRRT",boost::bind(&RRTPlugin::StartBiRRT,this,_1,_2),
                        "This command runs the RRT algorithm");
        env = penv;
        env->GetRobots(n);
        robot=n.at(0); //selecting the first robot
    }
    virtual ~RRTPlugin() {}

    // Registered Commands
    bool Test(std::ostream& sout, std::istream& sinput)
    {

        cout << "Plugin Interface Working"<<endl;
        return true;
    }
    bool StartRRT(std::ostream& sout, std::istream& sinput)
    {
        string filename;
        getline(sinput,filename);

        std::istringstream stm(filename) ;

        std::vector<int> jindex;
        jindex=robot->GetActiveDOFIndices();

        std::vector<double> startConfig;
        std::vector<double> goalConfig;
        float goalBias;


        double f;
        uint i=0;
        while( stm >> f )
        {

            if(i<7)
            {
                startConfig.push_back(f) ;
            }
            else if(i>=7 && i<14)
            {
                goalConfig.push_back(f) ;
            }
            else goalBias=f;
            ++i;
        }

        RRT R(startConfig,goalConfig,goalBias,env);
        NodeTree N;
        vector <vector<double>> trajConfigRaw,trajConfigSmooth;

        cout<<"RRT-Connect"<<endl;
        cout<<endl;
        cout<<"Goal Bias    "<<goalBias<<endl;
        pair<vector <vector<double>>,vector <vector<double>>> path=R.buildRRT(N);
        trajConfigRaw=path.first;
        trajConfigSmooth=path.second;

        // To plot the trajectory points, the trajectories are sent to the python file
        // Sending size in advance to know where to split
        sout<<trajConfigRaw.size();
        sout<<",";
        sout<<trajConfigSmooth.size();

        for (size_t i = 0; i < trajConfigRaw.size(); ++i) {
            for (size_t j = 0; j < startConfig.size(); ++j) {
                sout<<",";
                sout<<trajConfigRaw.at(i)[j];

            }

        }
        for (size_t i = 0; i < trajConfigSmooth.size(); ++i) {
            for (size_t j = 0; j < startConfig.size(); ++j) {
                sout<<",";
                sout<<trajConfigSmooth.at(i)[j];

            }

        }



        return true;

    }
    bool StartBiRRT(std::ostream& sout, std::istream& sinput)
    {
        string filename;
        getline(sinput,filename);

        std::istringstream stm(filename) ;

        std::vector<int> jindex;
        jindex=robot->GetActiveDOFIndices();

        std::vector<double> startConfig;
        std::vector<double> goalConfig;
        float goalBias=0.0;

        double f;
        uint i=0;
        while( stm >> f )
        {
            if(i<jindex.size())
            {
                startConfig.push_back(f) ;
            }
            else
            {
                goalConfig.push_back(f) ;
            }
            ++i;
        }

        RRT BR(startConfig,goalConfig,goalBias,env);
        NodeTree N1,N2;
        auto start = get_time::now();
        cout<<"BiDirectional RRT"<<endl;
        BR.buildBiRRT(N1,N2);
        auto end = get_time::now();
        auto diff = end - start;
        cout<<"Success!!    Time taken to find Goal:   "<<chrono::duration_cast<ns>(diff).count()<<" seconds"<<endl;
        cout<<"Number of Nodes Sampled  "<<N1.vecNodes.size()+N2.vecNodes.size()<<endl;


        return true;
    }
    /*
    bool RunRRT(std::ostream& sout, std::istream& sinput)
    {
        // Calculate other essentials needed to run the program
        // 1. Joint Limits
        std::vector<dReal> qmin, qmax;
        std::vector<int> jindex,jcircular;
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

        // 2. Generating random configuratin: TODO : Account for goal bias later
        std::uniform_real_distribution<double> random;
        random=std::uniform_real_distribution<double>(0.0,1.0);
        std::random_device rand_dev;
        std::mt19937_64 init_generator(rand_dev());
        cout<<"random number "<<random(init_generator)<<endl;
        cout<<"random number "<<random(init_generator)<<endl

        return true;

    }
*/
};
// Write and call functions here

// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "rrtplugin" ) {
        return InterfaceBasePtr(new RRTPlugin(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_Module].push_back("RRTPlugin");
    
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}

