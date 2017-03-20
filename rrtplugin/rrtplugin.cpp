#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include <string>
#include<RRTNode.h>
#include<NodeTree.h>


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
        /*
        RegisterCommand("SetConfigDimension",boost::bind(&RRTPlugin::SetConfigDimension,this,_1,_2),
                        "This command sets system configuration");
        RegisterCommand("SetStartConfig",boost::bind(&RRTPlugin::SetStartConfig,this,_1,_2),
                        "This command sets start config");
        RegisterCommand("SetGoalConfig",boost::bind(&RRTPlugin::SetGoalConfig,this,_1,_2),
                        "This command sets goal config");
        RegisterCommand("RunRRT",boost::bind(&RRTPlugin::RunRRT,this,_1,_2),
                        "This command runs the RRT algorithm");
*/
        env = penv;
        env->GetRobots(n);
        robot=n.at(0); //selecting the first robot
    }
    virtual ~RRTPlugin() {}

    // Registered Commands
    bool Test(std::ostream& sout, std::istream& sinput)
    {

        cout << "Plugin Interface Working"<<endl;
        vector<float> sample;
        sample.assign(7,0.2);
        RRTNode Node(sample,1,1);
        Node.setParentID(2);
        Node.setSelfID(4);

        cout<<Node.getParentID()<<endl;
        cout<<Node.getSelfID()<<endl;
        cout<<Node.getJoint(2)<<endl;

        NodeTree N;
        N.addNode(sample,1,1);
        sample.assign(7,0.5);
        N.addNode(sample,2,1);
        sample.assign(7,0.9);
        N.addNode(sample,3,2);
        N.printNodetree();

        return true;
    }
    /*
    bool SetConfigDimension(std::ostream& sout, std::istream& sinput)
    {
        string filename;
        sinput >> filename;
        Node.dimension=atoi(filename.c_str());
        return true;
    }
    bool SetStartConfig(std::ostream& sout, std::istream& sinput)
    {
        string filename;
        getline(sinput,filename);

        std::istringstream stm(filename) ;
        float f ;
        while( stm >> f )    Node.startConfig.push_back(f) ;
        return true;
    }
    bool SetGoalConfig(std::ostream& sout, std::istream& sinput)
    {
        string filename;
        getline(sinput,filename);

        std::istringstream stm(filename) ;
        float f ;
        while( stm >> f )    Node.goalConfig.push_back(f) ;

        return true;
    }
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

