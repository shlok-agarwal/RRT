#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include <string>
#include<RRTNode.h>

using namespace OpenRAVE;
using namespace std;

RRTNode Node;
class RRTPlugin : public ModuleBase
{
public:

    EnvironmentBasePtr env;
    RobotBasePtr robot;
    std::vector<RobotBasePtr> n; // gettting all the robots in the environment

    RRTPlugin(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {

        RegisterCommand("Test",boost::bind(&RRTPlugin::Test,this,_1,_2),
                        "This is a test command");
        RegisterCommand("SetConfigDimension",boost::bind(&RRTPlugin::SetConfigDimension,this,_1,_2),
                        "This command sets system configuration");
        RegisterCommand("SetStartConfig",boost::bind(&RRTPlugin::SetStartConfig,this,_1,_2),
                        "This command sets start config");
        RegisterCommand("SetGoalConfig",boost::bind(&RRTPlugin::SetGoalConfig,this,_1,_2),
                        "This command sets goal config");
        RegisterCommand("RunRRT",boost::bind(&RRTPlugin::RunRRT,this,_1,_2),
                        "This command runs the RRT algorithm");

        env = penv;
        env->GetRobots(n);
        robot=n.at(0); //selecting the first robot
    }
    virtual ~RRTPlugin() {}

    // Registered Commands
    bool Test(std::ostream& sout, std::istream& sinput)
    {

        sout << "Plugin Interface Working";
        return true;
    }

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

        return true;

    }

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

