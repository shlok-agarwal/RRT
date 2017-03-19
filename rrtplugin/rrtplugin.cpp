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
    RRTPlugin(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        RegisterCommand("Test",boost::bind(&RRTPlugin::Test,this,_1,_2),
                        "This is a test command");
        RegisterCommand("SetConfigDimension",boost::bind(&RRTPlugin::SetConfigDimension,this,_1,_2),
                        "This command sets system configuration");
        RegisterCommand("SetStartConfig",boost::bind(&RRTPlugin::SetStartConfig,this,_1,_2),
                        "This command sets start config");
        RegisterCommand("SetGoalConfig",boost::bind(&RRTPlugin::SetGoalConfig,this,_1,_2),
                        "This command sets goal config");

    }
    virtual ~RRTPlugin() {}
    
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

