#include <openrave/plugin.h>
#include <boost/bind.hpp>

using namespace OpenRAVE;

class RRTPlugin : public ModuleBase
{
public:
    RRTPlugin(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        RegisterCommand("Test",boost::bind(&RRTPlugin::Test,this,_1,_2),
                        "This is a test command");
    }
    virtual ~RRTPlugin() {}
    
    bool Test(std::ostream& sout, std::istream& sinput)
    {

        sout << "Plugin Interface Working";
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

