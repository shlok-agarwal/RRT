#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <boost/bind.hpp>
using namespace std;
using namespace OpenRAVE;

class RRTplugin : public ModuleBase
{
public:

    RRTplugin(EnvironmentBasePtr penv) : ModuleBase(penv)
    {
        std::cout << "Constructing plugin... ";
        __description = "plugin to print hello";
        RegisterCommand("hello",boost::bind(&RRTplugin::Hello,this,_1,_2),"prints hello");
        std::cout << "DONE" << std::endl;
    }
    bool Hello(ostream& sout, istream& sinput)
    {

        sout << "Hello";
        return true;
    }

};



InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "rrtplugin" ) {
        return InterfaceBasePtr(new RRTplugin(penv));
    }
    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO &info) {
    info.interfacenames[PT_Module].push_back("RRTplugin");

}
