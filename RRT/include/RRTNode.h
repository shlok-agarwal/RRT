#include<iostream>
#include <vector>

class RRTNode
{
public:
    /*
    std::vector<float> config;
    void storeconfig(std::vector<float> _config); // Function should store configuration in a 2D vector array
    //std::vector<> getparent(std::vector<float> _config); // Function should return parent for particular parent. Return -1 for non matching config file
    void storeNode(Node _node);
    bool addNode(Node _node);
    bool deleteNode(Node _node);
    Node getNode(int _index);
    std::vector<Node> getPath(int _index);
    struct Node
    {

    }; */

    typedef std::vector<float> config;
    typedef std::vector<config> Tree;
    Tree T;
    Tree buildRRT(config startconfig);
    RRTNode();
    config sampleRandomConfig();





};
