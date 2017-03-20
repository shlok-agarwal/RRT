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
    RRTNode();
    typedef std::vector<float> config;
    typedef std::vector<config> Tree;
    int self_id;
    int parent_id;


    int dimension;
    config startConfig;
    config goalConfig;
    config getConfig(int self_id);
    config getParent(int self_id);

    config sampleRandomConfig(); // should implement in RunRRT
};
class NodeTree :public RRTNode
{
public:

    bool addNode(int self_id, config);
    bool deleteNode(int self_id);
    bool getNode(int self_id);
    Tree getPath(int self_id);
    config nearestNode(config);

};





