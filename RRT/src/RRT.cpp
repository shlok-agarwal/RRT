#include <iostream>
#include<RRTNode.h>

using namespace std;

RRTNode::RRTNode()
{

}

RRTNode::Tree RRTNode::buildRRT(config startconfig)
{
    // Pushing first element in the tree
    T.push_back(startconfig);
    while(1)
    {
        // sample random configurations

    }

}
RRTNode::config RRTNode::sampleRandomConfig()
{

}

int main()
{
    cout << "Hello World   !!!!     !" << endl;
    return 0;
}

