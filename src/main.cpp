#include "MatchingCrater.hpp"
#include "debug.cxx"

using namespace std;

int main(int argc, char *argv[])
{
    if (argc > 1)
    {
        cout << "login success" << endl;        
        std::string name1 = argv[1];
        std::string name2 = argv[2];
        cout << name1 << " " << name2 << endl;
        //bool flag = std::string(argv[3]) == "true";        
        bool flag = true;
        MatchingCrater* MC = new MatchingCrater(name1, name2, flag);
        MC->runMatching();
        MC->test_keys();
    }
    return 0;
}
