#include "MatchingCrater.hpp"
#include "getdata.hpp"
#include "debug.cxx"
#include <boost/program_options.hpp>

using namespace std;
namespace po = boost::program_options;

int main(int argc, char *argv[])
{
    // 启动内存检查线程
    std::thread memoryThread(checkMemoryUsage);
    try 
    {        
        bool filter;
        bool show_image;
        po::options_description desc("Allowed options");
        desc.add_options()
            ("help", "produce help message")
            ("N1", po::value<std::string>()->required(), "set name1 value")
            ("N2", po::value<std::string>()->required(), "set name2 value")
            ("offset", po::value<std::string>(), "set offset value")
            ("variance", po::value<std::string>(), "set variance value")
            ("filter", po::value<bool>(&filter)->default_value(false), "set whether filter")
            ("image", po::value<bool>(&show_image)->default_value(true), "set whether show image");

        po::variables_map vm;
        try 
        {
            po::store(po::parse_command_line(argc, argv, desc), vm);

            if (vm.count("help")) 
            {
                std::cout << desc << std::endl;
                return 0;
            }

            po::notify(vm);
        } 
        catch (const po::error& e) 
        {
            std::cerr << "Error: " << e.what() << std::endl;
            std::cout << desc << std::endl;
            return 1;
        }

        cout << "\n匹配目标: " << vm["N1"].as<std::string>() << " " << vm["N2"].as<std::string>() << endl;
        std::string N1 = vm["N1"].as<std::string>();
        std::string N2 = vm["N2"].as<std::string>();

        MatchingCrater* MC;
        if (vm.count("offset") && vm.count("variance")) 
        {
            std::string offset = vm["offset"].as<std::string>();
            std::string variance = vm["variance"].as<std::string>();
            MC = new MatchingCrater(N1, N2, offset, variance, filter, show_image);
        } 
        else
        {
            MC = new MatchingCrater(N1, N2, filter, show_image);
        }
        MC->runMatching();
        MC->get_keys();
        delete MC;
        
    }
    catch (std::exception& e) 
    {
        std::cerr << "Error: " << e.what() << std::endl;        
        return 1;
    }
    catch (...) 
    {
        std::cerr << "Unknown error!" << std::endl;        
        return 1;
    }
    memoryThread.join();
    return 0;
}
