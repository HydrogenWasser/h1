#include <iostream>
#include <fstream>
#include <unistd.h>
#include "ParaMgrScene.h"

ParaMgrScene::ParaMgrScene()
{
}

ParaMgrScene::~ParaMgrScene()
{
}

bool ParaMgrScene::jsonload(std::string filepath)
{
    std::string::size_type name_pos = filepath.find_last_of('/') + 1;
    std::string filename = filepath.substr(name_pos, filepath.length() - name_pos);
    std::string file_extension = filepath.substr(filepath.find_last_of('.') + 1);
    std::string name = filename.substr(0, filename.rfind("."));
    if (file_extension.compare("json") != 0)
    {
        return false;
    }
    std::ifstream ifs(filepath, std::ios::in);
    if (!ifs.good())
    {
        std::cout << filepath << " No such Json File\n"
                  << std::endl;
        return false;
    }
    if (!(ifs >> (json_data[name])))
    {
        std::cout << filepath << " Error reading Json File\n"
                  << std::endl;
        return false;
    }
    return true;
};

bool ParaMgrScene::get(json &myjson)
{
    myjson = json_data;
    return true;
};

// test code
// int main()
// {
//     ParaMgr *my_paramgr = new ParaMgr();
//     my_paramgr->load("fusion_class.json");
//     my_paramgr->load("fusion_manager.json");
//     json my_param = nullptr;
//     bool flag = my_paramgr->get(my_param);
//     // copy(my_param);
//     // std::cout << &my_param << std::endl;
//     for (auto item : my_param.items())
//     {
//         std::cout << item.key() << ": " << item.value() << std::endl;
//     }
//     return 0;
// };