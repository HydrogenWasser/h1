#ifndef PARAMGRVIDEO_H
#define PARAMGRVIDEO_H

#include "json.hpp"
using json = nlohmann::json;

class ParaMgrVideo
{
public:
    ParaMgrVideo();
    ~ParaMgrVideo();
    bool jsonload(std::string filepath);
    bool get(json &myjson);

private:
    json json_data;
};
#endif