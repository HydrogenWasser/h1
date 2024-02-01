#include "json_ServiceArea.hpp"
using json = nlohmann::json;

class ParaMgr_ServiceArea
{
public:
    ParaMgr_ServiceArea();
    ~ParaMgr_ServiceArea();
    bool jsonload(std::string filepath);
    bool get(json &myjson);

private:
    json json_data;
};