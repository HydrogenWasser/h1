#include "json.hpp"
using json = nlohmann::json;

class ParaMgrScene
{
public:
    ParaMgrScene();
    ~ParaMgrScene();
    bool jsonload(std::string filepath);
    bool get(json &myjson);

private:
    json json_data;
};