#pragma once

#include <vector>
#include <string>

#include "TEventB3Data.h"
#include "TEventB4Data.h"
#include "TEventB5Data.h"

struct TEventData
{
    TEventB3Data m_B3Data;
    TEventB4Data m_B4Data;
    TB5Data m_B5Data;
};