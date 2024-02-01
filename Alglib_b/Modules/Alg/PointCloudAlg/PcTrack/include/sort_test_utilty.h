//
// Created by root on 2/16/22.
//

#ifndef SORT_TRACKER_SORT_TEST_UTILTY_H
#define SORT_TRACKER_SORT_TEST_UTILTY_H

#include <sys/stat.h>
#include <unistd.h>
#include <string>
#include <fstream>
inline bool exists_file (const std::string& name) {
    struct stat buffer;
    return (stat (name.c_str(), &buffer) == 0);
}

#endif //SORT_TRACKER_SORT_TEST_UTILTY_H
