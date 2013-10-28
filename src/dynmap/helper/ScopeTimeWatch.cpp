/**
 * @file ScopeTimeWatch.cpp
 *
 *  @date 10.11.2012
 *      @author Iwer Petersen
 */

#include "ScopeTimeWatch.h"
#include <iostream>

ScopeTimeWatch::ScopeTimeWatch(const char * scope) :
        start_(std::clock()), scope_(scope) {
}

ScopeTimeWatch::~ScopeTimeWatch() {
    clock_t total = std::clock() - start_; //get elapsed time
    std::cout << "Scopetime of '" << scope_ << "' : " << double(total) / CLOCKS_PER_SEC << " s" << std::endl;
}

