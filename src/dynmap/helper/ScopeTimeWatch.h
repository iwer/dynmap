/**
 * @file ScopeTimeWatch.h
 *
 *  @date 10.11.2012
 *      @author Iwer Petersen
 */

#ifndef SCOPETIMEWATCH_H_
#define SCOPETIMEWATCH_H_

#include <ctime>

/**
 * @class ScopeTimeWatch
 * @brief Time watch for scope time
 *
 * Time starts on creation and stops at deletion. Simply create a local:
 *      ScopeTimeWatch tw = new ScopeTimeWatch("My Scope");
 */
 class ScopeTimeWatch {
public:
    ScopeTimeWatch(const char * scope);
    ~ScopeTimeWatch();
private:
    /**
     * start time
     */
    std::clock_t start_;
    /**
     * Scope name for identification in output
     */
    const char * scope_;
};

#endif /* SCOPETIMEWATCH_H_ */
