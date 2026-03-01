// Lock.hpp

#ifndef LOCK_HPP
#define LOCK_HPP

#include "vex.h"


class Lock {
    public:
        vex::mutex& lck;

        void lock();

        explicit Lock(vex::mutex& l, bool adopted = false);

        ~Lock();

        Lock(const Lock&) = delete;
        Lock& operator=(const Lock&) = delete;

};

#endif // LOCK_HPP