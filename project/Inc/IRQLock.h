//
// Created by Michael R. Shannon on 3/5/19.
//

#include <cstdint>

#ifndef IRQLOCK_H
#define IRQLOCK_H


namespace slc {

    /** Use to prevent interrupts from taking control from a critical section.
     *
     * This class uses RAII to keep the lock until it goes out of scope.
     *
     */
    class IRQLock
    {
    public:
        IRQLock();

        ~IRQLock();

        void lock();

        void unlock();

    private:
        uint32_t primask_;
        bool locked_;
    };

}


#endif //IRQLOCK_H
