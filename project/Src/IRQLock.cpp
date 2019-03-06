//
// Created by Michael R. Shannon on 3/5/19.
//

// These are needed because CMSIS is broken.
#include <cstdint>

#define __ASM            __asm
#define __STATIC_INLINE  static inline

#include <cmsis_gcc.h>

#include "IRQLock.h"


namespace slc {

    /** Construct the IRQ lock, disabling the IRQ.
     *
     */
    IRQLock::IRQLock() : primask_(__get_PRIMASK()), locked_(true)
    {
        __disable_irq();
    }

    /** Destruct the IRQ lock, enable the IRQ if it was enabled when locked.
     *
     */
    IRQLock::~IRQLock()
    {
        unlock();
    }

    /** Disable the IRQ.
     *
     */
    void IRQLock::lock()
    {
        if (!locked_)
        {
            primask_ = __get_PRIMASK();
            locked_ = true;
            __disable_irq();
        }
    }

    /** Enable the IRQ if it was enabled when locked.
     *
     */
    void IRQLock::unlock()
    {
        if (locked_)
        {
            locked_ = false;
            if (!primask_)
            {
                __enable_irq();
            }
        }
    }

}
