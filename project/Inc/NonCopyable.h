//
// Created by Michael R. Shannon on 4/4/19.
//

#ifndef NONCOPYABLE_H
#define NONCOPYABLE_H

namespace slc {

    class NonCopyable
    {
    protected:
        NonCopyable() = default;

        ~NonCopyable() = default;

        NonCopyable(const NonCopyable &) = delete;

        void operator=(const NonCopyable &) = delete;

        NonCopyable(NonCopyable &&) = default;

        NonCopyable &operator=(NonCopyable &&) = default;
    };

}


#endif //NONCOPYABLE_H
