//
// Created by Michael R. Shannon on 4/4/19.
//

#ifndef STATUS_H
#define STATUS_H

namespace slc {

    /** Status enum.
     *
     * success = returned for a successful operation
     * failed = returned for a failed operation
     * working = returned when an operation is still underway, usally means busy
     * idle = default status
     *
     */
    enum class Status
    {
        success,
        failed,
        working,
        idle
    };

}

#endif //STATUS_H
