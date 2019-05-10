//
// Created by Michael R. Shannon on 5/2/19.
//

#ifndef CURRENTSENSOR_H
#define CURRENTSENSOR_H

namespace slc {

    /** Current sensor interface.
     *
     */
    class CurrentSensor
    {
    public:

        /** Get the current in amperes.
         *
         * @return current in amperes
         */
        virtual float amps() const = 0;

    };

}

#endif //CURRENTSENSOR_H
