//
// Created by Michael R. Shannon on 5/2/19.
//

#ifndef CURRENTSENSOR_H
#define CURRENTSENSOR_H

namespace slc {

    class CurrentSensor
    {
    public:
        virtual float amps() const = 0;

    };

}

#endif //CURRENTSENSOR_H
