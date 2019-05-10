//
// Created by Michael R. Shannnon on 5/2/19.
//

#ifndef DISTANCESENSOR_H
#define DISTANCESENSOR_H

namespace slc {

    /** Distance sensor base class.
     *
     */
    class DistanceSensor
    {
    public:
        void calibrate(float meters = 0.0);

        /** Get absolute, uncalibrated distance in meters.
         *
         * Must be implemented in subclass.
         *
         * @return uncalibrated distance in meters
         */
        virtual float absolute_meters() const = 0;

        float absolute_inches() const;

        float meters() const;

        float inches() const;

    protected:
        explicit DistanceSensor(float zero_offset = 0.0);

    private:
        float zero_offset_ = 0.0;

    };

}

#endif //DISTANCESENSOR_H
