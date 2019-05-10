//
// Created by Michael R. Shannon on 4/3/19.
//

#include <cstddef>
#include <utility>
#include <chrono>

#include "Status.h"
#include "Sensor.h"

#ifndef ANGULARENCODER_H
#define ANGULARENCODER_H


namespace slc {

    /** Angular encoder base class.
     *
     * Angular encoders are sensors that read an angle as a integer in a
     * range from 0 to a given maximum.
     *
     * With the exception of raw_value, this class uses continuous angles,
     * in other words a full rotation forward is 360 degrees and a full
     * rotation backwards is -360, not 0.
     *
     */
    class AngularEncoder : virtual Sensor
    {
    public:

        /** Get the raw encoder value.
         *
         * @return pair of sample count and raw value
         */
        virtual std::pair<size_t, int> raw_value() const = 0;

        void calibrate(int position = 0);

        int zero_offset() const;

        std::pair<size_t, int> position() const;

        unsigned int revolution_size() const;

        std::pair<size_t, float> degrees() const;

        float resolution_degrees() const;

        std::pair<size_t, float> radians() const;

        float resolution_radians() const;

        std::pair<size_t, float> rpm() const;

    protected:
        explicit AngularEncoder(
                unsigned int revolution_size,
                bool reversed = false, int zero_offset = 0);

    private:
        unsigned int revolution_size_;
        bool reversed_;
        int zero_offset_;
        mutable int revolutions_ = 0;
        mutable int old_raw_value_ = 0;
        mutable float rpm_ = 0.0;
        mutable std::chrono::high_resolution_clock::time_point old_time_;

    };

}


#endif //ANGULARENCODER_H
