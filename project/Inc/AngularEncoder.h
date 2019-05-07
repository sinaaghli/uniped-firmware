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

    class AngularEncoder : virtual Sensor
    {
    public:

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
                unsigned int positions_per_revolution,
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
