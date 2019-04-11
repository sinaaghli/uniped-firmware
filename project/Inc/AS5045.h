//
// Created by Michael R. Shannon on 4/4/19.
//

#ifndef AS5045_H
#define AS5045_H

#include <memory>
#include <utility>
#include <functional>
#include "SerialPeripheralInterface.h"
#include "AngularEncoder.h"
#include "NonCopyable.h"
#include "Status.h"
#include "AS5045Driver.h"


namespace slc {

    class AS5045 : public AngularEncoder
    {
        friend class AS5045Chain;

    public:

        explicit AS5045(std::shared_ptr<AS5045Driver> driver, size_t index = 0,
                        bool reversed = false, int zero_offset = 0);

        std::shared_ptr<AS5045Driver> driver();

        Status sample(bool blocking) override;

        Status status() const override;

        size_t sample_count() const override;

        bool busy() const override;

        std::pair<size_t, int> raw_position() const override;

    private:
        static constexpr unsigned int positions_per_revolution = 4096;
        mutable Status status_ = Status::idle;
        mutable size_t sample_count_ = 0;
        mutable int raw_position_ = 0;
        const size_t index_;
        std::shared_ptr<AS5045Driver> driver_;

        void check_for_new_() const;

    };

}


#endif //AS5045_H
