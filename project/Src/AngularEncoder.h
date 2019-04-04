//
// Created by Michael R. Shannon on 4/3/19.
//

#ifndef ANGULARENCODER_H
#define ANGULARENCODER_H

namespace slc {

    class AngularEncoder
    {
    public:
        virtual void update(bool blocking = true) = 0;

        virtual bool new_data() const = 0;

        virtual int raw_position() const = 0;

        void calibrate(int position = 0);

        int position() const;

        int zero_offset() const;

        float degrees() const;

        float resolution_degrees() const;

        float radians() const;

        float resolution_radians() const;

    protected:
        explicit AngularEncoder(float resolution, int zero_offset = 0);

    private:
        float resolution_;
        int zero_offset_;

    };

}

#endif //ANGULARENCODER_H
