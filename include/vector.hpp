#ifndef __VECTOR_HPP__
#define __VECTOR_HPP__

#include "types.hpp"
#include "immintrin.h"

class Quat {
    public:
        Quat(f32 x, f32 y, f32 z, f32 w) {

        }
    private:
        __m128 data;
};

#endif
