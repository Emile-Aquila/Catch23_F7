//
// Created by emile on 23/08/21.
//

#include "math.h"

float clip_f(float value, float min, float max){
    return fminf(max, fmaxf(value, min));
}