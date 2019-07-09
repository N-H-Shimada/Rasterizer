#ifndef TRIANGLE_H
#define TRIANGLE_H

#include <cmath>

#include "ray.h"

using Eigen::Vector3d;
using Eigen::Vector4d;

class Triangle {
public:
    Vector4d A;
    Vector4d B;
    Vector4d C;
    
    Triangle() {}
    
    Triangle (const Vector4d& vertix_A, const Vector4d& vertix_B, const Vector4d& vertix_C) {
        A = vertix_A;
        B = vertix_B;
        C = vertix_C;
    }
    
    void Init (const Vector4d& vertix_A, const Vector4d& vertix_B, const Vector4d& vertix_C) {
        A = vertix_A;
        B = vertix_B;
        C = vertix_C;
    }
    

};

#endif
