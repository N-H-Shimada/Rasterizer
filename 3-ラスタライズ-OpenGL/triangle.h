#ifndef TRIANGLE_H
#define TRIANGLE_H

#include <algorithm>

using namespace std;
using Eigen::Vector3d;
using Eigen::Vector4d;

class Triangle {
public:
    Vector4d A, B, C;
    Vector3d N_A, N_B, N_C;
    float min_x, min_y, max_x, max_y;
    float RGB_A, RGB_B, RGB_C;
    
    Triangle() {}
    
    Triangle (const Vector4d& vertix_A, const Vector4d& vertix_B, const Vector4d& vertix_C, const Vector3d& vertix_NA, const Vector3d& vertix_NB, const Vector3d& vertix_NC) {
        A = vertix_A;
        B = vertix_B;
        C = vertix_C;
        N_A = vertix_NA;
        N_B = vertix_NB;
        N_C = vertix_NC;
    }
    
    void Init (const Vector4d& vertix_A, const Vector4d& vertix_B, const Vector4d& vertix_C) {
        A = vertix_A;
        B = vertix_B;
        C = vertix_C;
    }
    
    void Init_N (const Vector3d& vertix_A, const Vector3d& vertix_B, const Vector3d& vertix_C) {
        N_A = vertix_A;
        N_B = vertix_B;
        N_C = vertix_C;
    }
    
    void Bounding_box () {
        min_x = min({A(0),B(0),C(0)});
        min_y = min({A(1),B(1),C(1)});
        max_x = max({A(0),B(0),C(0)});
        max_y = max({A(1),B(1),C(1)});
    }

};

#endif
