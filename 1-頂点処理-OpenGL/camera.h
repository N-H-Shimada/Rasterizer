#ifndef CAMERA_H
#define CAMERA_H

# include "ray.h"
using Eigen::Vector3d;


class Camera {
public:
    Vector3d origin;
    Vector3d endpoint;
    Vector3d up_vec;
    Vector3d x_vec;
    Vector3d y_vec;
    Vector3d z_vec;
    
    float n;
    float f;
    float w;
    float h;
    
    Camera() {
        // teapot bunny
        origin << 0.0,3.0,6.0;
        endpoint << -0.2,1.6,0.0;
        up_vec << 0.0,1.0,0.0;
        n = 2.0;
        f = 10.0;
        w = 1.0;
        h = 1.0;
        
        z_vec << (origin-endpoint)/(origin-endpoint).norm();
        x_vec << up_vec.cross(z_vec);
        y_vec << z_vec.cross(x_vec);
    }

    
//            ↓upper_left_corner
//            --------------------------
//            |                        |
//            |                        |
//            |                        |
//  vertical↑ |                        |
//            --------------------------
//            →horizontal
    
};

#endif
