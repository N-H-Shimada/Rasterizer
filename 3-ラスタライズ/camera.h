#ifndef CAMERA_H
#define CAMERA_H

using Eigen::Vector3d;


class Camera {
public:
    Vector3d origin, endpoint, up_vec;
    Vector3d x_vec, y_vec, z_vec;
    float n, f, w, h;
    
    Camera() {
        // teapot bunny
        origin << 0.0,5.0,10.0;
        endpoint << 0.0,1.0,0.0;
        up_vec << 0.0,1.0,0.0;
        n = 2.0;
        f = 10.0;
        w = 1.0;
        h = 1.0;
        
        // sphere-cone
//        origin << 0.0,6.0,28.0;
//        endpoint << -0.2,1.6,0.0;
//        up_vec << 0.0,1.0,0.0;
//        n = 2.0;
//        f = 10.0;
//        w = 1.0;
//        h = 1.0;
        
        z_vec << (origin-endpoint)/(origin-endpoint).norm();
        x_vec << up_vec.cross(z_vec);
        y_vec << z_vec.cross(x_vec);
    }
    
};

#endif
