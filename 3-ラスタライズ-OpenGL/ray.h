#ifndef RAY_H
#define RAY_H

using Eigen::Vector3d;

class Ray {
public:
    Vector3d o, d;
    
    Ray() {} // インスタンス生成: "Ray ray;" <- ()要らない
    Ray (const Vector3d origin, const Vector3d direction) {
        o = origin;
        d = direction;
    }
    
    Vector3d point_at_parameter (float t) {
        Vector3d temp;
        temp = o + d*t;
        return temp;
    }
};

#endif
