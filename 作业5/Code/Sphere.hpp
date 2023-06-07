#pragma once

#include "Object.hpp"
#include "Vector.hpp"

class Sphere : public Object
{
public:
    Sphere(const Vector3f& c, const float& r)
        : center(c)
        , radius(r)
        , radius2(r * r)
    {}

<<<<<<< HEAD
    bool intersect(const Vector3f& orig, const Vector3f& dir, float& tnear, uint32_t&, Vector2f&) const override
    {
        // analytic solution
        Vector3f L = orig - center;
=======
    //tnear是返回的距离的最近的焦点距离
    bool intersect(const Vector3f& orig, const Vector3f& dir, float& tnear, uint32_t&, Vector2f&) const override
    {
        // analytic solution
        Vector3f L = orig - center; //光线与球心之间的向量L
>>>>>>> 0f0be9e00abf61476239abf73c196b3d2e29c709
        float a = dotProduct(dir, dir);
        float b = 2 * dotProduct(dir, L);
        float c = dotProduct(L, L) - radius2;
        float t0, t1;
<<<<<<< HEAD
        if (!solveQuadratic(a, b, c, t0, t1))
            return false;
=======
        if (!solveQuadratic(a, b, c, t0, t1)) //方程有实根才继续
            return false;
        //返回近一点的那个解
>>>>>>> 0f0be9e00abf61476239abf73c196b3d2e29c709
        if (t0 < 0)
            t0 = t1;
        if (t0 < 0)
            return false;
        tnear = t0;

        return true;
    }

    void getSurfaceProperties(const Vector3f& P, const Vector3f&, const uint32_t&, const Vector2f&,
                              Vector3f& N, Vector2f&) const override
    {
        N = normalize(P - center);
    }

    Vector3f center;
    float radius, radius2;
};
