//triple.cpp

#include "triple.h"
#include <assert.h>

template <typename T>
triple<T>::triple(T e0, T e1, T e2)
{
    e[0] = e0;
    e[1] = e1;
    e[2] = e2;
}

template <typename T>
const triple<T>& triple<T>::operator+() const
{return *this;}


template <typename T>
triple<T> triple<T>::operator-() const
{return triple<T>(-e[0], -e[1], -e[2]);}

template <typename T>
bool operator==(const triple<T>& v1, const triple<T>& v2)
{
    if (v1.e[0] != v2.e[0]) return false;
    if (v1.e[1] != v2.e[1]) return false;
    if (v1.e[2] != v2.e[2]) return false;
    return true;
}

template <typename T>
bool operator!=(const triple<T>& v1, const triple<T>& v2)
{
    return !(v1==v2);
}

template <typename T>
triple<T> operator+(const triple<T>& v1, const triple<T>& v2)
{
    return triple<T>(v1.e[0]+v2.e[0], v1.e[1]+v2.e[1], v1.e[2]+v2.e[2]);
}

template <typename T>
triple<T> operator-(const triple<T>& v1, const triple<T>& v2)
{
    return triple<T>(v1.e[0]-v2.e[0], v1.e[1]-v2.e[1], v1.e[2]-v2.e[2]);
}

template <typename T>
triple<T> operator/(const triple<T>& v, T scalar)
{
    return triple<T>(v.e[0]/scalar, v.e[1]/scalar, v.e[2]/scalar);
}

template <typename T>
triple<T> operator*(const triple<T>& v, T scalar)
{
    return triple<T>(v.e[0]*scalar, v.e[1]*scalar, v.e[2]*scalar);
}

template <typename T>
triple<T> operator*(T scalar, const triple<T>& v)
{
    return triple<T>(v.e[0]*scalar, v.e[1]*scalar, v.e[2]*scalar);
}

template <typename T>
triple<T>& triple<T>::operator+=(const triple<T> &v)
{
    e[0] += v.e[0]; e[1] += v.e[1]; e[2] += v.e[2];
    return *this;
}

template <typename T>
triple<T>& triple<T>::operator-=(const triple<T> &v)
{
    e[0] -= v.e[0]; e[1] -= v.e[1]; e[2] -= v.e[2];
    return *this;
}

template <typename T>
triple<T>& triple<T>::operator*=(T scalar)
{
    e[0] *= scalar; e[1] *= scalar; e[2] *= scalar;
    return *this;
}

template <typename T>
triple<T>& triple<T>::operator/=(T scalar)
{
    assert(scalar != 0);
    float inv = 1.f/scalar;
    e[0] *= inv; e[1] *= inv; e[2] *= inv;
    return *this;
}

template <typename T>
T triple<T>::length() const
{ return sqrt(e[0]*e[0] + e[1]*e[1] + e[2]*e[2]); }

template <typename T>
T triple<T>::squaredlength() const
{ return (e[0]*e[0] + e[1]*e[1] + e[2]*e[2]); }

template <typename T>
void triple<T>::normalize()
{ *this = *this / (*this).length();}

template <typename T>
triple<T> unitVector(const triple<T>& v)
{
    T length  = v.length();
    return v / length;
}

template <typename T>
triple<T> crossProduct(const triple<T>& v1, const triple<T>& v2)
{
    triple<T> tmp;
    tmp.e[0] = v1.Y() * v2.Z() - v1.Z() * v2.Y();
    tmp.e[1] = v1.Z() * v2.X() - v1.X() * v2.Z();
    tmp.e[2] = v1.X() * v2.Y() - v1.Y() * v2.X();
    return tmp;
}

template <typename T>
T dotProduct(const triple<T>& v1, const triple<T>& v2)
{ return v1.X()*v2.X() + v1.Y()*v2.Y() + v1.Z()*v2.Z(); }

template <typename T>
T tripleProduct(const triple<T>& v1,const triple<T>& v2,const triple<T>& v3)
{
    return dotProduct(( crossProduct(v1, v2)), v3);
}
