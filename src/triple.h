#ifndef TRIPLE_H
#define TRIPLE_H

template <typename T>
class triple
{
public:
    triple<T>() {}
    triple<T>(T e0, T e1, T e2)
    {
        e[0] = e0;
        e[1] = e1;
        e[2] = e2;
    }
    triple<T>(const triple<T> &v) {*this = v;}

    T X() const{ return e[0];}
    T Y() const{ return e[1];}
    T Z() const{ return e[2];}

    void X(T x) {e[0] = x;}
    void Y(T y) {e[1] = y;}
    void Z(T z) {e[2] = z;}

    //define operators
    const triple<T>& operator+() const;
    triple<T> operator-() const;
    T operator[](int i) const {return e[i];}
    T& operator[](int i) {return e[i];}

    friend bool operator==(const triple<T>& v1, const triple<T>& v2);
    friend bool operator!=(const triple<T>& v1, const triple<T>& v2);
    friend triple<T> operator-(const triple<T>& v1, const triple<T>& v2);
    friend triple<T> operator/(const triple<T>& v, T scalar);
    triple<T>& operator-=(const triple& v);
    triple<T>& operator/=(T scalar);

    //triple<T> functions
    T length() const;
    T squaredlength() const;
    void normalize();
    triple<T> add(const triple<T>& v)
    {
        e[0] += v.e[0]; e[1] += v.e[1]; e[2] += v.e[2];
        return *this;
    }
    triple<T> multiply(T scalar)
    {
        e[0] *= scalar; e[1] *= scalar; e[2] *= scalar;
        return *this;
    }

    triple<T> scalarMult(T scalar);
    triple<T> unitVector(const triple<T>& v);
    triple<T> crossProduct(const triple<T>& v1, const triple<T>& v2);
    T dotProduct(const triple<T>& v1, const triple<T>& v2);
    T tripleProduct(const triple<T>& v1,const triple<T>& v2,const triple<T>& v3);

    //data member
    T e[3];
};

#endif // TRIPLE_H
