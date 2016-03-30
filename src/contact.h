#ifndef CONTACT_H
#define CONTACT_H

#include "rigidbody.h"
#include "triple.h"
class Contact
{
public:
    Contact(RigidBody *a, RigidBody *b, triple<double> p, triple<double> n)
        :a(a),b(b),p(p),n(n){}

    /* Colliding rigid bodies. */
    RigidBody *a, *b;

    triple<double> p;
    triple<double> n;
    triple<double> ea;
    triple<double> eb;

    bool vf;
};

#endif // CONTACT_H
