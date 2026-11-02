/*
 * Definição de Juntas (Joints) para o motor Cyclone.
 * Parte do livro Game Physics Engine Development por Ian Millington.
 * Conectam corpos rígidos limitando posições relativas (Capítulo 17.2.1).
 */

#ifndef CYCLONE_JOINTS_H
#define CYCLONE_JOINTS_H

#include "precision.h"
#include "body.h"
#include "contacts.h"

namespace cyclone {

    class Joint : public ContactGenerator
    {
    public:
        /**
         * Holds the two rigid bodies that are connected by this joint.
         */
        RigidBody* body[2];

        /**
         * Holds the relative location of the connection for each
         * body, given in local coordinates.
         */
        Vector3 position[2];

        /**
         * Holds the maximum displacement at the joint before the
         * joint is considered to be violated. This is normally a
         * small, epsilon value. It can be larger, however, in which
         * case the joint will behave as if an inelastic cable joined
         * the bodies at their joint locations.
         */
        real error;

        /**
         * Generates the contacts required to restore the joint if it
         * has been violated.
         */
        virtual unsigned addContact(Contact *contact, unsigned limit) const;
    };

} // namespace cyclone

#endif // CYCLONE_JOINTS_H
