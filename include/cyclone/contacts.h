/*
 * Definição do sistema de contatos e resolvedor do motor Cyclone.
 * Parte do livro Game Physics Engine Development por Ian Millington.
 */

#ifndef CYCLONE_CONTACTS_H
#define CYCLONE_CONTACTS_H

#include "cyclone/core.h"
#include "cyclone/bodies.h"

namespace cyclone {

    class ContactResolver;

    class Contact
    {
    public:
        friend class ContactResolver;

        RigidBody* body[2];
        real restitution;
        Vector3 contactPoint;
        Vector3 contactNormal;
        real penetration;

    protected:
        Matrix3 contactToWorld;
        Vector3 contactVelocity;
        real desiredDeltaVelocity;
        Vector3 relativeContactPosition[2];

    public:
        void calculateInternals(real duration);
        void resolveVelocity(real duration);
        void resolveInterpenetration(real duration);

        // Aplica a mudança de posição e preenche os vetores de rastreio para atualização
        void applyPositionChange(Vector3 linearChange[2],
                                 Vector3 angularChange[2],
                                 real angularAmount[2],
                                 real penetration);

    protected:
        void calculateContactBasis();
        void swapBodies();
        Vector3 calculateLocalVelocity(unsigned bodyIndex, real duration);
    };

    class ContactResolver
    {
    protected:
        unsigned velocityIterations;
        unsigned positionIterations;
        real velocityEpsilon;
        real positionEpsilon;

    public:
        unsigned velocityIterationsUsed;
        unsigned positionIterationsUsed;

        // Construtor para definir as iterações e tolerâncias
        ContactResolver(unsigned iterations);
        ContactResolver(unsigned velocityIterations, unsigned positionIterations,
                        real velocityEpsilon = (real)0.01f,
                        real positionEpsilon = (real)0.01f);

        void setIterations(unsigned velocityIterations, unsigned positionIterations);
        void setEpsilon(real velocityEpsilon, real positionEpsilon);

        void resolveContacts(Contact* contactArray, unsigned numContacts, real duration);

    protected:
        void prepareContacts(Contact* contactArray, unsigned numContacts, real duration);
        void adjustPositions(Contact* contacts, unsigned numContacts, real duration);
        void adjustVelocities(Contact* contacts, unsigned numContacts, real duration);
    };

} // namespace cyclone

#endif // CYCLONE_CONTACTS_H
