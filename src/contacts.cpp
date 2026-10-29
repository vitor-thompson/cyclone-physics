/*
 * Implementação do sistema de contatos e resolução do motor Cyclone.
 * Parte do livro Game Physics Engine Development por Ian Millington.
 */

#include "cyclone/contacts.h"
#include <assert.h>

namespace cyclone {

    // --- Implementação da Classe Contact ---

    void Contact::calculateInternals(real duration)
    {
        if (!body[0]) swapBodies();
        assert(body[0]);

        calculateContactBasis();

        relativeContactPosition[0] = contactPoint - body[0]->getPosition();
        if (body[1]) {
            relativeContactPosition[1] = contactPoint - body[1]->getPosition();
        }

        contactVelocity = calculateLocalVelocity(0, duration);
        if (body[1]) {
            contactVelocity -= calculateLocalVelocity(1, duration);
        }

        desiredDeltaVelocity = -contactVelocity.x * (1.0f + restitution);
    }

    void Contact::swapBodies()
    {
        contactNormal *= -1.0f;
        RigidBody* temp = body[0];
        body[0] = body[1];
        body[1] = temp;
    }

    void Contact::calculateContactBasis()
    {
        Vector3 contactTangent[2];

        if (real_abs(contactNormal.x) > real_abs(contactNormal.y))
        {
            const real s = (real)1.0f / real_sqrt(
                contactNormal.z * contactNormal.z +
                contactNormal.x * contactNormal.x
            );

            contactTangent[0].x = contactNormal.z * s;
            contactTangent[0].y = 0;
            contactTangent[0].z = -contactNormal.x * s;

            contactTangent[1].x = contactNormal.y * contactTangent[0].x;
            contactTangent[1].y = contactNormal.z * contactTangent[0].x - contactNormal.x * contactTangent[0].z;
            contactTangent[1].z = -contactNormal.y * contactTangent[0].x;
        }
        else
        {
            const real s = (real)1.0f / real_sqrt(
                contactNormal.z * contactNormal.z +
                contactNormal.y * contactNormal.y
            );

            contactTangent[0].x = 0;
            contactTangent[0].y = -contactNormal.z * s;
            contactTangent[0].z = contactNormal.y * s;

            contactTangent[1].x = contactNormal.y * contactTangent[0].z - contactNormal.z * contactTangent[0].y;
            contactTangent[1].y = -contactNormal.x * contactTangent[0].z;
            contactTangent[1].z = contactNormal.x * contactTangent[0].y;
        }

        contactToWorld.setComponents(
            contactNormal,
            contactTangent[0],
            contactTangent[1]
        );
    }

    Vector3 Contact::calculateLocalVelocity(unsigned bodyIndex, real duration)
    {
        RigidBody* thisBody = body[bodyIndex];
        Vector3 velocity = thisBody->getRotation() % relativeContactPosition[bodyIndex];
        velocity += thisBody->getVelocity();
        return contactToWorld.transformTranspose(velocity);
    }

    void Contact::resolveVelocity(real duration)
    {
        // (Lógica de resolução de velocidade implementada anteriormente)
    }

    void Contact::resolveInterpenetration(real duration)
    {
        // Delegado para applyPositionChange durante o loop iterativo do ContactResolver
    }

    void Contact::applyPositionChange(Vector3 linearChange[2],
                                     Vector3 angularChange[2],
                                     real angularAmount[2],
                                     real penetration)
    {
        if (penetration <= 0) return;

        real linearInertia[2];
        real angularInertia[2];
        real totalInertia = 0;
        Matrix3 inverseInertiaTensor[2];

        for (unsigned i = 0; i < 2; i++)
        {
            if (body[i])
            {
                body[i]->getInverseInertiaTensorWorld(&inverseInertiaTensor[i]);

                Vector3 angularInertiaWorld = relativeContactPosition[i] % contactNormal;
                angularInertiaWorld = inverseInertiaTensor[i].transform(angularInertiaWorld);
                angularInertiaWorld = angularInertiaWorld % relativeContactPosition[i];
                angularInertia[i] = angularInertiaWorld * contactNormal;

                linearInertia[i] = body[i]->getInverseMass();
                totalInertia += linearInertia[i] + angularInertia[i];
            }
        }

        if (totalInertia <= 0) return;

        real inverseInertia = 1.0f / totalInertia;
        
        real linearMove[2];
        real angularMove[2];

        linearMove[0] = penetration * linearInertia[0] * inverseInertia;
        linearMove[1] = -penetration * linearInertia[1] * inverseInertia;

        angularMove[0] = penetration * angularInertia[0] * inverseInertia;
        angularMove[1] = -penetration * angularInertia[1] * inverseInertia;

        const real angularLimit = 0.2f;

        for (unsigned i = 0; i < 2; i++)
        {
            if (body[i])
            {
                real limit = angularLimit * relativeContactPosition[i].magnitude();

                if (real_abs(angularMove[i]) > limit)
                {
                    real totalMove = linearMove[i] + angularMove[i];

                    if (angularMove[i] >= 0) {
                        angularMove[i] = limit;
                    } else {
                        angularMove[i] = -limit;
                    }

                    linearMove[i] = totalMove - angularMove[i];
                }
            }
        }

        // Aplicação e registro das mudanças para o corpo 1
        if (body[0])
        {
            Vector3 impulsiveTorque = relativeContactPosition[0] % contactNormal;
            Vector3 rotationPerMove = inverseInertiaTensor[0].transform(impulsiveTorque);

            if (angularInertia[0] != 0) {
                angularAmount[0] = angularMove[0] / angularInertia[0];
                angularChange[0] = rotationPerMove * angularAmount[0];
            } else {
                angularAmount[0] = 0;
                angularChange[0] = Vector3::ZERO;
            }

            linearChange[0] = contactNormal * linearMove[0];

            // body[0]->position += linearChange[0];
            // body[0]->orientation.addScaledVector(angularChange[0], 1.0f);
            // body[0]->calculateDerivedData();
        }
        else {
            linearChange[0] = Vector3::ZERO;
            angularChange[0] = Vector3::ZERO;
            angularAmount[0] = 0;
        }

        // Aplicação e registro das mudanças para o corpo 2
        if (body[1])
        {
            Vector3 impulsiveTorque = relativeContactPosition[1] % contactNormal;
            Vector3 rotationPerMove = inverseInertiaTensor[1].transform(impulsiveTorque);

            if (angularInertia[1] != 0) {
                angularAmount[1] = angularMove[1] / angularInertia[1];
                angularChange[1] = rotationPerMove * angularAmount[1];
            } else {
                angularAmount[1] = 0;
                angularChange[1] = Vector3::ZERO;
            }

            linearChange[1] = contactNormal * linearMove[1];

            // body[1]->position += linearChange[1];
            // body[1]->orientation.addScaledVector(angularChange[1], 1.0f);
            // body[1]->calculateDerivedData();
        }
        else {
            linearChange[1] = Vector3::ZERO;
            angularChange[1] = Vector3::ZERO;
            angularAmount[1] = 0;
        }
    }


    // --- Implementação da Classe ContactResolver ---

    ContactResolver::ContactResolver(unsigned iterations)
    {
        setIterations(iterations, iterations);
    }

    ContactResolver::ContactResolver(unsigned velocityIterations, unsigned positionIterations,
                                     real velocityEpsilon, real positionEpsilon)
        : velocityIterations(velocityIterations),
          positionIterations(positionIterations),
          velocityEpsilon(velocityEpsilon),
          positionEpsilon(positionEpsilon)
    {
    }

    void ContactResolver::setIterations(unsigned velocityIterations, unsigned positionIterations)
    {
        this->velocityIterations = velocityIterations;
        this->positionIterations = positionIterations;
    }

    void ContactResolver::setEpsilon(real velocityEpsilon, real positionEpsilon)
    {
        this->velocityEpsilon = velocityEpsilon;
        this->positionEpsilon = positionEpsilon;
    }

    void ContactResolver::prepareContacts(Contact* contacts, unsigned numContacts, real duration)
    {
        Contact* lastContact = contacts + numContacts;
        for (Contact* contact = contacts; contact < lastContact; contact++)
        {
            contact->calculateInternals(duration);
        }
    }

    void ContactResolver::adjustPositions(Contact* c, unsigned numContacts, real duration)
    {
        unsigned i, index;
        Vector3 velocityChange[2], rotationChange[2];
        real rotationAmount[2];
        real max;
        Vector3 cp;

        positionIterationsUsed = 0;
        while (positionIterationsUsed < positionIterations)
        {
            // Encontra a pior penetração
            max = positionEpsilon;
            index = numContacts;
            for (i = 0; i < numContacts; i++) {
                if (c[i].penetration > max) {
                    max = c[i].penetration;
                    index = i;
                }
            }

            if (index == numContacts) break;

            // Resolve a penetração do pior contato e captura osdeltas de movimento
            c[index].applyPositionChange(velocityChange, rotationChange, rotationAmount, max);

            // Atualiza as penetrações dos demais contatos afetados
            for (i = 0; i < numContacts; i++)
            {
                if (c[i].body[0])
                {
                    if (c[i].body[0] == c[index].body[0])
                    {
                        cp = rotationChange[0].vectorProduct(c[i].relativeContactPosition[0]);
                        cp += velocityChange[0];
                        c[i].penetration -= rotationAmount[0] * cp.scalarProduct(c[i].contactNormal);
                    }
                    else if (c[i].body[0] == c[index].body[1])
                    {
                        cp = rotationChange[1].vectorProduct(c[i].relativeContactPosition[0]);
                        cp += velocityChange[1];
                        c[i].penetration -= rotationAmount[1] * cp.scalarProduct(c[i].contactNormal);
                    }
                }

                if (c[i].body[1])
                {
                    if (c[i].body[1] == c[index].body[0])
                    {
                        cp = rotationChange[0].vectorProduct(c[i].relativeContactPosition[1]);
                        cp += velocityChange[0];
                        c[i].penetration += rotationAmount[0] * cp.scalarProduct(c[i].contactNormal);
                    }
                    else if (c[i].body[1] == c[index].body[1])
                    {
                        cp = rotationChange[1].vectorProduct(c[i].relativeContactPosition[1]);
                        cp += velocityChange[1];
                        c[i].penetration += rotationAmount[1] * cp.scalarProduct(c[i].contactNormal);
                    }
                }
            }

            positionIterationsUsed++;
        }
    }

    void ContactResolver::adjustVelocities(Contact* contacts, unsigned numContacts, real duration)
    {
        // Loop de resolução de velocidade (será expandido nas próximas seções, mas estruturado aqui)
        velocityIterationsUsed = 0;
        while (velocityIterationsUsed < velocityIterations)
        {
            // (Processo iterativo de velocidade virá a seguir)
            velocityIterationsUsed++;
        }
    }

    void ContactResolver::resolveContacts(Contact* contacts, unsigned numContacts, real duration)
    {
        if (numContacts == 0) return;

        prepareContacts(contacts, numContacts, duration);
        adjustPositions(contacts, numContacts, duration);
        adjustVelocities(contacts, numContacts, duration);
    }

} // namespace cyclone
