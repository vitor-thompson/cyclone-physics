/*
 * Implementação do sistema de contatos e resolução do motor Cyclone.
 * Parte do livro Game Physics Engine Development por Ian Millington.
 * Atualizado com suporte a fricção isotrópica, prevenção em rampas (Cap. 16.1.2) 
 * e sincronização de sono em colisões (Cap. 16.2.1).
 */

#include "cyclone/contacts.h"
#include <assert.h>

namespace cyclone {

    // ==========================================
    // IMPLEMENTAÇÃO DA CLASSE CONTACT
    // ==========================================

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

        calculateDesiredDeltaVelocity(duration);
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
            contactTangent[1].z = -contactNormal.x * contactTangent[0].y;
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
        
        // 1. Velocidade padrão do ponto de contato
        Vector3 velocity = thisBody->getRotation() % relativeContactPosition[bodyIndex];
        velocity += thisBody->getVelocity();
        Vector3 localVelocity = contactToWorld.transformTranspose(velocity);

        // 2. CORREÇÃO DE ESTABILIDADE (Capítulo 16.1.2): Antecipação de aceleração em planos inclinados
        Vector3 accVelocity = thisBody->getLastFrameAcceleration() * duration;
        accVelocity = contactToWorld.transformTranspose(accVelocity);
        
        // Remove a componente na direção normal; focamos apenas na aceleração planar (tangencial)
        accVelocity.x = 0;
        localVelocity += accVelocity;

        return localVelocity;
    }

    void Contact::calculateDesiredDeltaVelocity(real duration)
    {
        const real velocityLimit = (real)0.1f;

        real velocityFromAcc = 0;
        if (body[0]) {
            velocityFromAcc += body[0]->getLastFrameAcceleration() * duration * contactNormal;
        }
        if (body[1]) {
            velocityFromAcc -= body[1]->getLastFrameAcceleration() * duration * contactNormal;
        }

        real thisRestitution = restitution;
        if (real_abs(contactVelocity.x) < velocityLimit) {
            thisRestitution = (real)0.0f;
        }

        desiredDeltaVelocity = -contactVelocity.x - thisRestitution * (contactVelocity.x - velocityFromAcc);
    }

    void Contact::matchAwakeState()
    {
        // Contatos com o mundo (sem corpo 2) não acordam objetos
        if (!body[1]) return;

        bool body0awake = body[0]->getIsAwake();
        bool body1awake = body[1]->getIsAwake();

        // Se um está acordado e o outro dormindo, acorda o dormindo (Capítulo 16.2.1)
        if (body0awake ^ body1awake) {
            if (body0awake) body[1]->setAwake(true);
            else body[0]->setAwake(true);
        }
    }

    void Contact::resolveVelocity(real duration)
    {
        Matrix3 inverseInertiaTensor[2];
        if (body[0]) body[0]->getInverseInertiaTensorWorld(&inverseInertiaTensor[0]);

        real inverseMass = body[0]->getInverseMass();

        Matrix3 impulseToTorque;
        impulseToTorque.setSkewSymmetric(relativeContactPosition[0]);

        Matrix3 deltaVelWorld = impulseToTorque;
        deltaVelWorld *= inverseInertiaTensor[0];
        deltaVelWorld *= impulseToTorque;
        deltaVelWorld *= -1;

        if (body[1])
        {
            body[1]->getInverseInertiaTensorWorld(&inverseInertiaTensor[1]);
            impulseToTorque.setSkewSymmetric(relativeContactPosition[1]);

            Matrix3 deltaVelWorld2 = impulseToTorque;
            deltaVelWorld2 *= inverseInertiaTensor[1];
            deltaVelWorld2 *= impulseToTorque;
            deltaVelWorld2 *= -1;

            deltaVelWorld += deltaVelWorld2;
            inverseMass += body[1]->getInverseMass();
        }

        Matrix3 deltaVelocity = contactToWorld.transpose();
        deltaVelocity *= deltaVelWorld;
        deltaVelocity *= contactToWorld;

        deltaVelocity.data[0] += inverseMass;
        deltaVelocity.data[4] += inverseMass;
        deltaVelocity.data[8] += inverseMass;

        Matrix3 impulseMatrix = deltaVelocity.inverse();

        Vector3 velKill(
            desiredDeltaVelocity,
            -contactVelocity.y,
            -contactVelocity.z
        );

        Vector3 impulseContact = impulseMatrix.transform(velKill);

        real planarImpulse = real_sqrt(
            impulseContact.y * impulseContact.y +
            impulseContact.z * impulseContact.z
        );

        if (planarImpulse > impulseContact.x * friction)
        {
            impulseContact.y /= planarImpulse;
            impulseContact.z /= planarImpulse;

            impulseContact.x = deltaVelocity.data[0] +
                deltaVelocity.data[1] * friction * impulseContact.y +
                deltaVelocity.data[2] * friction * impulseContact.z;

            impulseContact.x = desiredDeltaVelocity / impulseContact.x;
            impulseContact.y *= friction * impulseContact.x;
            impulseContact.z *= friction * impulseContact.x;
        }

        Vector3 impulse = contactToWorld.transform(impulseContact);

        // Aplicação ao Corpo 1
        Vector3 impulsiveTorque = relativeContactPosition[0] % impulse;
        Vector3 rotationChange0 = inverseInertiaTensor[0].transform(impulsiveTorque);
        Vector3 velocityChange0 = impulse * body[0]->getInverseMass();

        body[0]->setAwake(true);
        body[0]->velocity += velocityChange0;
        body[0]->rotation += rotationChange0;

        // Aplicação ao Corpo 2
        if (body[1])
        {
            Vector3 impulseBody2 = impulse * -1.0f;
            Vector3 impulsiveTorque2 = relativeContactPosition[1] % impulseBody2;
            Vector3 rotationChange1 = inverseInertiaTensor[1].transform(impulsiveTorque2);
            Vector3 velocityChange1 = impulseBody2 * body[1]->getInverseMass();

            body[1]->setAwake(true);
            body[1]->velocity += velocityChange1;
            body[1]->rotation += rotationChange1;
        }
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
        }
        else {
            linearChange[0] = Vector3::ZERO;
            angularChange[0] = Vector3::ZERO;
            angularAmount[0] = 0;
        }

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
        }
        else {
            linearChange[1] = Vector3::ZERO;
            angularChange[1] = Vector3::ZERO;
            angularAmount[1] = 0;
        }
    }


    // ==========================================
    // IMPLEMENTAÇÃO DA CLASSE CONTACTRESOLVER
    // ==========================================

    ContactResolver::ContactResolver(unsigned iterations)
    {
        setIterations(iterations, iterations);
        setEpsilon((real)0.01f, (real)0.01f);
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
        Vector3 linearChange[2], angularChange[2];
        real angularAmount[2];
        real max;
        Vector3 cp;

        positionIterationsUsed = 0;
        while (positionIterationsUsed < positionIterations)
        {
            max = positionEpsilon;
            index = numContacts;
            for (i = 0; i < numContacts; i++) {
                if (c[i].penetration > max) {
                    max = c[i].penetration;
                    index = i;
                }
            }

            if (index == numContacts) break;

            // Sincroniza o estado de sono antes de resolver a penetração
            c[index].matchAwakeState();

            c[index].applyPositionChange(linearChange, angularChange, angularAmount, max);

            for (i = 0; i < numContacts; i++)
            {
                if (c[i].body[0])
                {
                    if (c[i].body[0] == c[index].body[0])
                    {
                        cp = angularChange[0].vectorProduct(c[i].relativeContactPosition[0]);
                        cp += linearChange[0];
                        c[i].penetration -= angularAmount[0] * cp.scalarProduct(c[i].contactNormal);
                    }
                    else if (c[i].body[0] == c[index].body[1])
                    {
                        cp = angularChange[1].vectorProduct(c[i].relativeContactPosition[0]);
                        cp += linearChange[1];
                        c[i].penetration -= angularAmount[1] * cp.scalarProduct(c[i].contactNormal);
                    }
                }

                if (c[i].body[1])
                {
                    if (c[i].body[1] == c[index].body[0])
                    {
                        cp = angularChange[0].vectorProduct(c[i].relativeContactPosition[1]);
                        cp += linearChange[0];
                        c[i].penetration += angularAmount[0] * cp.scalarProduct(c[i].contactNormal);
                    }
                    else if (c[i].body[1] == c[index].body[1])
                    {
                        cp = angularChange[1].vectorProduct(c[i].relativeContactPosition[1]);
                        cp += linearChange[1];
                        c[i].penetration += angularAmount[1] * cp.scalarProduct(c[i].contactNormal);
                    }
                }
            }

            positionIterationsUsed++;
        }
    }

    void ContactResolver::adjustVelocities(Contact* c, unsigned numContacts, real duration)
    {
        unsigned i, index;
        real max;

        velocityIterationsUsed = 0;
        while (velocityIterationsUsed < velocityIterations)
        {
            max = velocityEpsilon;
            index = numContacts;
            for (i = 0; i < numContacts; i++)
            {
                // Ignora contatos cuja velocidade de fechamento já foi resolvida ou onde ambos dormem
                if (c[i].contactVelocity.x < max)
                {
                    max = c[i].contactVelocity.x;
                    index = i;
                }
            }

            if (index == numContacts) break;

            // Sincroniza o estado de sono antes de resolver a velocidade
            c[index].matchAwakeState();

            c[index].resolveVelocity(duration);

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
