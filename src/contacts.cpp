/*
 * Implementação de Contatos e Resolvedor do motor Cyclone.
 * Atualizado com correção em rampas (Cap. 16.1.2), sincronização de sono (Cap. 16.2.1)
 * e margens de erro / epsilons no resolvedor (Cap. 16.2.2).
 */

#include "cyclone/contacts.h"
#include <assert.h>

namespace cyclone {

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
            const real s = (real)1.0f / real_sqrt(contactNormal.z * contactNormal.z + contactNormal.x * contactNormal.x);
            contactTangent[0].x = contactNormal.z * s;
            contactTangent[0].y = 0;
            contactTangent[0].z = -contactNormal.x * s;
            contactTangent[1].x = contactNormal.y * contactTangent[0].x;
            contactTangent[1].y = contactNormal.z * contactTangent[0].x - contactNormal.x * contactTangent[0].z;
            contactTangent[1].z = -contactNormal.y * contactTangent[0].x;
        }
        else
        {
            const real s = (real)1.0f / real_sqrt(contactNormal.z * contactNormal.z + contactNormal.y * contactNormal.y);
            contactTangent[0].x = 0;
            contactTangent[0].y = -contactNormal.z * s;
            contactTangent[0].z = contactNormal.y * s;
            contactTangent[1].x = contactNormal.y * contactTangent[0].z - contactNormal.z * contactTangent[0].y;
            contactTangent[1].y = -contactNormal.x * contactTangent[0].z;
            contactTangent[1].z = -contactNormal.x * contactTangent[0].y;
        }
        contactToWorld.setComponents(contactNormal, contactTangent[0], contactTangent[1]);
    }

    Vector3 Contact::calculateLocalVelocity(unsigned bodyIndex, real duration)
    {
        RigidBody* thisBody = body[bodyIndex];
        Vector3 velocity = thisBody->getRotation() % relativeContactPosition[bodyIndex];
        velocity += thisBody->getVelocity();
        Vector3 localVelocity = contactToWorld.transformTranspose(velocity);

        // CORREÇÃO DE ESTABILIDADE (Cap. 16.1.2): Antecipação de aceleração em planos inclinados
        Vector3 accVelocity = thisBody->getLastFrameAcceleration() * duration;
        accVelocity = contactToWorld.transformTranspose(accVelocity);
        accVelocity.x = 0; // Foco na aceleração tangencial
        localVelocity += accVelocity;

        return localVelocity;
    }

    void Contact::calculateDesiredDeltaVelocity(real duration)
    {
        const real velocityLimit = (real)0.1f;
        real velocityFromAcc = 0;
        if (body[0]) velocityFromAcc += body[0]->getLastFrameAcceleration() * duration * contactNormal;
        if (body[1]) velocityFromAcc -= body[1]->getLastFrameAcceleration() * duration * contactNormal;

        real thisRestitution = restitution;
        if (real_abs(contactVelocity.x) < velocityLimit) {
            thisRestitution = (real)0.0f;
        }

        desiredDeltaVelocity = -contactVelocity.x - thisRestitution * (contactVelocity.x - velocityFromAcc);
    }

    void Contact::matchAwakeState()
    {
        if (!body[1]) return;
        bool body0awake = body[0]->getIsAwake();
        bool body1awake = body[1]->getIsAwake();

        // Sincroniza sono em colisões (Cap. 16.2.1)
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
        Vector3 velKill(desiredDeltaVelocity, -contactVelocity.y, -contactVelocity.z);
        Vector3 impulseContact = impulseMatrix.transform(velKill);

        real planarImpulse = real_sqrt(impulseContact.y * impulseContact.y + impulseContact.z * impulseContact.z);
        if (planarImpulse > impulseContact.x * friction)
        {
            impulseContact.y /= planarImpulse;
            impulseContact.z /= planarImpulse;
            impulseContact.x = deltaVelocity.data[0] + deltaVelocity.data[1] * friction * impulseContact.y + deltaVelocity.data[2] * friction * impulseContact.z;
            impulseContact.x = desiredDeltaVelocity / impulseContact.x;
            impulseContact.y *= friction * impulseContact.x;
            impulseContact.z *= friction * impulseContact.x;
        }

        Vector3 impulse = contactToWorld.transform(impulseContact);

        Vector3 impulsiveTorque = relativeContactPosition[0] % impulse;
        body[0]->setAwake(true);
        body[0]->velocity += impulse * body[0]->getInverseMass();
        body[0]->rotation += inverseInertiaTensor[0].transform(impulsiveTorque);

        if (body[1])
        {
            Vector3 impulseBody2 = impulse * -1.0f;
            Vector3 impulsiveTorque2 = relativeContactPosition[1] % impulseBody2;
            body[1]->setAwake(true);
            body[1]->velocity += impulseBody2 * body[1]->getInverseMass();
            body[1]->rotation += inverseInertiaTensor[1].transform(impulsiveTorque2);
        }
    }

    void Contact::applyPositionChange(Vector3 linearChange[2], Vector3 angularChange[2], real angularAmount[2], real penetration)
    {
        // Implementação padrão de correção de posição...
    }

    // ==========================================
    // CONTACT RESOLVER COM OTIMIZAÇÕES DE EPSILON (Cap. 16.2.2)
    // ==========================================

    ContactResolver::ContactResolver(unsigned iterations)
    {
        setIterations(iterations, iterations);
        setEpsilon((real)0.01f, (real)0.01f);
    }

    ContactResolver::ContactResolver(unsigned velocityIterations, unsigned positionIterations, real velocityEpsilon, real positionEpsilon)
        : velocityIterations(velocityIterations), positionIterations(positionIterations),
          velocityEpsilon(velocityEpsilon), positionEpsilon(positionEpsilon) {}

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
        for (Contact* c = contacts; c < contacts + numContacts; c++) c->calculateInternals(duration);
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
            // OTIMIZAÇÃO (Cap. 16.2.2): Inicializa worstPenetration com positionEpsilon para ignorar micro-ajustes
            max = positionEpsilon;
            index = numContacts;
            for (i = 0; i < numContacts; i++) {
                if (c[i].penetration > max) {
                    max = c[i].penetration;
                    index = i;
                }
            }

            if (index == numContacts) break;

            c[index].matchAwakeState();
            c[index].applyPositionChange(linearChange, angularChange, angularAmount, max);
            // Propagação de ajustes de posição...
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
            // OTIMIZAÇÃO (Cap. 16.2.2): Inicializa com velocityEpsilon
            max = velocityEpsilon;
            index = numContacts;
            for (i = 0; i < numContacts; i++)
            {
                if (c[i].contactVelocity.x < max)
                {
                    max = c[i].contactVelocity.x;
                    index = i;
                }
            }

            if (index == numContacts) break;

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
