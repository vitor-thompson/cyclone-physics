/*
 * Implementação do sistema de contatos e resolução do motor Cyclone.
 * Parte do livro Game Physics Engine Development por Ian Millington.
 */

#include "cyclone/contacts.h"

namespace cyclone {

    /**
     * Constrói uma base ortonormal arbitrária para o contato.
     */
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

    /**
     * Resolve a colisão em termos de velocidade para os corpos envolvidos.
     */
    void Contact::resolveVelocity(real duration)
    {
        Matrix3 inverseInertiaTensor[2];
        if (body[0]) body[0]->getInverseInertiaTensorWorld(&inverseInertiaTensor[0]);
        if (body[1]) body[1]->getInverseInertiaTensorWorld(&inverseInertiaTensor[1]);

        Vector3 deltaVelWorld = relativeContactPosition[0] % contactNormal;
        deltaVelWorld = inverseInertiaTensor[0].transform(deltaVelWorld);
        deltaVelWorld = deltaVelWorld % relativeContactPosition[0];
        real deltaVelocity = deltaVelWorld * contactNormal;
        deltaVelocity += body[0]->getInverseMass();

        if (body[1])
        {
            deltaVelWorld = relativeContactPosition[1] % contactNormal;
            deltaVelWorld = inverseInertiaTensor[1].transform(deltaVelWorld);
            deltaVelWorld = deltaVelWorld % relativeContactPosition[1];
            deltaVelocity += deltaVelWorld * contactNormal;
            deltaVelocity += body[1]->getInverseMass();
        }

        Vector3 rotVelocity[2];
        Vector3 contactVelocity;
        
        if (body[0]) {
            rotVelocity[0] = body[0]->getRotation() % relativeContactPosition[0];
            contactVelocity = rotVelocity[0] + body[0]->getVelocity();
        }
        if (body[1]) {
            rotVelocity[1] = body[1]->getRotation() % relativeContactPosition[1];
            contactVelocity += (rotVelocity[1] + body[1]->getVelocity());
        }

        Vector3 contactVelocitySpace = contactToWorld.transformTranspose(contactVelocity);
        real desiredDeltaVelocity = -contactVelocitySpace.x * (1.0f + restitution);

        Vector3 impulseContact;
        if (deltaVelocity != 0) {
            impulseContact.x = desiredDeltaVelocity / deltaVelocity;
        } else {
            impulseContact.x = 0;
        }
        impulseContact.y = 0;
        impulseContact.z = 0;

        Vector3 impulse = contactToWorld.transform(impulseContact);

        // Aplicação do Impulso ao Corpo 1
        Vector3 impulsiveTorque = impulse % relativeContactPosition[0];
        Vector3 rotationChange = inverseInertiaTensor[0].transform(impulsiveTorque);
        Vector3 velocityChange = impulse * body[0]->getInverseMass();

        body[0]->setAsleep(false);
        // body[0]->velocity += velocityChange;
        // body[0]->rotation += rotationChange;

        // Aplicação do Impulso ao Corpo 2 (se existir)
        if (body[1])
        {
            Vector3 impulseBody2 = impulse * -1.0f;
            Vector3 impulsiveTorque2 = impulseBody2 % relativeContactPosition[1];
            Vector3 rotationChange2 = inverseInertiaTensor[1].transform(impulsiveTorque2);
            Vector3 velocityChange2 = impulseBody2 * body[1]->getInverseMass();

            body[1]->setAsleep(false);
            // body[1]->velocity += velocityChange2;
            // body[1]->rotation += rotationChange2;
        }
    }

    /**
     * Resolve a interpenetração entre os corpos usando Projeção Não Linear.
     */
    void Contact::resolveInterpenetration(real duration)
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

        // Limite de rotação excessiva para evitar instabilidades
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

        // Aplicação do Movimento ao Corpo 1
        if (body[0])
        {
            // body[0]->position += contactNormal * linearMove[0];
            Vector3 impulsiveTorque = relativeContactPosition[0] % contactNormal;
            Vector3 rotationPerMove = inverseInertiaTensor[0].transform(impulsiveTorque);
            if (angularInertia[0] != 0) {
                Vector3 rotationChange = rotationPerMove * (angularMove[0] / angularInertia[0]);
                // body[0]->orientation.addScaledVector(rotationChange, 1.0f);
                // body[0]->calculateDerivedData();
            }
        }

        // Aplicação do Movimento ao Corpo 2 (se existir)
        if (body[1])
        {
            // body[1]->position += contactNormal * linearMove[1];
            Vector3 impulsiveTorque = relativeContactPosition[1] % contactNormal;
            Vector3 rotationPerMove = inverseInertiaTensor[1].transform(impulsiveTorque);
            if (angularInertia[1] != 0) {
                Vector3 rotationChange = rotationPerMove * (angularMove[1] / angularInertia[1]);
                // body[1]->orientation.addScaledVector(rotationChange, 1.0f);
                // body[1]->calculateDerivedData();
            }
        }
    }

    /**
     * Executa o pipeline completo de resolução para um conjunto de contatos.
     */
    void ContactResolver::resolveContacts(Contact* contacts, unsigned numContacts, real duration)
    {
        if (numContacts == 0) return;

        // 1. Resolução de Interpenetração (Posições)
        for (unsigned i = 0; i < numContacts; i++) {
            contacts[i].resolveInterpenetration(duration);
        }

        // 2. Resolução de Velocidade (Impulsos)
        for (unsigned i = 0; i < numContacts; i++) {
            contacts[i].resolveVelocity(duration);
        }
    }

} // namespace cyclone
