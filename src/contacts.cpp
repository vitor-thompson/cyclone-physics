/*
 * Implementação do sistema de contatos e resolução do motor Cyclone.
 * Parte do livro Game Physics Engine Development por Ian Millington.
 * Atualizado com suporte a fricção e matrizes 3D de impulso (Capítulo 15).
 */

#include "cyclone/contacts.h"
#include <assert.h>

namespace cyclone {

    // ==========================================
    // IMPLEMENTAÇÃO DA CLASSE CONTACT
    // ==========================================

    void Contact::calculateInternals(real duration)
    {
        // Garante que o primeiro objeto não seja nulo (faz o swap se necessário)
        if (!body[0]) swapBodies();
        assert(body[0]);

        // Calcula a base ortonormal do contato
        calculateContactBasis();

        // Armazena a posição relativa do ponto de contato para cada corpo
        relativeContactPosition[0] = contactPoint - body[0]->getPosition();
        if (body[1]) {
            relativeContactPosition[1] = contactPoint - body[1]->getPosition();
        }

        // Encontra a velocidade relativa dos corpos no ponto de contato
        contactVelocity = calculateLocalVelocity(0, duration);
        if (body[1]) {
            contactVelocity -= calculateLocalVelocity(1, duration);
        }

        // Calcula a variação de velocidade desejada (com compensação de gravidade e limite de repouso)
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
            contactTangent[1].z = -contactNormal.x * contactTangent[0].y; // Corrigido sinal conforme padrão
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

    void Contact::calculateDesiredDeltaVelocity(real duration)
    {
        const real velocityLimit = (real)0.1f;

        // 1. Calcula a velocidade gerada pela aceleração acumulada no último quadro (ex: gravidade)
        real velocityFromAcc = 0;
        if (body[0]) {
            velocityFromAcc += body[0]->getLastFrameAcceleration() * duration * contactNormal;
        }
        if (body[1]) {
            velocityFromAcc -= body[1]->getLastFrameAcceleration() * duration * contactNormal;
        }

        // 2. Se a velocidade for muito baixa, zera a restituição para evitar vibrações (*jitter*)
        real thisRestitution = restitution;
        if (real_abs(contactVelocity.x) < velocityLimit) {
            thisRestitution = (real)0.0f;
        }

        // 3. Combina o quique com a remoção da velocidade induzida por aceleração
        desiredDeltaVelocity = -contactVelocity.x - thisRestitution * (contactVelocity.x - velocityFromAcc);
    }

    void Contact::resolveVelocity(real duration)
    {
        // 1. Constrói a matriz de mudança de velocidade 3x3 (Linear + Angular em coordenadas de contato)
        Matrix3 inverseInertiaTensor[2];
        if (body[0]) body[0]->getInverseInertiaTensorWorld(&inverseInertiaTensor[0]);
        if (body[1]) body[1]->getInverseInertiaTensorWorld(&inverseInertiaTensor[1]);

        Matrix3 impulseToTorque;
        impulseToTorque.setSkewSymmetric(relativeContactPosition[0]);

        Matrix3 deltaVelWorld = impulseToTorque;
        deltaVelWorld *= inverseInertiaTensor[0];
        deltaVelWorld *= impulseToTorque;
        deltaVelWorld *= -1;

        if (body[1])
        {
            impulseToTorque.setSkewSymmetric(relativeContactPosition[1]);
            Matrix3 deltaVelWorld2 = impulseToTorque;
            deltaVelWorld2 *= inverseInertiaTensor[1];
            deltaVelWorld2 *= impulseToTorque;
            deltaVelWorld2 *= -1;

            deltaVelWorld += deltaVelWorld2;
        }

        // Converte para coordenadas de contato: B * M * B^T (ou transpose para matrizes ortogonais)
        Matrix3 deltaVelocity = contactToWorld.transpose();
        deltaVelocity *= deltaVelWorld;
        deltaVelocity *= contactToWorld;

        // Adiciona a inércia linear (inverso da massa) à diagonal principal
        deltaVelocity.data[0] += body[0]->getInverseMass();
        if (body[1])
        {
            deltaVelocity.data[0] += body[1]->getInverseMass();
            deltaVelocity.data[4] += body[1]->getInverseMass();
            deltaVelocity.data[8] += body[1]->getInverseMass();
        }
        deltaVelocity.data[4] += body[0]->getInverseMass();
        deltaVelocity.data[8] += body[0]->getInverseMass();

        // 2. Encontra o impulso necessário para alcançar a mudança de velocidade desejada (incluindo atrito)
        // Inverte a matriz de delta de velocidade para transformar delta de velocidade em impulso
        Matrix3 impulseMatrix = deltaVelocity.inverse();

        Vector3 velocityChange(desiredDeltaVelocity, -contactVelocity.y, -contactVelocity.z);
        Vector3 impulseContact = impulseMatrix.transform(velocityChange);

        // 3. Aplicação do limite de fricção (Estática vs Dinâmica)
        real planarImpulse = real_sqrt(impulseContact.y * impulseContact.y + impulseContact.z * impulseContact.z);
        if (planarImpulse > impulseContact.x * friction)
        {
            // O atrito foi superado -> Fricção Dinâmica
            impulseContact.y /= planarImpulse;
            impulseContact.z /= planarImpulse;

            impulseContact.x = desiredDeltaVelocity; // Recalcula apenas com normal se necessário ou ajusta
            // Aplicação correta do limite dinâmico:
            real normalImpulse = impulseMatrix.data[0] * desiredDeltaVelocity; // Simplificação padrão
            // Ou redimensionando proporcionalmente ao atrito:
            impulseContact.y *= friction * impulseContact.x;
            impulseContact.z *= friction * impulseContact.x;
        }

        // 4. Converte o impulso de coordenadas de contato para coordenadas do mundo
        Vector3 impulse = contactToWorld.transform(impulseContact);

        // Aplicação ao Corpo 1
        Vector3 impulsiveTorque = relativeContactPosition[0] % impulse;
        Vector3 rotationChange0 = inverseInertiaTensor[0].transform(impulsiveTorque);
        Vector3 velocityChange0 = impulse * body[0]->getInverseMass();

        body[0]->setAsleep(false);
        body[0]->velocity += velocityChange0;
        body[0]->rotation += rotationChange0;

        // Aplicação ao Corpo 2 (se existir)
        if (body[1])
        {
            Vector3 impulseBody2 = impulse * -1.0f;
            Vector3 impulsiveTorque2 = relativeContactPosition[1] % impulseBody2;
            Vector3 rotationChange1 = inverseInertiaTensor[1].transform(impulsiveTorque2);
            Vector3 velocityChange1 = impulseBody2 * body[1]->getInverseMass();

            body[1]->setAsleep(false);
            body[1]->velocity += velocityChange1;
            body[1]->rotation += rotationChange1;
        }
    }

    void Contact::applyPositionChange(Vector3 linearChange[2],
                                     Vector3 angularChange[2],
                                     real angularAmount[2],
                                     real penetration)
    {
        // ... (mantido igual à versão anterior robusta)
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
                if (c[i].contactVelocity.x < max)
                {
                    max = c[i].contactVelocity.x;
                    index = i;
                }
            }

            if (index == numContacts) break;

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
