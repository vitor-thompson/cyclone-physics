/*
 * Implementação do sistema de contatos e resolução do motor Cyclone.
 * Parte do livro Game Physics Engine Development por Ian Millington.
 * Atualizado com suporte completo a fricção isotrópica e matrizes 3D (Capítulo 15).
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
        Matrix3 inverseInertiaTensor[2];
        if (body[0]) body[0]->getInverseInertiaTensorWorld(&inverseInertiaTensor[0]);

        real inverseMass = body[0]->getInverseMass();

        // 1. O equivalente ao produto vetorial em matrizes é a multiplicação por matriz antissimétrica
        Matrix3 impulseToTorque;
        impulseToTorque.setSkewSymmetric(relativeContactPosition[0]);

        // Constrói a matriz para converter impulso de contato em mudança de velocidade no mundo (Corpo 1)
        Matrix3 deltaVelWorld = impulseToTorque;
        deltaVelWorld *= inverseInertiaTensor[0];
        deltaVelWorld *= impulseToTorque;
        deltaVelWorld *= -1;

        // 2. Se houver um segundo corpo, adiciona os dados dele
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

        // 3. Mudança de base para converter em coordenadas de contato
        Matrix3 deltaVelocity = contactToWorld.transpose();
        deltaVelocity *= deltaVelWorld;
        deltaVelocity *= contactToWorld;

        // Adiciona a mudança de velocidade linear (inverso da massa)
        deltaVelocity.data[0] += inverseMass;
        deltaVelocity.data[4] += inverseMass;
        deltaVelocity.data[8] += inverseMass;

        // 4. Inverte para obter o impulso necessário por unidade de velocidade
        Matrix3 impulseMatrix = deltaVelocity.inverse();

        // Encontra as velocidades alvo a zerar (normal + 2 componentes tangenciais de atrito)
        Vector3 velKill(
            desiredDeltaVelocity,
            -contactVelocity.y,
            -contactVelocity.z
        );

        // Encontra o vetor de impulso necessário para atingir as velocidades alvo
        Vector3 impulseContact = impulseMatrix.transform(velKill);

        // 5. Verificação do limite de atrito estático
        real planarImpulse = real_sqrt(
            impulseContact.y * impulseContact.y +
            impulseContact.z * impulseContact.z
        );

        if (planarImpulse > impulseContact.x * friction)
        {
            // O atrito estático foi superado -> Uso de Fricção Dinâmica (Seção 15.4.3)
            impulseContact.y /= planarImpulse;
            impulseContact.z /= planarImpulse;

            impulseContact.x = deltaVelocity.data[0] +
                deltaVelocity.data[1] * friction * impulseContact.y +
                deltaVelocity.data[2] * friction * impulseContact.z;

            impulseContact.x = desiredDeltaVelocity / impulseContact.x;
            impulseContact.y *= friction * impulseContact.x;
            impulseContact.z *= friction * impulseContact.x;
        }

        // 6. Converte o impulso de coordenadas de contato para coordenadas do mundo
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

        // Corpo 1
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

        // Corpo 2
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
          positionIterations
