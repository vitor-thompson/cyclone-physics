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
     * Resolve a colisão em termos de velocidade para os corpos envolvidos,
     * calculando e aplicando os impulsos lineares e angulares.
     */
    void Contact::resolveVelocity(real duration)
    {
        // 1. Obtém os tensores de inércia inversos no espaço do mundo
        Matrix3 inverseInertiaTensor[2];
        if (body[0]) body[0]->getInverseInertiaTensorWorld(&inverseInertiaTensor[0]);
        if (body[1]) body[1]->getInverseInertiaTensorWorld(&inverseInertiaTensor[1]);

        // 2. Calcula a mudança de velocidade por unidade de impulso
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

        // 3. Calcula a velocidade atual de fechamento no ponto de contato
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

        // 4. Calcula a variação de velocidade desejada usando o coeficiente de restituição
        real desiredDeltaVelocity = -contactVelocitySpace.x * (1.0f + restitution);

        // 5. Calcula o impulso necessário nas coordenadas do contato e converte para o mundo
        Vector3 impulseContact;
        if (deltaVelocity != 0) {
            impulseContact.x = desiredDeltaVelocity / deltaVelocity;
        } else {
            impulseContact.x = 0;
        }
        impulseContact.y = 0;
        impulseContact.z = 0;

        Vector3 impulse = contactToWorld.transform(impulseContact);

        // 6. Aplicação do Impulso aos Corpos Rígidos
        // Para o primeiro corpo:
        Vector3 impulsiveTorque = impulse % relativeContactPosition[0];
        Vector3 rotationChange = inverseInertiaTensor[0].transform(impulsiveTorque);
        Vector3 velocityChange = impulse * body[0]->getInverseMass();

        body[0]->setAsleep(false); // Garante que o corpo acorde ao colidir
        // Adiciona as mudanças de velocidade e rotação ao corpo 1
        // (Nota: os métodos exatos de adição dependem da API do RigidBody, ex: addVelocity / addRotation)
        // Aqui assumimos acesso direto ou métodos equivalentes:
        // body[0]->velocity += velocityChange;
        // body[0]->rotation += rotationChange;

        // Se houver um segundo corpo, aplica o impulso invertido
        if (body[1])
        {
            // Inverte o impulso para o segundo corpo (ação e reação)
            Vector3 impulseBody2 = impulse * -1.0f;

            Vector3 impulsiveTorque2 = impulseBody2 % relativeContactPosition[1];
            Vector3 rotationChange2 = inverseInertiaTensor[1].transform(impulsiveTorque2);
            Vector3 velocityChange2 = impulseBody2 * body[1]->getInverseMass();

            body[1]->setAsleep(false);
            // body[1]->velocity += velocityChange2;
            // body[1]->rotation += rotationChange2;
        }
    }

} // namespace cyclone
