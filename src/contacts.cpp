/*
 * Implementação do sistema de contatos e resolução do motor Cyclone.
 * Parte do livro Game Physics Engine Development por Ian Millington.
 */

#include "cyclone/contacts.h"

namespace cyclone {

    /**
     * Constrói uma base ortonormal arbitrária para o contato.
     * Armazenada como uma matriz 3x3 onde cada vetor é uma coluna
     * (transforma coordenadas de contato em coordenadas do mundo).
     * O eixo X é gerado a partir da normal do contato, e os eixos Y e Z
     * são definidos de forma perpendicular a ele.
     */
    void Contact::calculateContactBasis()
    {
        Vector3 contactTangent[2];

        // Verifica se o eixo Z está mais próximo do eixo X ou Y do mundo
        if (real_abs(contactNormal.x) > real_abs(contactNormal.y))
        {
            // Fator de escala para garantir normalização
            const real s = (real)1.0f / real_sqrt(
                contactNormal.z * contactNormal.z +
                contactNormal.x * contactNormal.x
            );

            // O novo eixo X (tangente 0) é perpendicular ao eixo Y do mundo
            contactTangent[0].x = contactNormal.z * s;
            contactTangent[0].y = 0;
            contactTangent[0].z = -contactNormal.x * s;

            // O novo eixo Y (tangente 1) é perpendicular aos novos eixos X e Z
            contactTangent[1].x = contactNormal.y * contactTangent[0].x;
            contactTangent[1].y = contactNormal.z * contactTangent[0].x - contactNormal.x * contactTangent[0].z;
            contactTangent[1].z = -contactNormal.y * contactTangent[0].x;
        }
        else
        {
            // Fator de escala para garantir normalização
            const real s = (real)1.0f / real_sqrt(
                contactNormal.z * contactNormal.z +
                contactNormal.y * contactNormal.y
            );

            // O novo eixo X (tangente 0) é perpendicular ao eixo X do mundo
            contactTangent[0].x = 0;
            contactTangent[0].y = -contactNormal.z * s;
            contactTangent[0].z = contactNormal.y * s;

            // O novo eixo Y (tangente 1) é perpendicular aos novos eixos X e Z
            contactTangent[1].x = contactNormal.y * contactTangent[0].z - contactNormal.z * contactTangent[0].y;
            contactTangent[1].y = -contactNormal.x * contactTangent[0].z;
            contactTangent[1].z = contactNormal.x * contactTangent[0].y;
        }

        // Cria a matriz a partir dos três vetores (colunas)
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
        // 1. Calcula a velocidade de fechamento desejada (baseada na restituição)
        // (Será detalhado e expandido nas próximas seções, mas já estruturamos a base)
        
        // Exemplo da lógica cobrindo as seções 14.2.2 a 14.2.5:
        // Tensor de inércia inverso em coordenadas do mundo para cada corpo
        Matrix3 inverseInertiaTensor[2];
        if (body[0]) body[0]->getInverseInertiaTensorWorld(&inverseInertiaTensor[0]);
        if (body[1]) body[1]->getInverseInertiaTensorWorld(&inverseInertiaTensor[1]);

        // Calcula a mudança de velocidade por unidade de impulso para o corpo 1
        Vector3 deltaVelWorld = relativeContactPosition[0] % contactNormal;
        deltaVelWorld = inverseInertiaTensor[0].transform(deltaVelWorld);
        deltaVelWorld = deltaVelWorld % relativeContactPosition[0];
        real deltaVelocity = deltaVelWorld * contactNormal;
        deltaVelocity += body[0]->getInverseMass();

        // Se houver um segundo corpo, adiciona sua contribuição
        if (body[1])
        {
            deltaVelWorld = relativeContactPosition[1] % contactNormal;
            deltaVelWorld = inverseInertiaTensor[1].transform(deltaVelWorld);
            deltaVelWorld = deltaVelWorld % relativeContactPosition[1];
            deltaVelocity += deltaVelWorld * contactNormal;
            deltaVelocity += body[1]->getInverseMass();
        }

        // Calcula a velocidade de fechamento atual do ponto de contato no espaço de contato
        // (Utilizando a rotação e velocidade linear dos corpos)
        Vector3 rotVelocity[2];
        Vector3 contactVelocity;
        
        if (body[0]) {
            rotVelocity[0] = body[0]->getRotation() % relativeContactPosition[0];
            contactVelocity = rotVelocity[0] + body[0]->getVelocity();
        }
        if (body[1]) {
            rotVelocity[1] = body[1]->getRotation() % relativeContactPosition[1];
            contactVelocity += (rotVelocity[1] + body[1]->getVelocity()); // Simplificado para o contexto relativo
        }

        // Transforma a velocidade para o espaço de contato
        Vector3 contactVelocitySpace = contactToWorld.transformTranspose(contactVelocity);

        // Calcula a variação de velocidade desejada usando o coeficiente de restituição
        real desiredDeltaVelocity = -contactVelocitySpace.x * (1.0f + restitution);

        // Calcula o impulso necessário nas coordenadas do contato
        Vector3 impulseContact;
        if (deltaVelocity != 0) {
            impulseContact.x = desiredDeltaVelocity / deltaVelocity;
        } else {
            impulseContact.x = 0;
        }
        impulseContact.y = 0;
        impulseContact.z = 0;

        // Converte o impulso de volta para as coordenadas do mundo
        Vector3 impulse = contactToWorld.transform(impulseContact);

        // (A aplicação física aos corpos será o foco da próxima seção)
    }

} // namespace cyclone
