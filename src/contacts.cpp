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

} // namespace cyclone
