/*
 * Definição do sistema de contatos e resolvedor do motor Cyclone.
 * Parte do livro Game Physics Engine Development por Ian Millington.
 */

#ifndef CYCLONE_CONTACTS_H
#define CYCLONE_CONTACTS_H

#include "cyclone/core.h"
#include "cyclone/bodies.h"

namespace cyclone {

    // Declaração antecipada para o relacionamento de amizade
    class ContactResolver;

    /**
     * Representa um ponto de contato entre dois corpos rígidos (ou um corpo e o cenário).
     */
    class Contact
    {
    public:
        // Permite que o ContactResolver acesse os membros internos do contato
        friend class ContactResolver;

        // Corpos rígidos envolvidos no contato (body[1] pode ser nulo se for colisão com o cenário fixo)
        RigidBody* body[2];

        // Coeficiente de restituição (elasticidade) da colisão
        real restitution;

        // Ponto de contato no espaço do mundo
        Vector3 contactPoint;

        // Normal do contato no espaço do mundo
        Vector3 contactNormal;

        // Profundidade da interpenetração
        real penetration;

        // Matriz de transformação que converte o espaço de contato para o espaço do mundo
        Matrix3 contactToWorld;

        // Posição do ponto de contato relativa ao centro de cada corpo
        Vector3 relativeContactPosition[2];

        /**
         * Constrói uma base ortonormal arbitrária para o contato (espaço de contato).
         */
        void calculateContactBasis();

        /**
         * Resolve a colisão em termos de velocidade para os corpos envolvidos.
         */
        void resolveVelocity(real duration);

        /**
         * Resolve a interpenetração entre os corpos usando Projeção Não Linear.
         */
        void resolveInterpenetration(real duration);
    };

    /**
     * O resolvedor de contatos gerencia e processa um conjunto de contatos por quadro.
     */
    class ContactResolver
    {
    public:
        /**
         * Resolve um conjunto de contatos para penetração e velocidade.
         */
        void resolveContacts(Contact* contactArray, unsigned numContacts, real duration);
    };

} // namespace cyclone

#endif // CYCLONE_CONTACTS_H
