/*
 * Interface de detecção fina e geração de contatos do motor Cyclone.
 * Parte do livro Game Physics Engine Development por Ian Millington.
 */

#ifndef CYCLONE_COLLIDE_FINE_H
#define CYCLONE_COLLIDE_FINE_H

#include "core.h"
#include "body.h"
#include "contacts.h"

namespace cyclone {

    /**
     * Estrutura auxiliar contendo informações para o detector registrar 
     * os dados de contato gerados na fase fina.
     */
    struct CollisionData
    {
        /** Apontador para o vetor de contatos onde os resultados serão gravados. */
        Contact *contacts;

        /** Número máximo de contatos que ainda podem ser adicionados. */
        unsigned contactsLeft;

        /** Quantidade total de contatos encontrados até o momento. */
        unsigned contactCount;

        /** Coeficientes globais de fricción e restituição padrão para os contatos. */
        real friction;
        real restitution;
        real tolerance;

        /** Retorna verdadeiro se ainda há espaço no vetor para novos contatos. */
        bool hasMoreContacts() const
        {
            return contactsLeft > 0;
        }

        /** Reinicia os contadores para um novo quadro de simulação. */
        void reset(unsigned maxContacts)
        {
            contactsLeft = maxContacts;
            contactCount = 0;
        }

        /** Avança o ponteiro e atualiza os contadores após registrar novos contatos. */
        void addContacts(unsigned count)
        {
            contactsLeft -= count;
            contactCount += count;
            contacts += count;
        }
    };

    /**
     * Classe base para primitivas geométricas de colisão. Permite posicionar
     * e rotacionar formas de colisão deslocadas do centro de massa do corpo rígido.
     */
    class Primitive
    {
    public:
        /** O corpo rígido associado a esta geometria de colisão. */
        RigidBody *body;

        /** Matriz de transformação local que afasta a primitiva da origem do corpo. */
        Matrix4 offset;

        /** Retorna a matriz de transformação final da primitiva no espaço global do mundo. */
        Matrix4 getTransform() const
        {
            return body->getTransform() * offset;
        }
    };

    /**
     * Representa uma primitiva esférica de colisão.
     */
    class Sphere : public Primitive
    {
    public:
        real radius;
    };

    /**
     * Representa um plano fixo no mundo (ex: chão ou paredes).
     */
    class Plane : public Primitive
    {
    public:
        Vector3 normal;
        real offset;
    };

    /**
     * Representa uma caixa rígida orientada (OBB) para colisão detalhada.
     */
    class Box : public Primitive
    {
    public:
        /** As distâncias do centro até as faces da caixa em cada eixo local. */
        Vector3 halfSize;
    };

    // --- Funções de Detecção e Geração de Contatos entre Primitivas ---

    unsigned collideSphereAndSphere(
        const Sphere &one,
        const Sphere &two,
        CollisionData *data
    );

    unsigned collideSphereAndPlane(
        const Sphere &sphere,
        const Plane &plane,
        CollisionData *data
    );

    unsigned collideBoxAndPlane(
        const Box &box,
        const Plane &plane,
        CollisionData *data
    );

    unsigned collideBoxAndSphere(
        const Box &box,
        const Sphere &sphere,
        CollisionData *data
    );

    unsigned collideBoxAndBox(
        const Box &one,
        const Box &two,
        CollisionData *data
    );

} // namespace cyclone

#endif // CYCLONE_COLLIDE_FINE_H
