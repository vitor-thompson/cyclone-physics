/*
 * Implementação dos algoritmos de detecção fina e geração de contatos do motor Cyclone.
 * Parte do livro Game Physics Engine Development por Ian Millington.
 */

#include "cyclone/collide_fine.h"

namespace cyclone {

    unsigned CollisionDetector::sphereAndSphere(
        const Sphere &one,
        const Sphere &two,
        CollisionData *data
    )
    {
        // Certifica-se de que há espaço no vetor de contatos.
        if (data->contactsLeft <= 0) return 0;

        // Obtém as posições globais das esferas.
        Vector3 positionOne = one.getAxis(3);
        Vector3 positionTwo = two.getAxis(3);

        // Calcula o vetor de separação entre os centros.
        Vector3 midline = positionOne - positionTwo;
        real size = midline.magnitude();

        // Verifica se estão se tocando ou sobrepondo.
        if (size <= 0.0f || size >= one.radius + two.radius)
        {
            return 0;
        }

        // Cria a normal e os dados do contato.
        Vector3 normal = midline * (((real)1.0) / size);
        Contact* contact = data->contacts;
        contact->contactNormal = normal;
        contact->contactPoint = positionOne + midline * (real)0.5;
        contact->penetration = (one.radius + two.radius - size);

        // Associa os corpos rígidos e propriedades de material.
        contact->body[0] = one.body;
        contact->body[1] = two.body;
        contact->restitution = data->restitution;
        contact->friction = data->friction;

        return 1;
    }

    unsigned CollisionDetector::sphereAndHalfSpace(
        const Sphere &sphere,
        const Plane &plane,
        CollisionData *data
    )
    {
        if (data->contactsLeft <= 0) return 0;

        Vector3 position = sphere.getAxis(3);

        // Calcula a distância do centro da esfera em relação ao plano.
        real ballDistance =
            plane.direction * position -
            sphere.radius - plane.offset;

        if (ballDistance >= 0) return 0;

        Contact* contact = data->contacts;
        contact->contactNormal = plane.direction;
        contact->penetration = -ballDistance;
        contact->contactPoint =
            position - plane.direction * (ballDistance + sphere.radius);

        contact->body[0] = sphere.body;
        contact->body[1] = NULL; // O plano é estático (sem corpo rígido móvel)
        contact->restitution = data->restitution;
        contact->friction = data->friction;

        return 1;
    }

    unsigned CollisionDetector::sphereAndTruePlane(
        const Sphere &sphere,
        const Plane &plane,
        CollisionData *data
    )
    {
        if (data->contactsLeft <= 0) return 0;

        Vector3 position = sphere.getAxis(3);

        real centerDistance = plane.direction * position - plane.offset;

        if (centerDistance * centerDistance > sphere.radius * sphere.radius)
        {
            return 0;
        }

        Vector3 normal = plane.direction;
        real penetration = -centerDistance;
        if (centerDistance < 0)
        {
            normal *= -1;
            penetration = -penetration;
        }

        penetration += sphere.radius;

        Contact* contact = data->contacts;
        contact->contactNormal = normal;
        contact->penetration = penetration;
        contact->contactPoint = position - plane.direction * centerDistance;

        contact->body[0] = sphere.body;
        contact->body[1] = NULL;
        contact->restitution = data->restitution;
        contact->friction = data->friction;

        return 1;
    }

    // Funções de colisão com caixas (serão implementadas nas próximas seções)
    unsigned CollisionDetector::boxAndPlane(const Box &/*box*/, const Plane &/*plane*/, CollisionData *__restrict__ /*data*/)
    {
        return 0;
    }

    unsigned CollisionDetector::boxAndSphere(const Box &/*box*/, const Sphere &/*sphere*/, CollisionData *__restrict__ /*data*/)
    {
        return 0;
    }

    unsigned CollisionDetector::boxAndBox(const Box &/*one*/, const Box &/*two*/, CollisionData *__restrict__ /*data*/)
    {
        return 0;
    }

} // namespace cyclone
