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

    unsigned CollisionDetector::boxAndPlane(
        const Box &box,
        const Plane &plane,
        CollisionData *data
    )
    {
        // Certifica-se de que há espaço no vetor de contatos.
        if (data->contactsLeft <= 0) return 0;

        // Define os 8 vértices locais da caixa com base em halfSize.
        Vector3 vertices[8] = {
            Vector3(-box.halfSize.x, -box.halfSize.y, -box.halfSize.z),
            Vector3(-box.halfSize.x, -box.halfSize.y,  box.halfSize.z),
            Vector3(-box.halfSize.x,  box.halfSize.y, -box.halfSize.z),
            Vector3(-box.halfSize.x,  box.halfSize.y,  box.halfSize.z),
            Vector3( box.halfSize.x, -box.halfSize.y, -box.halfSize.z),
            Vector3( box.halfSize.x, -box.halfSize.y,  box.halfSize.z),
            Vector3( box.halfSize.x,  box.halfSize.y, -box.halfSize.z),
            Vector3( box.halfSize.x,  box.halfSize.y,  box.halfSize.z)
        };

        Matrix4 transform = box.getTransform();
        unsigned contactsUsed = 0;

        // Testa cada vértice individualmente contra o plano.
        for (unsigned i = 0; i < 8; i++)
        {
            // Transforma o vértice para o espaço global do mundo.
            Vector3 vertexPos = transform * vertices[i];

            // Calcula a distância do vértice ao plano.
            real vertexDistance = vertexPos * plane.direction;

            // Se o vértice penetrou o plano (abaixo da superfície), gera um ponto de contato.
            if (vertexDistance <= plane.offset)
            {
                if (contactsUsed >= data->contactsLeft) break;

                Contact *contact = data->contacts + contactsUsed;
                contact->contactPoint = vertexPos;
                contact->contactNormal = plane.direction;
                contact->penetration = plane.offset - vertexDistance;

                contact->body[0] = box.body;
                contact->body[1] = NULL;
                contact->restitution = data->restitution;
                contact->friction = data->friction;

                contactsUsed++;
            }
        }

        return contactsUsed;
    }

    unsigned CollisionDetector::boxAndSphere(
        const Box &box,
        const Sphere &sphere,
        CollisionData *data
    )
    {
        if (data->contactsLeft <= 0) return 0;

        // Transforma o centro da esfera para as coordenadas locais da caixa.
        Vector3 center = sphere.getAxis(3);
        Matrix4 transform = box.getTransform();
        Vector3 relCenter = transform.transformInverse(center);

        // Teste rápido de eixo separador (early-out).
        if (real_abs(relCenter.x) - sphere.radius > box.halfSize.x ||
            real_abs(relCenter.y) - sphere.radius > box.halfSize.y ||
            real_abs(relCenter.z) - sphere.radius > box.halfSize.z)
        {
            return 0;
        }

        Vector3 closestPt(0, 0, 0);
        real dist;

        // Limita (clamp) cada componente às dimensões da caixa.
        dist = relCenter.x;
        if (dist > box.halfSize.x) dist = box.halfSize.x;
        if (dist < -box.halfSize.x) dist = -box.halfSize.x;
        closestPt.x = dist;

        dist = relCenter.y;
        if (dist > box.halfSize.y) dist = box.halfSize.y;
        if (dist < -box.halfSize.y) dist = -box.halfSize.y;
        closestPt.y = dist;

        dist = relCenter.z;
        if (dist > box.halfSize.z) dist = box.halfSize.z;
        if (dist < -box.halfSize.z) dist = -box.halfSize.z;
        closestPt.z = dist;

        // Verifica se a distância ao ponto mais próximo está dentro do raio.
        dist = (closestPt - relCenter).squareMagnitude();
        if (dist > sphere.radius * sphere.radius) return 0;

        // Compila o contato em coordenadas do mundo.
        Vector3 closestPtWorld = transform.transform(closestPt);

        Contact* contact = data->contacts;
        contact->contactNormal = center - closestPtWorld;
        contact->contactNormal.normalize();
        contact->contactPoint = closestPtWorld;
        contact->penetration = sphere.radius - real_sqrt(dist);

        // Associa os corpos e propriedades do material.
        contact->body[0] = box.body;
        contact->body[1] = sphere.body;
        contact->restitution = data->restitution;
        contact->friction = data->friction;

        return 1;
    }

    unsigned CollisionDetector::boxAndPoint(
        const Box &box,
        const Vector3 &point,
        CollisionData *data
    )
    {
        if (data->contactsLeft <= 0) return 0;

        // Transforma o ponto para as coordenadas locais da caixa.
        Matrix4 transform = box.getTransform();
        Vector3 relPt = transform.transformInverse(point);
        Vector3 normal;

        // Verifica cada eixo buscando a penetração menos profunda.
        real min_depth = box.halfSize.x - real_abs(relPt.x);
        if (min_depth < 0) return 0;
        normal = box.getAxis(0) * ((relPt.x < 0) ? -1 : 1);

        real depth = box.halfSize.y - real_abs(relPt.y);
        if (depth < 0) return 0;
        else if (depth < min_depth)
        {
            min_depth = depth;
            normal = box.getAxis(1) * ((relPt.y < 0) ? -1 : 1);
        }

        depth = box.halfSize.z - real_abs(relPt.z);
        if (depth < 0) return 0;
        else if (depth < min_depth)
        {
            min_depth = depth;
            normal = box.getAxis(2) * ((relPt.z < 0) ? -1 : 1);
        }

        Contact* contact = data->contacts;
        contact->contactNormal = normal;
        contact->contactPoint = point;
        contact->penetration = min_depth;

        contact->body[0] = box.body;
        contact->body[1] = NULL;
        contact->restitution = data->restitution;
        contact->friction = data->friction;

        return 1;
    }

    // Função de colisão entre caixas (será detalhada nas próximas implementações)
    unsigned CollisionDetector::boxAndBox(const Box &/*one*/, const Box &/*two*/, CollisionData *__restrict__ /*data*/)
    {
        return 0;
    }

} // namespace cyclone
