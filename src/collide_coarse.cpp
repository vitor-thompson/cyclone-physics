/*
 * collide_coarse.cpp
 * 
 * Este arquivo contém a implementação dos métodos de hierarquia de volumes envolventes (BVH)
 * e gestão de esferas envolventes (BoundingSphere e BVHNode) para detecção ampla de colisão
 * (broad phase) no motor físico Cyclone.
 * 
 * Parte da biblioteca física Cyclone.
 */

#include <cyclone/collide_coarse.h>
#include <cmath>
#include <cassert>

namespace cyclone {

/*
 * =========================================================
 * IMPLEMENTAÇÃO DE BOUNDINGSPHERE
 * =========================================================
 */

BoundingSphere::BoundingSphere(const Vector3 &centre, real radius)
    : centre(centre), radius(radius) {}

BoundingSphere::BoundingSphere(const BoundingSphere &one, const BoundingSphere &two) {
    Vector3 centroidDirection = two.centre - one.centre;
    real distanceSq = centroidDirection.squareMagnitude();
    real radiusDiff = two.radius - one.radius;

    // Se uma esfera estiver completamente contida dentro da outra
    if (radiusDiff * radiusDiff >= distanceSq) {
        if (one.radius > two.radius) {
            centre = one.centre;
            radius = one.radius;
        } else {
            centre = two.centre;
            radius = two.radius;
        }
    } else {
        // Caso contrário, calcula a esfera envolvente combinada cobrindo ambas
        real distance = std::sqrt(distanceSq);
        radius = (distance + one.radius + two.radius) * (real)0.5;
        centre = one.centre;
        if (distance > 0.0f) {
            centre += centroidDirection * ((radius - one.radius) / distance);
        }
    }
}

bool BoundingSphere::overlaps(const BoundingSphere *other) const {
    real distanceSq = (centre - other->centre).squareMagnitude();
    real radiusSum = radius + other->radius;
    return distanceSq <= (radiusSum * radiusSum);
}

real BoundingSphere::getGrowth(const BoundingSphere &other) const {
    // Cria uma esfera temporária combinada e calcula a expansão do volume
    BoundingSphere newSphere(*this, other);
    
    // Retorna o aumento volumétrico aproximado (cubo do raio)
    real oldVolume = radius * radius * radius;
    real newVolume = newSphere.radius * newSphere.radius * newSphere.radius;
    return newVolume - oldVolume;
}

/*
 * =========================================================
 * IMPLEMENTAÇÃO DE BVHNODE
 * =========================================================
 */

template <class BoundingVolume>
void BVHNode<BoundingVolume>::insert(RigidBody *newBody, const BoundingVolume &newVolume) {
    // Se o nó atual for uma folha, transformamo-lo em um nó interno com dois filhos
    if (isLeaf()) {
        // O primeiro filho copia os dados antigos do nó atual
        children[0] = new BVHNode<BoundingVolume>(this, volume, body);

        // O segundo filho recebe o novo corpo e volume
        children[1] = new BVHNode<BoundingVolume>(this, newVolume, newBody);

        // Remove o corpo do nó atual, tornando-o um nó interno puro
        body = nullptr;

        // Recalcula o volume envolvente englobando ambos os filhos
        volume = BoundingVolume(children[0]->volume, children[1]->volume);
    } else {
        // Se for um nó interno, escolhe o filho cujo crescimento de volume seja menor
        real growth0 = children[0]->volume.getGrowth(newVolume);
        real growth1 = children[1]->volume.getGrowth(newVolume);

        if (growth0 < growth1) {
            children[0]->insert(newBody, newVolume);
        } else {
            children[1]->insert(newBody, newVolume);
        }

        // Atualiza o volume do nó pai para acomodar a mudança
        volume = BoundingVolume(children[0]->volume, children[1]->volume);
    }
}

template <class BoundingVolume>
void BVHNode<BoundingVolume>::remove() {
    // Se este nó possuir um pai, precisamos ajustar a árvore
    if (parent) {
        // Encontra qual dos filhos do pai é o irmão deste nó
        BVHNode<BoundingVolume> *sibling;
        if (parent->children[0] == this) {
            sibling = parent->children[1];
        } else {
            sibling = parent->children[0];
        }

        // O pai assume os dados do irmão e descarta os ponteiros de filhos
        parent->volume = sibling->volume;
        parent->body = sibling->body;
        parent->children[0] = sibling->children[0];
        parent->children[1] = sibling->children[1];

        // Corrige o ponteiro de pai dos filhos do irmão adotado
        if (parent->children[0]) {
            parent->children[0]->parent = parent;
        }
        if (parent->children[1]) {
            parent->children[1]->parent = parent;
        }

        // Evita que o destrutor destrua o irmão adotado ao deletar o nó atual
        sibling->children[0] = nullptr;
        sibling->children[1] = nullptr;
        delete sibling;

        // Recalcula os volumes subindo recursivamente na árvore
        parent->recalculateBoundingVolume();
    }
}

// Instanciação explícita para o tipo padrão BoundingSphere
template class BVHNode<BoundingSphere>;

} // namespace cyclone
