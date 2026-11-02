/*
 * collide_coarse.h
 * 
 * Este arquivo define o sistema de detecção ampla de colisão (broad phase) e a 
 * Hierarquia de Volumes Envolventes (BVH - Bounding Volume Hierarchy) baseada em 
 * Esferas Envolventes (Bounding Spheres) no motor físico Cyclone.
 * 
 * Parte da biblioteca física Cyclone.
 */

#ifndef CYCLONE_COLLIDE_COARSE_H
#define CYCLONE_COLLIDE_COARSE_H

#include "core.h"
#include "body.h"
#include <vector>
#include <algorithm>

namespace cyclone {

/**
 * @brief Representa uma esfera envolvente (Bounding Sphere) para testes rápidos de colisão espacial.
 */
class BoundingSphere {
public:
    Vector3 centre; // Centro da esfera no espaço do mundo
    real radius;    // Raio da esfera envolvente

public:
    /**
     * @brief Construtor padrão da esfera envolvente.
     */
    BoundingSphere(const Vector3 &centre, real radius);

    /**
     * @brief Construtor que cria uma esfera envolvente combinando duas outras esferas.
     */
    BoundingSphere(const BoundingSphere &one, const BoundingSphere &two);

    /**
     * @brief Verifica se esta esfera intersecta outra esfera envolvente.
     */
    bool overlaps(const BoundingSphere *other) const;

    /**
     * @brief Retorna o volume (aproximado pelo cubo do raio) útil para heurísticas de árvore BVH.
     */
    real getGrowth(const BoundingSphere &other) const;

    /**
     * @brief Retorna o tamanho da esfera (raio).
     */
    real getSize() const {
        return radius;
    }
};

/**
 * @brief Nó genérico de uma árvore de hierarquia de volumes envolventes (BVH).
 * 
 * @tparam BoundingVolume Tipo do volume envolvente (ex: BoundingSphere).
 */
template <class BoundingVolume>
class BVHNode {
public:
    BVHNode *parent;             // Nó pai na árvore (nullptr se for a raiz)
    BVHNode *children[2];        // Filhos esquerdo e direito
    BoundingVolume volume;       // Volume envolvente que engloba este nó e sua subárvore
    RigidBody *body;             // Corpo rígido associado (apenas folhas possuem corpo; nós internos possuem nullptr)

    /**
     * @brief Construtor de um nó BVH.
     */
    BVHNode(BVHNode *parent, BoundingVolume volume, RigidBody *body = nullptr)
        : parent(parent), volume(volume), body(body) {
        children[0] = nullptr;
        children[1] = nullptr;
    }

    ~BVHNode() {
        // Libera os filhos recursivamente
        delete children[0];
        delete children[1];
    }

    /**
     * @brief Retorna true se este nó for uma folha da árvore.
     */
    bool isLeaf() const {
        return body != nullptr;
    }

    /**
     * @brief Insere um novo corpo rígido com seu volume envolvente na árvore BVH.
     */
    void insert(RigidBody *newBody, const BoundingVolume &newVolume);

    /**
     * @brief Remove e atualiza a árvore quando um corpo é destruído ou atualizado.
     */
    void remove();
};

/**
 * @brief Estrutura auxiliar para armazenar potenciais pares de colisões detectados na fase ampla.
 */
struct PotentialContact {
    RigidBody *body[2];
};

/**
 * @brief Realiza a travessia de duas árvores BVH (ou uma árvore contra ela mesma) para encontrar contatos potenciais.
 * 
 * @tparam BoundingVolume Tipo do volume envolvente.
 * @param nodeOne Primeiro nó da árvore a ser testado.
 * @param nodeTwo Segundo nó da árvore a ser testado.
 * @param contacts Ponteiro para o array de contatos potenciais.
 * @param limit Limite máximo de contatos que podem ser gerados.
 * @return O número de pares potenciais encontrados.
 */
template <class BoundingVolume>
unsigned generatePotentialContacts(
    const BVHNode<BoundingVolume> *nodeOne,
    const BVHNode<BoundingVolume> *nodeTwo,
    PotentialContact *contacts,
    unsigned limit
) {
    // Verifica se os volumes envolventes se sobrepõem
    if (!nodeOne->volume.overlaps(&nodeTwo->volume)) {
        return 0;
    }

    // Se ambos os nós forem folhas, temos um par de corpos candidatos a colisão fina
    if (nodeOne->isLeaf() && nodeTwo->isLeaf()) {
        if (limit > 0) {
            contacts->body[0] = nodeOne->body;
            contacts->body[1] = nodeTwo->body;
            return 1;
        }
        return 0;
    }

    // Se o nó um não for folha e o dois for (ou se o um for maior), desce na árvore do nó um
    if (nodeOne->isLeaf() || (!nodeTwo->isLeaf() && nodeOne->volume.getSize() < nodeTwo->volume.getSize())) {
        unsigned count = generatePotentialContacts(nodeOne, nodeTwo->children[0], contacts, limit);
        if (limit > count) {
            count += generatePotentialContacts(nodeOne, nodeTwo->children[1], contacts + count, limit - count);
        }
        return count;
    } else {
        unsigned count = generatePotentialContacts(nodeOne->children[0], nodeTwo, contacts, limit);
        if (limit > count) {
            count += generatePotentialContacts(nodeOne->children[1], nodeTwo, contacts + count, limit - count);
        }
        return count;
    }
}

} // namespace cyclone

#endif // CYCLONE_COLLIDE_COARSE_H
