#pragma once

#include <cyclone/body.h>

namespace cyclone {

    /**
     * @brief Armazena um par de corpos rígidos que podem estar em contato (potencial colisão).
     */
    struct PotentialContact
    {
        RigidBody* body[2];
    };

    /**
     * @brief Representa uma esfera envolvente (bounding sphere) usada para testes rápidos de sobreposição.
     */
    struct BoundingSphere
    {
        Vector3 center;
        real radius;

    public:
        /** Cria uma nova esfera envolvente com o centro e raio dados. */
        BoundingSphere(const Vector3 &center, real radius);

        /** Cria uma esfera envolvente que engloba outras duas esferas. */
        BoundingSphere(const BoundingSphere &one, const BoundingSphere &two);

        /** Verifica se esta esfera se sobrepõe a outra. */
        bool overlaps(const BoundingSphere *other) const;

        /** Calcula o quanto a esfera precisaria crescer para englobar uma nova esfera. */
        real getGrowth(const BoundingSphere &other) const;
    };

    /**
     * @brief Nó base em uma Hierarquia de Volumes Envolventes (BVH) usando árvore binária.
     */
    template<class BoundingVolumeClass>
    class BVHNode
    {
    public:
        /** Nós filhos desta árvore binária (até 2). */
        BVHNode *children[2];

        /** Ponteiro para o nó pai. */
        BVHNode *parent;

        /** Volume envolvente que engloba todos os descendentes deste nó. */
        BoundingVolumeClass volume;

        /** Corpo rígido associado (apenas nós folha possuem um corpo válido). */
        RigidBody *body;

        /** Construtor para nós internos ou raiz. */
        BVHNode(BVHNode *parent, const BoundingVolumeClass &volume, RigidBody *body = nullptr)
            : parent(parent), volume(volume), body(body)
        {
            children[0] = nullptr;
            children[1] = nullptr;
        }

        /** Destruidor limpa os nós filhos e reorganiza a árvore ao remover elementos. */
        ~BVHNode()
        {
            if (parent)
            {
                BVHNode<BoundingVolumeClass> *sibling;
                if (parent->children[0] == this) sibling = parent->children[1];
                else sibling = parent->children[0];

                parent->volume = sibling->volume;
                parent->body = sibling->body;
                parent->children[0] = sibling->children[0];
                parent->children[1] = sibling->children[1];

                if (parent->children[0]) parent->children[0]->parent = parent;
                if (parent->children[1]) parent->children[1]->parent = parent;

                sibling->parent = nullptr;
                sibling->body = nullptr;
                sibling->children[0] = nullptr;
                sibling->children[1] = nullptr;
                delete sibling;

                parent->recalculateBoundingVolume();
            }

            if (children[0]) {
                children[0]->parent = nullptr;
                delete children[0];
            }
            if (children[1]) {
                children[1]->parent = nullptr;
                delete children[1];
            }
        }

        /** Verifica se este nó é uma folha (contém um corpo físico). */
        bool isLeaf() const
        {
            return (body != nullptr);
        }

        /** Insere um novo corpo rígido e seu volume na hierarquia em tempo de execução. */
        void insert(RigidBody* newBody, const BoundingVolumeClass &newVolume)
        {
            if (isLeaf())
            {
                // Transforma o nó atual em um galho, criando dois filhos
                children[0] = new BVHNode<BoundingVolumeClass>(this, volume, body);
                children[1] = new BVHNode<BoundingVolumeClass>(this, newVolume, newBody);

                this->body = nullptr;
                recalculateBoundingVolume();
            }
            else
            {
                // Escolhe o filho cujo volume cresce menos com o novo objeto
                if (children[0]->volume.getGrowth(newVolume) < children[1]->volume.getGrowth(newVolume))
                {
                    children[0]->insert(newBody, newVolume);
                }
                else
                {
                    children[1]->insert(newBody, newVolume);
                }
            }
        }

        /** Coleta potenciais contatos a partir deste nó para baixo. */
        unsigned getPotentialContacts(PotentialContact* contacts, unsigned limit) const
        {
            if (isLeaf() || limit == 0) return 0;
            return children[0]->getPotentialContactsWith(children[1], contacts, limit);
        }

    private:
        /** Recalcula o volume do nó combinando os filhos. */
        void recalculateBoundingVolume()
        {
            if (isLeaf()) return;
            volume = BoundingVolumeClass(children[0]->volume, children[1]->volume);
            if (parent) {
                parent->recalculateBoundingVolume();
            }
        }

        /** Verifica potenciais contatos entre sub-árvores. */
        unsigned getPotentialContactsWith(const BVHNode<BoundingVolumeClass> *other,
                                          PotentialContact* contacts,
                                          unsigned limit) const
        {
            if (!overlaps(other) || limit == 0) return 0;

            if (isLeaf() && other->isLeaf())
            {
                contacts->body[0] = body;
                contacts->body[1] = other->body;
                return 1;
            }

            if (other->isLeaf() || (!isLeaf() && volume.getSize() >= other->volume.getSize()))
            {
                unsigned count = children[0]->getPotentialContactsWith(other, contacts, limit);
                if (limit > count) {
                    return count + children[1]->getPotentialContactsWith(other, contacts + count, limit - count);
                } else {
                    return count;
                }
            }
            else
            {
                unsigned count = getPotentialContactsWith(other->children[0], contacts, limit);
                if (limit > count) {
                    return count + getPotentialContactsWith(other->children[1], contacts + count, limit - count);
                } else {
                    return count;
                }
            }
        }

        bool overlaps(const BVHNode<BoundingVolumeClass> *other) const
        {
            return volume.overlaps(&(other->volume));
        }

        // Método auxiliar para estimativa de tamanho (usado no balanceamento)
        real getSize() const {
            return volume.radius; // Simplificação baseada no raio da esfera
        }
    };

}
