namespace cyclone {

    /**
     * @brief Armazena um par de corpos rígidos que podem estar colidindo.
     */
    struct PotentialContact
    {
        RigidBody* body[2];
    };

    /**
     * @brief Classe base para nós em uma hierarquia de volume envolvente (Árvore Binária).
     */
    template<class BoundingVolumeClass>
    class BVHNode
    {
    public:
        /** Nós filhos (geralmente 2 em uma árvore binária). */
        BVHNode *children[2];

        /** Volume envolvente que engloba todos os descendentes deste nó. */
        BoundingVolumeClass volume;

        /** Corpo rígido associado (apenas nós folha possuem um corpo válido). */
        RigidBody *body;

        /** Verifica se este nó é uma folha (contém um objeto físico). */
        bool isLeaf() const
        {
            return (body != nullptr);
        }

        /** Obtém a lista de potenciais contatos a partir deste nó para baixo. */
        unsigned getPotentialContacts(PotentialContact* contacts, unsigned limit) const;

    protected:
        unsigned getPotentialContactsWith(const BVHNode<BoundingVolumeClass> *other,
                                          PotentialContact* contacts,
                                          unsigned limit) const;
    };
}
