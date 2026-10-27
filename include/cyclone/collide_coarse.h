namespace cyclone {

    template<class BoundingVolumeClass>
    class BVHNode
    {
    public:
        BVHNode *children[2];
        BVHNode *parent;
        BoundingVolumeClass volume;
        RigidBody *body;

        // Construtor para nós internos ou raiz
        BVHNode(BVHNode *parent, const BoundingVolumeClass &volume, RigidBody *body = nullptr)
            : parent(parent), volume(volume), body(body)
        {
            children[0] = nullptr;
            children[1] = nullptr;
        }

        ~BVHNode()
        {
            // Se tivermos um pai, removemos este nó e promovemos o irmão
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

            // Deleta os filhos de forma segura
            if (children[0]) {
                children[0]->parent = nullptr;
                delete children[0];
            }
            if (children[1]) {
                children[1]->parent = nullptr;
                delete children[1];
            }
        }

        bool isLeaf() const
        {
            return (body != nullptr);
        }

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

        void recalculateBoundingVolume()
        {
            if (isLeaf()) return;
            volume = BoundingVolumeClass(children[0]->volume, children[1]->volume);
            if (parent) {
                parent->recalculateBoundingVolume();
            }
        }
    };

}
