# Capítulo 12: Detecção Grossa de Colisões (Coarse Collision Detection)

Este capítulo introduz a primeira fase do pipeline de detecção de colisões do motor **Cyclone**: a **Detecção Grossa (*Coarse Collision Detection*)**. O objetivo desta etapa é filtrar rapidamente os pares de objetos que estão próximos o suficiente para colidir, descartando a grande maioria do cenário e evitando checagens geométricas detalhadas desnecessárias.

---

## 1. O Pipeline de Colisão

Para manter o desempenho em tempo real, a detecção de colisões é dividida em duas etapas:
1. **Detecção Grossa (Coarse):** Um filtro rápido e conservador baseado em volumes simples (esferas/caixas) e estruturas espaciais para gerar uma lista de potenciais contatos. Nunca deve gerar falsos negativos (perder uma batida real).
2. **Detecção Fina (Fine):** Uma análise geométrica detalhada apenas sobre os candidatos selecionados na fase anterior, calculando o ponto exato de contato, a normal e a profundidade de penetração (Capítulo 13).

---

## 2. Volumes Envolventes e Esferas (`BoundingSphere`)

Para simplificar os testes iniciais, os objetos do motor são envolvidos por **Esferas Envolventes (`BoundingSphere`)**. 
* **Vantagem:** A verificação de sobreposição entre esferas é extremamente rápida, utilizando a distância ao quadrado para evitar o cálculo custoso de raízes quadradas.

```cpp
bool BoundingSphere::overlaps(const BoundingSphere *other) const
{
    real distanceSquared = (center - other->center).squareMagnitude();
    real radiusSum = radius + other->radius;
    return distanceSquared < (radiusSum * radiusSum);
}
```

---

## 3. Hierarquia de Volumes Envolventes (BVH)

Para evitar comparar todos os objetos do mundo entre si ($O(N^2)$), as esferas envolventes são organizadas em uma árvore binária chamada **BVH (*Bounding Volume Hierarchy*)**.

* **Poda de Galhos:** Se duas esferas de nós pais no topo da árvore não se sobrepõem, todos os descendentes abaixo delas são descartados instantaneamente.
* **Inserção Dinâmica:** O motor implementa o algoritmo de inserção em tempo de execução, permitindo adicionar e remover corpos rígidos sem a necessidade de reconstruir a árvore inteira a cada quadro. O nó é inserido naquele que sofre a menor expansão volumétrica:

```cpp
template<class BoundingVolumeClass>
void BVHNode<BoundingVolumeClass>::insert(
    RigidBody* newBody, const BoundingVolumeClass &newVolume
)
{
    if (isLeaf())
    {
        children[0] = new BVHNode<BoundingVolumeClass>(this, volume, body);
        children[1] = new BVHNode<BoundingVolumeClass>(this, newVolume, newBody);
        this->body = nullptr;
        recalculateBoundingVolume();
    }
    else
    {
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
```

---

## 4. Estruturas de Dados Espaciais

Além das BVHs (organizadas por objetos), o capítulo aborda estruturas fixas no mundo:
* **BSP Trees (Binary Space Partition):** Divisão do espaço através de planos infinitos usando o produto escalar.
* **Quad-trees e Oct-trees:** Divisão simétrica em quadrantes (2D/XZ) ou octantes (3D) alinhados aos eixos do mundo.
* **Grades (*Grids*):** Tabuleiros regulares que mapeiam posições diretamente para células através de índices matemáticos em tempo constante $O(1)$.

---

## 5. Próximos Passos
Com a infraestrutura de detecção grossa concluída e os potenciais contatos mapeados, o próximo passo (**Capítulo 13**) abordará a **Detecção Fina e a Geração de Contatos**, onde as geometrias reais dos corpos rígidos interagem.