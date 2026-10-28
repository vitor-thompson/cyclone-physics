# Capítulo 13: Gerando Contatos (Fase Fina e Geometria de Colisão)

## 1. Visão Geral
No Capítulo 13 do motor **Cyclone**, transitamos da detecção de colisão ampla (fase grossa ou *coarse phase*) para a **fase fina (*fine phase*)** e **geração de contatos**. O objetivo desta etapa é inspecionar pares de primitivas geométricas candidatos a colisão, verificar se estão de fato sobrepostos e calcular os dados exatos necessários para a resolução física:
* **Ponto de contato (`contactPoint`)**: A localização do impacto no espaço global.
* **Normal de contato (`contactNormal`)**: O vetor unitário que aponta na direção de afastamento dos corpos.
* **Profundidade de penetração (`penetration`)**: A distância de sobreposição entre os objetos.

---

## 2. Estrutura de Dados e Design do Motor

### 2.1 Gerenciamento de Contatos (`CollisionData`)
Para acomodar algoritmos que geram de zero a múltiplos pontos de contato por par de primitivas, o motor utiliza a estrutura `CollisionData`. Ela funciona como um *buffer* seguro que previne estouro de memória durante a simulação:

```cpp
struct CollisionData
{
    Contact *contacts;       // Vetor onde os contatos são gravados
    unsigned contactsLeft;  // Vagas restantes no vetor
    unsigned contactCount; // Total de contatos gerados
    real friction;         // Coeficiente global de atrito
    real restitution;      // Coeficiente global de restituição
    real tolerance;        // Tolerância de penetração
};
```

### 2.2 Primitivas de Colisão (`Primitive`)
As primitivas representam as formas geométricas invisíveis conectadas aos corpos rígidos. A classe base `Primitive` possui uma matriz de *offset* local, permitindo que a geometria de colisão esteja deslocada ou rotacionada em relação ao centro de massa do corpo rígido:

$$\mathbf{T}_{\text{world}} = \mathbf{T}_{\text{body}} \cdot \mathbf{M}_{\text{offset}}$$

```cpp
class Primitive
{
public:
    RigidBody *body;
    Matrix4 offset;

    Matrix4 getTransform() const
    {
        return body->getTransform() * offset;
    }

    Vector3 getAxis(unsigned index) const
    {
        return getTransform().getAxis(index);
    }
};
```

---

## 3. Algoritmos de Colisão por Primitiva

### 3.1 Esfera vs. Esfera (`sphereAndSphere`)
Dois objetos esféricos colidem se a distância $d$ entre seus centros for menor do que a soma de seus raios ($r_1 + r_2$). Produz sempre **exatamente 1 ponto de contato**.

$$\text{Penetração } p = (r_1 + r_2) - d$$
$$\text{Normal } \mathbf{n} = \frac{\mathbf{c}_1 - \mathbf{c}_2}{d}$$

### 3.2 Esfera vs. Plano / Meia-Espaço (`sphereAndHalfSpace` / `sphereAndTruePlane`)
O plano é definido por uma normal $\mathbf{l}$ e um deslocamento escalar $d$. A distância do centro da esfera ao plano é dada por:

$$s = \mathbf{c} \cdot \mathbf{l} - d$$

* **Meia-espaço (*Half-Space*)**: Um lado é totalmente sólido. Se $s - r < 0$, há colisão.
* **Plano Verdadeiro (*True Plane*)**: Permite colisão em ambas as faces, invertendo a normal conforme o lado de abordagem.

### 3.3 Caixa vs. Plano (`boxAndPlane`)
Para tratar colisões entre uma caixa orientada (OBB) e um plano, cada um dos **8 vértices** da caixa é testado contra a equação do plano. Esta rotina pode gerar até **4 pontos de contato simultâneos** (um para cada canto penetrante), oferecendo estabilidade para caixas apoiadas sobre superfícies planas.

### 3.4 Caixa vs. Esfera (`boxAndSphere`)
1. O centro da esfera é transformado para o espaço local da caixa.
2. É feito um teste rápido de eixo separador (*early-out*).
3. Utiliza-se *clamping* nas coordenadas relativas para determinar o ponto mais próximo da superfície da caixa.
4. Se a distância até o ponto mais próximo for menor que o raio, gera-se o contato.

### 3.5 Caixa vs. Caixa (`boxAndBox`)
A colisão entre duas caixas orientadas é resolvida combinando:
1. **Teorema dos Eixos Separadores (SAT)**: Teste de 15 eixos (3 eixos da Caixa A, 3 eixos da Caixa B e 9 produtos vetoriais cruzados) para descarte rápido (*early-out*).
2. **Determinação do Ponto Mais Profundo**: Testes de vértices contra faces (`boxAndPoint`) e arestas contra arestas.
3. **Coerência Temporal (*Caching*)**: Acúmulo de pontos de contato ao longo de múltiplos quadros para manter o empilhamento de objetos estável sem recalcular manifolds complexos a cada quadro.

---

## 4. Arquivos do Sistema

* **Interface (`include/cyclone/collide_fine.h`)**: Define `CollisionData`, `Primitive`, `Sphere`, `Plane`, `Box` e a classe estática `CollisionDetector`.
* **Implementação (`src/collide_fine.cpp`)**: Contém a lógica matemática completa das funções de geração de contatos.

---

## 5. Próximos Passos
Com a infraestrutura de detecção grossa e fina finalizada, o motor está pronto para avançar para o **Capítulo 14 (Tratamento de Impactos e Resolução de Contatos com Rotação)**.