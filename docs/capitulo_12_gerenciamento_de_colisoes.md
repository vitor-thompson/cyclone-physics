# Capitulo 12: Gerenciamento de Colisoes (Collision Processing)

Este documento registra a estrutura do pipeline de deteccao de colisoes e a organizacao espacial no motor **Cyclone**, correspondente ao **Capitulo 12** do livro *Game Physics Engine Development* (Ian Millington).

O Capitulo 12 estabelece a ponte entre a deteccao de colisao teorica e a simulacao em larga escala, dividindo o processo em **Fase Broad-Phase** (selecao bruta) e **Fase Narrow-Phase** (teste detalhado).

## 🗂️ 1. As Duas Fases da Deteccao de Colisoes

Em uma cena com $N$ objetos, testar todas as combinacoes possiveis exige $O(N^2)$ verificacoes complexas. Para resolver esse problema de desempenho, o motor divide a deteccao em duas etapas distintas:

```
[ Todos os Objetos da Cena ]
             │
             ▼
   1. BROAD-PHASE  ────► Descarta pares distantes rapidamente (Bounding Volumes)
             │
             ▼
   2. NARROW-PHASE ────► Testes exatos nas primitivas (SAT, Esferas, etc.)
             │
             ▼
   [ Lista de Contatos Final ]
```

### 1.1 Fase Bruta (*Broad-Phase*)

Filtra rapidamente a maioria dos pares de objetos que estao distantes uns dos outros e que comprovadamente nao estao colidindo. Usa algoritmos leves de baixo custo computacional.

### 1.2 Fase Fina (*Narrow-Phase*)

Executa os algoritmos exatos de calculo de intersecao (vistos nos Capitulos 10 e 11) apenas nos pares de objetos selecionados pela fase *broad-phase*.

## 📦 2. Volumes Envolventes (*Bounding Volumes*)

Para realizar os testes rapidos na *broad-phase*, cada objeto da cena e envolvido por uma forma geometrica simplificada de calculo ultra-rapido.

### 2.1 Esfera Envolvente (*Bounding Sphere*)

Uma esfera definida por um raio $r$ centrado no objeto. O teste entre duas esferas envolventes exige apenas uma comparacao de distancia entre centros:

$$
\vert{} \mathbf{c}_1 - \mathbf{c}_2 \vert{}^2 < (r_1 + r_2)^2
$$

* **Vantagens:** Invariante a rotacao (nao precisa ser recalculada quando o corpo gira) e extremamente performatica.

### 2.2 Hierarquia de Volumes Envolventes (BVH - Bounding Volume Hierarchy)

Para cenas complexas, o motor organiza as esferas envolventes em uma estrutura de arvore binaria chamada **BVH** (`BVHNode`).

* Cada no da arvore possui uma esfera envolvente que cobre totalmente as esferas de seus nos filhos.

* Durante o teste de colisao, se a esfera de um no pai nao intercepta o teste, toda a sub-arvore abaixo dele e descartada instantaneamente.

## ⚙️ 3. O Pipeline de Gerenciamento de Colisoes

A classe `CollisionBroadPhase` e a estrutura de nos `BVHNode` gerenciam os testes na cena:

1. **Atualizacao da Arvore:** As esferas envolventes dos objetos em movimento sao reavaliadas a cada quadro.

2. **Travessia da Arvore:** A arvore e percorrida em busca de nos cujas esferas envolventes estejam se sobrepondo.

3. **Geracao do Buffer de Pares Potenciais:** Os pares de objetos cujos volumes envolventes colidiram sao enviados para a *narrow-phase*.

4. **Geracao da Lista Final de Contatos:** A *narrow-phase* executa os testes exatos e preenche o array global de objetos `Contact` para envio ao `ContactResolver`.