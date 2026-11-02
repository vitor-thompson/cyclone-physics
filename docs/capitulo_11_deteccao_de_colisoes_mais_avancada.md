# Capitulo 11: Deteccao de Colisoes Mais Avancada (Advanced Collision Detection)

Este documento registra os algoritmos de teste fino e a geracao detalhada de contatos entre caixas e outras primitivas no motor **Cyclone**, correspondente ao **Capitulo 11** do livro *Game Physics Engine Development* (Ian Millington).

O Capitulo 11 aprofunda o uso do **Teorema do Eixo Separador (SAT)** para gerar pontos de contato precisos, normais de colisao e profundidades de penetracao no teste **Caixa vs Caixa** (OBB vs OBB).

## 📦 1. Testes de Caixa vs Caixa (OBB vs OBB)

Para detectar a colisao entre duas caixas orientadas no espaco 3D, o motor avalia os 15 eixos de separacao potenciais. O teste de projecao de uma caixa em relação a outra e expresso comparando o deslocamento entre os centros com o raio de projecao combinado:

$$
\vert{} \mathbf{T} \cdot \mathbf{L} \vert{} > r_A + r_B
$$

Onde $\mathbf{T}$ e o vetor entre os centros das duas caixas e $\mathbf{L}$ e o eixo de teste atual.

### Os 15 Eixos de Teste:

1. **Eixos Principais da Caixa A (**$\mathbf{x}_A, \mathbf{y}_A, \mathbf{z}_A$**):** Testam se uma face da Caixa A esta separando os objetos.

2. **Eixos Principais da Caixa B (**$\mathbf{x}_B, \mathbf{y}_B, \mathbf{z}_B$**):** Testam se uma face da Caixa B esta separando os objetos.

3. **Produtos Vetoriais das Arestas (**$\mathbf{e}_{Ai} \times \mathbf{e}_{Bj}$**):** 9 eixos que testam contatos entre arestas (*edge-to-edge*).

## 🛠️ 2. Determinacao dos Tipos de Contato

Apos checar todos os 15 eixos, se nenhum eixo separador for encontrado, confirma-se a colisao. O eixo que registrar a **menor penetracao positiva** determina a geometria da resposta:

### 2.1 Contato Face-Vertice ou Face-Face

Ocorre quando o eixo de menor penetracao e um dos eixos principais de uma das caixas ($\mathbf{x}, \mathbf{y}$ ou $\mathbf{z}$).

* A normal de contato e paralela ao eixo da face identificada.

* O ponto de contato e determinado buscando o vertice da outra caixa que esta mais profundo em relacao a essa face.

### 2.2 Contato Aresta-Aresta (*Edge-to-Edge*)

Ocorre quando o eixo de menor penetracao e resultado de um produto vetorial entre uma aresta da Caixa A e uma aresta da Caixa B.

* A normal de contato e paralela ao vetor perpendicular a ambas as arestas ($\mathbf{e}_A \times \mathbf{e}_B$).

* O ponto de contato e determinado pelo ponto mais proximo entre os dois segmentos de reta no espaco 3D.

## ⚡ 3. Oportunidades de Otimizacao (*Early-Outs*)

O algoritmo SAT pode ser custoso se todos os 15 eixos forem testados rigorosamente a cada quadro. O motor aplica estrategias para descartar pares rapidamente:

* **Checagem Hierarquica Preliminar:** Testar primeiro as esferas envolventes (*bounding spheres*) de cada caixa antes de executar o SAT.

* **Fase de Early-Out:** Interromper imediatamente o teste do objeto se qualquer um dos eixos retornar uma separacao valida ($\vert{} \mathbf{T} \cdot \mathbf{L} \vert{} > r_A + r_B$), pois a ausencia de sobreposicao em um unico eixo garante que nao ha colisao.

## 💻 4. Integracao com o Detector de Colisoes (`CollisionDetector`)

Os algoritmos do Capítulo 11 completam a classe `CollisionDetector` no motor, que recebe duas primitivas de colisao e preenche a estrutura `CollisionData` com a lista de contatos gerados (`Contact`).