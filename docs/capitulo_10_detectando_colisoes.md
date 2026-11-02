# Capitulo 10: Detectando Colisoes (Collision Detection)

Este documento registra os conceitos teoricos e as estruturas de teste para deteccao de colisoes entre primitivas no motor **Cyclone**, correspondente ao **Capitulo 10** do livro *Game Physics Engine Development* (Ian Millington).

O Capitulo 10 aborda a primeira fase do pipeline de colisoes: determinar se dois objetos geometricos se sobrepoem e gerar os dados necessarios (`Contact`) para que o resolvedor possa corrigir a interpenetracao e calcular a resposta fisica.

## 🟨 1. Primitivas de Colisao (`CollisionPrimitive`)

Para manter o processamento em tempo real extremamente eficiente, o motor nao testa malhas complexas diretamente. Em vez disso, utiliza **primitivas geometricas simples**:

* **Esfera (`CollisionSphere`):** Definida por um raio $r$ e um ponto central.
* **Caixa (`CollisionBox`):** Definida por tres meias-extensoes (*half-sizes*) ao longo de seus eixos locais $X$, $Y$ e $Z$.
* **Plano / Semi-espaco (`CollisionPlane`):** Definido por uma normal $\mathbf{\hat{n}}$ e um deslocamento escalar (*offset*).

Cada primitiva mantem um ponteiro para o seu respectivo `RigidBody` e possui uma matriz de transformacao derivada para converter pontos entre o espaco local da primitiva e o espaco do mundo.

## 🔍 2. Algoritmos de Teste de Intersecao

### 2.1 Esfera vs Esfera

O teste mais simples e performatico. Duas esferas com centros $\mathbf{c}_1, \mathbf{c}_2$ e raios $r_1, r_2$ estao em colisao se a distancia entre seus centros for menor que a soma de seus raios:

$$
\vert{} \mathbf{c}_1 - \mathbf{c}_2 \vert{} < r_1 + r_2
$$

* **Penetracao:** $g = (r_1 + r_2) - \vert{} \mathbf{c}_1 - \mathbf{c}_2 \vert{}$.
* **Normal de Contato:** $\mathbf{\hat{n}} = \frac{\mathbf{c}_1 - \mathbf{c}_2}{\vert{} \mathbf{c}_1 - \mathbf{c}_2 \vert{}}$ (direcionada do segundo objeto para o primeiro).

### 2.2 Esfera vs Plano / Semi-espaco

Avalia a distancia do centro da esfera $\mathbf{c}$ ao plano definido por $\mathbf{\hat{n}} \cdot \mathbf{p} = d$:

$$
s = \mathbf{\hat{n}} \cdot \mathbf{c} - d
$$

* Se $s < r$, a esfera esta penetrando o plano.
* **Penetracao:** $g = r - s$.
* **Normal de Contato:** $\mathbf{\hat{n}}$ do plano.

### 2.3 Caixa vs Plano / Semi-espaco

Testa cada um dos 8 vertices da caixa em relacao ao plano. Para cada vertice que estiver atras da superficie do plano, um contato e gerado.

* O ponto de contato e a posicao exata do vertice penetrante.
* A penetracao de cada vertice e calculada pela distancia perpendicular ate a superficie do plano.

## 📐 3. O Teorema do Eixo Separador (SAT - Separating Axis Theorem)

O **SAT** e o algoritmo fundamental utilizado para determinar a intersecao entre formas convexas 3D (como caixas orientadas - OBBs).

### Principio Fundamental:

> *Dois objetos convexos nao se sobrepoem se existir pelo menos um eixo no qual suas projecoes nao se sobreponham.*

Para duas caixas 3D orientadas, existem exatamente **15 eixos potenciais de separacao** que precisam ser testados:

1. **3 eixos locais da Caixa A** ($\mathbf{u}_{A1}, \mathbf{u}_{A2}, \mathbf{u}_{A3}$).
2. **3 eixos locais da Caixa B** ($\mathbf{u}_{B1}, \mathbf{u}_{B2}, \mathbf{u}_{B3}$).
3. **9 eixos formados pelo produto vetorial das arestas** ($\mathbf{u}_{Ai} \times \mathbf{u}_{Bj}$).

### Regra do Contato:

* Se um eixo separador for encontrado, os objetos **nao estao em colisao**.
* Se nenhum eixo separador for encontrado em todos os 15 testes, os objetos **estao se sobrepondo**.
* O eixo que apresentar a **menor sobreposicao de projecao** e escolhido como o eixo da colisao, fornecendo a **normal de contato** e a **profundidade de penetracao**.