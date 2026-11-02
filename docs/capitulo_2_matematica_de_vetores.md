# Capitulo 2: Matematica de Vetores (Vector Math)

Este documento registra a fundamentacao matematica e a implementacao da biblioteca de vetores tridimensionais no motor **Cyclone**, correspondente ao **Capitulo 2** do livro *Game Physics Engine Development* (Ian Millington).

O Capitulo 2 estabelece a estrutura de dados central para todo o processamento espacial da engine: a classe `Vector3` e suas operacoes algebricas fundamentais.

## 📐 1. O Conceito de Vetor na Fisica 3D

Na simulacao fisica, vetores sao utilizados para representar duas entidades distintas:

1. **Pontos no Espaco:** Posicoes relativas a uma origem fixada.

2. **Grandezas Vetoriais:** Entidades que possuem **magnitude** e **direcao** (ex.: velocidade, aceleracao, forca, torque, momento).

Uma estrutura tridimensional armazena tres valores reais correspondentes aos eixos $X$, $Y$ e $Z$:

$$
\mathbf{v} = \begin{bmatrix} x \\ y \\ z \end{bmatrix}
$$

## 🧮 2. Operacoes Vetoriais Fundamentais

### 2.1 Soma e Subtracao de Vetores

Permitem a combinacao de deslocamentos e o calculo de posicoes relativas entre dois objetos.

$$
\mathbf{a} + \mathbf{b} = \begin{bmatrix} a_x + b_x \\ a_y + b_y \\ a_z + b_z \end{bmatrix}, \quad \mathbf{a} - \mathbf{b} = \begin{bmatrix} a_x - b_x \\ a_y - b_y \\ a_z - b_z \end{bmatrix}
$$

### 2.2 Escalonamento (Multiplicacao por Escalar)

Altera a magnitude de um vetor sem modificar sua direcao basica (ou invertendo-a se o escalar for negativo).

$$
k \mathbf{a} = \begin{bmatrix} k a_x \\ k a_y \\ k a_z \end{bmatrix}
$$

### 2.3 Magnitude e Normalizacao

A magnitude (comprimento) de um vetor e dada pela norma euclidiana:

$$
\vert{}\mathbf{a}\vert{} = \sqrt{a_x^2 + a_y^2 + a_z^2}
$$

Para obter o vetor unitario (direcao pura com magnitude $1$), realiza-se a normalizacao:

$$
\hat{\mathbf{a}} = \frac{\mathbf{a}}{\vert{}\mathbf{a}\vert{}}
$$

### 2.4 Produto Escalar (*Dot Product*)

O produto escalar de dois vetores resulta em um valor escalar real:

$$
\mathbf{a} \cdot \mathbf{b} = a_x b_x + a_y b_y + a_z b_z = \vert{}\mathbf{a}\vert{} \vert{}\mathbf{b}\vert{} \cos(\theta)
$$

* **Propriedade Geometrica:** Se $\mathbf{a} \cdot \mathbf{b} = 0$, os vetores sao ortogonais (perpendiculares). Se $> 0$, apontam para direcoes semelhantes; se $< 0$, apontam em direcoes opostas.

### 2.5 Produto Vetorial (*Cross Product*)

O produto vetorial produz um novo vetor que e **simultaneamente perpendicular** aos dois vetores de entrada, seguindo a regra da mao direita:

$$
\mathbf{a} \times \mathbf{b} = \begin{bmatrix} a_y b_z - a_z b_y \\ a_z b_x - a_x b_z \\ a_x b_y - a_y b_x \end{bmatrix}
$$

* **Aplicacoes:** Calculo de normais de superfícies, vetores de torque e eixos de rotacao.

### 2.6 Componente a Componente (*Component Product*)

Multiplicacao elemento por elemento, util para escalonar eixos independentemente (como coeficientes de atrito ou amortecimento):

$$
\mathbf{a} \odot \mathbf{b} = \begin{bmatrix} a_x b_x \\ a_y b_y \\ a_z b_z \end{bmatrix}
$$

## 💻 3. Estrutura do Codigo em C++

A classe `Vector3` e definida na biblioteca do motor (`core.h`).

### Recursos de Implementacao:

* Uso do tipo `real` (definido em `precision.h`) para alternar facilmente entre `float` e `double`.

* Sobrecarga de operadores C++ (`+`, `-`, `*`, `+=`, `-=`, `*=`, `[]`) para sintaxe matematica limpa.

* Metodos utilitarios como `squareMagnitude()`, `invert()`, `normalise()`, `dot()`, `cross()`, e `addScaledVector()`.