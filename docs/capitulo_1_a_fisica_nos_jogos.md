# Capitulo 1: A Fisica nos Jogos (Physics in Games)

Este documento registra a analise teorica e os conceitos fundamentais apresentados no **Capitulo 1** do livro *Game Physics Engine Development* (Ian Millington).

O Capitulo 1 estabelece os fundamentos teoricos para o desenvolvimento de motores de fisica 3D em tempo real, definindo os objetivos, limites e a matematica necessaria para a construcao do motor **Cyclone**.

## 📐 1. O Papel da Fisica nos Jogos

A fisica em jogos nao busca a precisao cientifica absoluta, mas sim a **ilusao de realidade e estabilidade** dentro de um limite de tempo estrito por quadro (normalmente $16.6\text{ms}$ para 60 FPS).

### Principais Objetivos:

1. **Credibilidade (*Believabilidade*):** Os objetos devem se comportar de maneira intuitiva para o jogador.

2. **Estabilidade Numrica:** A simulacao nao pode explodir ou acumular erros infinitos ao longo do tempo.

3. **Desempenho:** O algoritmo deve ser leve o suficiente para rodar junto com renderizacao, IA e sistemas de jogo.

## 🧮 2. Conceitos Matematicos Fundamentais

### 2.1 Vetores e Espaco Tridimensional

Um ponto ou vetor no espaco 3D e representado por tres componentes reais:

$$
\mathbf{p} = \begin{bmatrix} x \\ y \\ z \end{bmatrix}
$$

* **Soma e Subtracao de Vetores:** Permite deslocamentos e operacoes de posicao relativa.

* **Multiplicacao por Escalar:** Altera a magnitude do vetor mantendo sua direcao.

* **Produto Escalar (*Dot Product*):**
  

  $$
  \mathbf{a} \cdot \mathbf{b} = a_x b_x + a_y b_y + a_z b_z = \vert{}\mathbf{a}\vert{} \vert{}\mathbf{b}\vert{} \cos(\theta)
  $$

  
  Utilizado para determinar projeções e angulos entre vetores.

* **Produto Vetorial (*Cross Product*):**
  

  $$
  \mathbf{a} \times \mathbf{b} = \begin{bmatrix} a_y b_z - a_z b_y \\ a_z b_x - a_x b_z \\ a_x b_y - a_y b_x \end{bmatrix}
  $$

  
  Gera um vetor perpendicular a ambos os vetores de entrada, essencial para torques e normais de plano.

## ⏱️ 3. O Loop de Simulacao e Integracao Numérica

A simulacao de fisica avanca no tempo atraves de pequenos incrementos chamados $\Delta t$ (*frame time* ou *duration*).

### Equacoes Basicas do Movimento de Newton:

* **Posicao:** $\mathbf{p}' = \mathbf{p} + \mathbf{v} \Delta t + \frac{1}{2} \mathbf{a} \Delta t^2$

* **Velocidade:** $\mathbf{v}' = \mathbf{v} + \mathbf{a} \Delta t$

No motor Cyclone, o metodo inicial de integracao numérica escolhido e o **Newton–Euler de 1ª Ordem (Euler Explicito/Semi-implicito)** devido ao seu baixo custo computacional.

## 📂 4. Estrutura Inicial do Codigo

O capítulo introduz os tipos base e abstracoes iniciais de precisao matematica:

* `precision.h`: Define o tipo de ponto flutuante `real` (`float` ou `double`).

* `core.h`: Implementa a classe base `Vector3` e suas operacoes algebricas fundamentais.