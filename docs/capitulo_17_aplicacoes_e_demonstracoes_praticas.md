# Capitulo 17: Aplicacoes e Demonstracoes Praticas (Applications and Demos)

Este documento registra o encerramento do desenvolvimento do motor **Cyclone**, integrando todos os sistemas desenvolvidos ao longo do livro *Game Physics Engine Development* (Ian Millington) em aplicacoes praticas, sistemas de fratura e efeitos ambientais complexos, correspondente ao **Capitulo 17**.

O Capitulo 17 demonstra como utilizar a fisica de particulas, a dinamica de corpos rigidos, a deteccao de colisoes e o resolvedor de contatos para construir cenarios interativos e complexos.

## 🧱 1. Implementacao de Fraturas e Quebra de Objetos (`fracture.cpp`)

A simulacao de fraturas demonstra a transicao dinamica de um unico corpo rigido para multiplos fragmentos independentes no momento do impacto.

### Mecanismo de Funcionamento:

1. **Monitoramento de Impacto:** A cada colisao registrada pelo `ContactResolver`, inspeciona-se a magnitude do impulso aplicado ($k$).

2. **Criterio de Fratura:** Se o impulso transferido em um unico ponto exceder o limite de resistencia de material configurado ($k_{\text{limite}}$), o objeto pai e removido da simulacao.

3. **Geracao de Fragmentos:** Sub-componentes rigidos geometricos (caixas ou tetraedros menores) sao gerados na posicao exata do objeto pai, herdando sua velocidade linear ($\mathbf{v}$) e velocidade angular ($\boldsymbol{\omega}$) no instante do impacto.

4. **Impulso de Explosao:** Um pequeno vetor de velocidade divergente e adicionado a cada fragmento para simular o espalhamento de estilhacos.

## 💥 2. Explosoes de 3 Componentes (`fgen.h`)

Para simular o efeito de uma explosao realista em objetos proximos, implementa-se um gerador de forca especializado que divide a explosao em tres fases fisicas complementares:

### 2.1 Onde de Choque Repulsiva (*Shockwave Force*)

Aplica uma forca radial instantanea apontando da origem da explosao $\mathbf{p}_{\text{exp}}$ para a posicao do objeto $\mathbf{p}_{\text{obj}}$:

$$
\mathbf{F}_{\text{choque}} = k_{\text{exp}} \cdot \frac{\mathbf{p}_{\text{obj}} - \mathbf{p}_{\text{exp}}}{\vert{}\mathbf{p}_{\text{obj}} - \mathbf{p}_{\text{exp}}\vert{}^3}
$$

A intensidade atena quadraticamente com a distancia.

### 2.2 Implosao por Conveccao / Vazio (*Convective Implosion*)

Apos a expansao inicial da onda de choque, simula-se o vacuo criado pela explosao puxando temporariamente objetos leves de volta em direcao ao epicentro.

### 2.3 Ondas de Elevacao Aerodinamica (*Convective Lift*)

Aplica uma componente de forca ascendente em $Y$ nos objetos dentro do raio de alcance para simular a subida de ar quente criada pela bola de fogo.

## ⚠️ 3. Limitacoes Conhecidas do Motor Cyclone

Com base na integracao completa do motor no Capitulo 17, foram identificadas e documentadas as seguintes limitacoes estruturais:

1. **Instabilidade de Pilhas (*Stacking Instabilities*):** Como a resolucao de contatos e puramente por impulsos iterativos locais, pilhas altas de caixas tendem a afundar ou tremer (*jitter*) devido a falta de um resolvedor global de equacoes simultaneas (LCP - *Linear Complementarity Problem*).

2. **Atrito em Resolucao de Posicao:** A resolucao de penetracao posicional aplica translacoes diretas nos corpos sem considerar o atrito tangencial, o que pode causar descompressões bruscas em cantos colididos.

3. **Molas Rígidas (*Stiff Springs*):** Molas com constantes de rigidez $k$ muito elevadas exigem passos de tempo de integracao ($\Delta t$) extremamente pequenos para evitar instabilidades ou divisoes por valores divergentes (*exploding physics*).

## 🗂️ 4. Conclusao da Documentacao da Engine

Com a conclusao deste capitulo, a documentacao completa do motor **Cyclone** (Capitulos 1 a 17) esta finalizada, cobrindo:

* **Matematica Espacial:** Vetores, Matrizes, Quaternions.

* **Fisica de Particulas:** Integracao, Geradores de Forca, Cabos e Hastes.

* **Dinamica de Corpos Rigidos:** Tensores de Inercia, Torque, Matrizes de Transformacao.

* **Deteccao de Colisoes:** Primitivas, SAT (Eixo Separador), Hierarquias BVH.

* **Resolucao e Estabilidade:** Impulsos 3D, Atrito de Coulomb, Desativacao (*Sleep*), Juntas e Aplicacoes Praticas.