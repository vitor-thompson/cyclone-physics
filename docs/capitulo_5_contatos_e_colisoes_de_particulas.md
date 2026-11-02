# Capitulo 5: Contatos e Colisoes de Particulas (Particle Contacts)

Este documento registra a teoria, a estrutura e a implementacao do sistema de contatos e colisoes entre particulas no motor **Cyclone**, correspondente ao **Capitulo 5** do livro *Game Physics Engine Development* (Ian Millington).

O Capitulo 5 introduz a resolucao de colisoes e restricoes posicionais sem recorrer a forcas contínuas, utilizando a teoria de **impulsos instantaneos**.

## 💥 1. O Modelo de Contato de Particulas

Um contato representa uma restricao fisica entre duas particulas (ou entre uma particula e um objeto estatico, como um plano) onde o movimento livre e limitado.

### Estrutura de Dados do Contato (`ParticleContact`):

* **Particulas Envolvidas (`particle[2]`):** Ponteiros para as duas particulas afetadas (se a segunda for nula, o contato e contra o ambiente estatico).

* **Coeficiente de Restituicao ($e$):** Define quao elastica e a colisaes ($e = 1$ para colisao perfeitamente elastica; $e = 0$ para colisao perfeitamente inelastica/plastica).

* **Normal de Contato ($\mathbf{\hat{n}}$):** Vetor unitario que indica a direcao ao longo da qual a colisao ou restricao atua.

* **Penetracao ($g$):** A quantidade de sobreposicao espacial entre os objetos (deve ser resolvida para evitar interpenetracoes visiveis).

## ⚡ 2. Resolucao por Impulso

Como as colisoes ocorrem em intervalos de tempo infinitesimais, o motor calcula um **impulso** ($\mathbf{g} = m \Delta \mathbf{v}$) para alterar instantaneamente as velocidades das particulas envolvidas.

### 2.1 Velocidade de Separacao

A velocidade com que os dois objetos se movem ao longo da normal de contato e dada por:

$$
v_s = (\mathbf{v}_a - \mathbf{v}_b) \cdot \mathbf{\hat{n}}
$$

* Se $v_s > 0$, as particulas estao se separando e nenhuma acao e necessaria.
* Se $v_s \le 0$, as particulas estao se aproximando ou em contato firme, exigindo resolucao.

### 2.2 Nova Velocidade de Separacao

Apos a colisao, a nova velocidade de separacao desejada e:

$$
v_s' = -e \cdot v_s
$$

A mudanca de velocidade necessaria e $\Delta v_s = v_s' - v_s = -v_s (1 + e)$.

### 2.3 Calculo do Impulso Escalar ($k$)

O valor da diferenca de velocidade e distribuido inversamente proporcional as massas das duas particulas:

$$
k = \frac{\Delta v_s}{m_a^{-1} + m_b^{-1}}
$$

O vetor impulso aplicado e $\mathbf{g} = k \cdot \mathbf{\hat{n}}$, resultando nas atualizacoes de velocidade:

$$
\mathbf{v}_a' = \mathbf{v}_a + \frac{k}{m_a} \mathbf{\hat{n}}, \quad \mathbf{v}_b' = \mathbf{v}_b - \frac{k}{m_b} \mathbf{\hat{n}}
$$

## 📐 3. Resolucao de Interpenetracao

Para evitar que particulas fiquem "presas" umas dentro das outras devido a erros de integracao numerica, o motor desloca fisicamente as particulas ao longo da normal de contato proporcionalmente a suas massas inversas:

$$
\Delta \mathbf{p}_a = \mathbf{\hat{n}} \cdot g \cdot \frac{m_a^{-1}}{m_a^{-1} + m_b^{-1}}, \quad \Delta \mathbf{p}_b = -\mathbf{\hat{n}} \cdot g \cdot \frac{m_b^{-1}}{m_a^{-1} + m_b^{-1}}
$$

## 🛠️ 4. O Resolvedor de Contatos (`ParticleContactResolver`)

Quando multiplas colisoes ocorrem ao mesmo tempo, resolve-las em uma ordem inadequada pode causar instabilidades.

* O `ParticleContactResolver` utiliza uma abordagem **iterativa gulosa (*greedy iterative approach*)**.
* A cada iterao, ele busca o contato com a **menor velocidade de separacao** (o impacto mais severo ou com maior penetracao) e o resolve.
* O processo se repete ate que todas as colisoes sejam resolvidas ou ate atingir o limite maximo de iteracoes configurado.