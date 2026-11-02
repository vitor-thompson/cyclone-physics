# Capitulo 13: Resolucao de Contatos (Contact Resolution)

Este documento registra os fundamentos da resolucao de contatos de corpos rigidos no motor **Cyclone**, correspondente ao **Capitulo 13** do livro *Game Physics Engine Development* (Ian Millington).

O Capitulo 13 aborda a matematica dos impulsos em corpos rigidos, conectando alteracoes instantaneas de velocidade linear e angular a contatos localizados no espaco 3D.

## 💥 1. A Matematica do Impulso em Corpos Rigidos

Ao contrario de uma particula (onde um impulso atua apenas na velocidade linear), um impulso $\mathbf{g}$ aplicado a um corpo rigido em um ponto relativo $\mathbf{r} = \mathbf{p}_{\text{contato}} - \mathbf{p}_{\text{centro}}$ altera simultaneamente sua **velocidade linear** e sua **velocidade angular**.

### Mudancas de Velocidade Geradas por um Impulso:

1. **Variacao de Velocidade Linear:**

   $$
   \Delta \mathbf{v} = \mathbf{g} \cdot w
   $$

2. **Variacao de Velocidade Angular:**

   $$
   \Delta \boldsymbol{\omega} = \mathbf{I}_{\text{world}}^{-1} (\mathbf{r} \times \mathbf{g})
   $$

### Velocidade no Ponto de Contato:

A velocidade total de um ponto de contato $\mathbf{p}_c$ em um corpo rigido e a soma da sua velocidade linear com a velocidade tangencial gerada pela rotacao:

$$
\mathbf{v}_c = \mathbf{v} + \boldsymbol{\omega} \times \mathbf{r}
$$

## 📐 2. Calculo da Mudanca de Velocidade por Unidade de Impulso

Para resolver uma colisaes, precisamos determinar qual magnitude de impulso escalar $k$ aplicado ao longo da normal de contato $\mathbf{\hat{n}}$ produzira a mudanca de velocidade de separacao desejada $\Delta v_s$.

A mudanca na velocidade do ponto por unidade de impulso ao longo da normal e dada por:

$$
\Delta v_s = \left[ w_a + w_b + \left( (\mathbf{I}_{a}^{-1} (\mathbf{r}_a \times \mathbf{\hat{n}})) \times \mathbf{r}_a + (\mathbf{I}_{b}^{-1} (\mathbf{r}_b \times \mathbf{\hat{n}})) \times \mathbf{r}_b \right) \cdot \mathbf{\hat{n}} \right] k
$$

Definindo o termo entre colchetes como a **massa delta inversa do contato** ($m_{\Delta}^{-1}$):

$$
k = \frac{\Delta v_s}{m_{\Delta}^{-1}}
$$

## ⚡ 3. Algoritmo Basico de Resolucao por Impulso

O motor utiliza uma abordagem de resolucao **iterativa em cadeia**:

1. **Calculo da Velocidade de Separacao Inicial:**

   $$
   v_s = (\mathbf{v}_{cA} - \mathbf{v}_{cB}) \cdot \mathbf{\hat{n}}
   $$

2. **Verificacao:** Se $v_s \ge 0$, os corpos ja estao se separando e nao precisam de ajuste de velocidade.

3. **Velocidade Alvo:**

   $$
   v_s' = -e \cdot v_s
   $$

4. **Aplicacao do Impulso:** Calcula-se $k$ e aplicam-se as mudancas de velocidade linear e angular correspondentes a ambos os corpos envolvidos.

5. **Recalculo dos Contatos Afetados:** Como a resolucao de um contato altera as velocidades dos corpos, os outros contatos associados a esses corpos sao atualizados na mesma iteração.