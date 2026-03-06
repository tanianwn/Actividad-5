# Actividad-3
# Robot Cartesiano 3GDL

## Sistema de referencia
<p align="center">
  <img src="https://github.com/user-attachments/assets/d1dc1293-3cec-44a9-b42c-86b106e3942e" width="600" alt="Robot Cartesiano 3GDL"/>
</p>

---

## L1 → L2

### Descripción
- Rotación negativa de -90° alrededor del eje X1
- Traslación negativa en el eje Z1

### Evaluación de la rotación fija

$$
R_{12} =
\begin{bmatrix}
1 & 0 & 0 \\
0 & 0 & 1 \\
0 & -1 & 0
\end{bmatrix}
$$

### Vector de traslación

$$
P_{12} =
\begin{bmatrix}
0 \\
0 \\
-L_1
\end{bmatrix}
$$

### Matriz de transformación homogénea

$$
T_{12} =
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 0 & 1 & 0 \\
0 & -1 & 0 & -L_1 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

---

## L2 → L3

### Descripción
- Rotación positiva de 90° alrededor del eje Y2
- Traslación positiva en el eje Z2

### Evaluación de la rotación fija

$$
R_{23} =
\begin{bmatrix}
0 & 0 & 1 \\
0 & 1 & 0 \\
-1 & 0 & 0
\end{bmatrix}
$$

### Vector de traslación

$$
P_{23} =
\begin{bmatrix}
0 \\
0 \\
L_2
\end{bmatrix}
$$

### Matriz de transformación homogénea

$$
T_{23} =
\begin{bmatrix}
0 & 0 & 1 & 0 \\
0 & 1 & 0 & 0 \\
-1 & 0 & 0 & L_2 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

---

## L3 → L4

### Descripción
- No existe rotación
- Traslación positiva en el eje Z3

### Matriz de rotación

$$
R_{34} =
\begin{bmatrix}
1 & 0 & 0 \\
0 & 1 & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

### Vector de traslación

$$
P_{34} =
\begin{bmatrix}
0 \\
0 \\
L_3
\end{bmatrix}
$$

### Matriz de transformación homogénea

$$
T_{34} =
\begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
0 & 0 & 1 & L_3 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

---

## Obtención de la Energía Cinética Total

### Paso 1 — Matrices de transformación homogénea globales

Se construyen las matrices locales $A_i$ y se acumulan para obtener las globales $T_i$:

$$T_1 = A_1, \quad T_2 = T_1 \cdot A_2, \quad T_3 = T_2 \cdot A_3$$

De cada $T_i$ se extraen la matriz de rotación global $R^O_i$ y la posición global $P^O_i$.

### Paso 2 — Jacobianos por eslabón

Como todas las juntas son **prismáticas** ($RP = [1,1,1]$), el jacobiano lineal de cada junta $k$ es la tercera columna de la rotación global anterior:

$$J_{v,k} = R^O_{k-1}(:,3)$$

Y el jacobiano angular es siempre cero:

$$J_{w,k} = \begin{bmatrix} 0 \\ 0 \\ 0 \end{bmatrix}$$

Se calcula un jacobiano distinto para cada eslabón usando solo las juntas que lo afectan:

$$J_3 \in \mathbb{R}^{3\times3}, \quad J_2 \in \mathbb{R}^{3\times2}, \quad J_1 \in \mathbb{R}^{3\times1}$$

### Paso 3 — Velocidades lineales y angulares

$$V_i = J_{v,i} \cdot \dot{Q}_i, \qquad W_i = J_{w,i} \cdot \dot{Q}_i = \vec{0}$$

Donde $\dot{Q} = [\dot{l}_1,\ \dot{l}_2,\ \dot{l}_3]^T$.

### Paso 4 — Vectores al centro de masa

La posición del centro de masa de cada eslabón se aproxima como la mitad del vector de posición local, sustituyendo $l_i$ por $l_{c_i}$:

$$P_{01} = \frac{P_1}{2}\bigg|_{l_1 \to l_{c1}}, \quad P_{12} = \frac{P_2}{2}\bigg|_{l_2 \to l_{c2}}, \quad P_{23} = \frac{P_3}{2}\bigg|_{l_3 \to l_{c3}}$$

### Paso 5 — Velocidad total en el centro de masa

$$V_{i,total} = V_i + W_i \times P_{i-1,i}$$

Como $W_i = \vec{0}$ para juntas prismáticas, queda simplemente $V_{i,total} = V_i$.

### Paso 6 — Energía cinética por eslabón

Para cada eslabón se aplica:

$$K_i = \frac{1}{2} m_i \, V_{i,total}^T V_{i,total} + \frac{1}{2} W_i^T I_i W_i$$

Donde $I_i$ es la matriz de inercia diagonal:

$$I_i =
\begin{bmatrix}
I_{xx_i} & 0 & 0 \\
0 & I_{yy_i} & 0 \\
0 & 0 & I_{zz_i}
\end{bmatrix}$$

### Paso 7 — Energía cinética total

$$\boxed{K_{Total} = K_1 + K_2 + K_3}$$

### Paso 8 — Energía potencial

Se toma la componente de la posición del centro de masa en la dirección de la gravedad para cada eslabón:

$$U_1 = m_1 g \, h_1, \quad U_2 = m_2 g \, h_2, \quad U_3 = m_3 g \, h_3$$

$$U_{Total} = U_1 + U_2 + U_3$$

### Paso 9 — Lagrangiano y Hamiltoniano

$$\mathcal{L} = K_{Total} - U_{Total}$$

$$H = K_{Total} + U_{Total}$$
