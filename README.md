# OmniSLAM: Graph-Based Dynamic Navigation System

![ROS 2](https://img.shields.io/badge/ROS2-Humble-22314E.svg) ![C++](https://img.shields.io/badge/C++-20-blue.svg) ![SLAM](https://img.shields.io/badge/SLAM-Graph--Based-green.svg)

**OmniSLAM** es un sistema de navegación autónoma completo diseñado para robótica móvil en entornos dinámicos (almacenes, hospitales). Implementa un backend de SLAM basado en grafos para resolver el problema de localización y mapeo simultáneo con precisión centimétrica.

## 🏛️ Arquitectura
El sistema opera sobre ROS 2 (Robot Operating System) y segrega la lógica en nodos de alto rendimiento:

1.  **Frontend (Sensor Fusion)**: Procesa raw data de LiDAR y Odometría de ruedas. Utiliza *Point-to-Line ICP* para estimar el movimiento relativo entre frames.
2.  **Backend (Pose Graph)**: Construye un grafo disperso donde los nodos son las poses del robot y las aristas son restricciones espaciales. Utiliza optimización de mínimos cuadrados no lineales para corregir el drift global.

## 🚀 Retos Técnicos Superados

### Corrección de Drift mediante Loop Closure
La odometría mecánica (encoders) sufre de error acumulativo ilimitado.
*   **Problema**: Tras 50m de recorrido, el robot cree estar 2m lejos de su posición real.
*   **Solución**: Implementé un detector de *Loop Closure* basado en histogramas de descriptores de escaneo. Cuando el robot vuelve a una zona conocida, se añade una "Arista de Cierre" al grafo. El backend (G2O/Custom Solver) utiliza esta restricción fuerte para "doblar" toda la trayectoria pasada, distribuyendo el error acumulado y cerrando el mapa de manera consistente.

## 📊 Análisis de Complejidad Computacional

### Optimización del Grafo
Resolver el sistema $H\Delta x = -b$ para encontrar la configuración óptima de poses.
*   **Matriz Hessiana ($H$)**: Es una matriz dispersa por bloques. Aunque el tamaño es $O(N^2)$, la estructura de banda permite usar descomposición *Sparse Cholesky*.
*   **Complejidad**: Típicamente $O(N^{1.5})$ en 2D, donde $N$ es el número de poses.
*   **Marginalización**: Para mantener la operación en tiempo real ($O(1)$ amortizado), las poses antiguas lejos de la ventana activa se marginalizan (Schur Complement) o se fijan, evitando que el grafo crezca indefinidamente en memoria operativa.

## 🛠️ Build & Run (Docker)

```bash
# Construir la imagen con ROS 2 Humble
docker build -t omnislam -f Dockerfile .

# Ejecutar tests
docker run omnislam colcon test --packages-select omnislam_core
```
