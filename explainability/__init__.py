"""
explainability — modulo de explicabilidad del EVRP-Module (version inicial).

Idea general
------------
Las metaheuristicas (ACO hoy, otra poblacional manana) y el simulador son cajas negras
costosas: evaluarlas miles de veces para explicar una sola corrida es inviable. Por eso
el modulo trabaja en tres pasos independientes:

  1. DATOS      (explainability.dataset)    Corre el algoritmo muchas veces variando escenario y parametros
                                 y guarda una tabla: una fila por corrida, con
                                 caracteristicas de la instancia, parametros del algoritmo y resultado.
  2. SUSTITUTO  (explainability.surrogate)  Entrena un modelo rapido (Random Forest) que aprende resultado = f(fila).
  3. EXPLICADOR (explainability.explainers) Explica el sustituto. Hoy SHAP; manana LIME, sensibilidad global, etc.
                                 Todos comparten la misma interfaz (BaseExplainer.explain).

Para agregar otra metaheuristica: registrarla en explainability/algorithms.py (nombre, espacio de
parametros y funcion que la ejecuta). Para agregar otra tecnica de XAI: una clase en
explainability/explainers.py y una linea en EXPLAINERS. El resto no cambia.

Lo que se explica es el COMPORTAMIENTO GENERAL del algoritmo (que variables de la instancia y
del algoritmo empujan el resultado hacia arriba o abajo), no la comparacion con otro solver.
"""
