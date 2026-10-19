"""
Explicadores. Todos implementan la misma interfaz:

    explainer = EXPLAINERS["shap"]()
    explanation = explainer.explain(model, X)      # -> Explanation

Explanation guarda, para cada fila de X, la contribucion de cada variable a la prediccion
(base_value + suma de contribuciones = prediccion). Asi las tecnicas futuras (LIME, sensibilidad
global con SALib, contrafactuales, ...) se agregan como una clase mas y reutilizan el mismo informe.
"""
from dataclasses import dataclass

import numpy as np
import pandas as pd


@dataclass
class Explanation:
    method: str
    feature_names: list
    values: np.ndarray        # (n_filas, n_variables): contribucion de cada variable
    base_value: float         # prediccion media del modelo
    data: pd.DataFrame        # las filas explicadas

    def global_importance(self):
        """Importancia global = media del valor absoluto de la contribucion, de mayor a menor."""
        imp = pd.Series(np.abs(self.values).mean(axis=0), index=self.feature_names)
        return imp.sort_values(ascending=False)

    def local(self, i):
        """Contribuciones de la fila i, ordenadas por magnitud."""
        s = pd.Series(self.values[i], index=self.feature_names)
        return s.reindex(s.abs().sort_values(ascending=False).index)

    def direction(self, feature):
        """+1 / -1 / 0: un valor mayor de la variable tiende a SUBIR / BAJAR / no mueve de forma consistente el resultado."""
        j = self.feature_names.index(feature)
        x, v = self.data[feature].to_numpy(dtype=float), self.values[:, j]
        if np.std(x) == 0 or np.std(v) == 0:
            return 0
        corr = float(np.corrcoef(x, v)[0, 1])
        return 0 if abs(corr) < 0.2 else (1 if corr > 0 else -1)

    def additivity_error(self, model):
        """Maximo |base + suma(contribuciones) - prediccion|: comprueba que la explicacion es fiel al modelo."""
        pred = model.predict(self.data)
        return float(np.max(np.abs(self.base_value + self.values.sum(axis=1) - pred)))


class BaseExplainer:
    name = "base"

    def explain(self, model, X):
        raise NotImplementedError


class ShapExplainer(BaseExplainer):
    """SHAP (valores de Shapley). Con arboles se usa TreeSHAP: exacto y rapido, sin re-ejecutar el solver."""
    name = "shap"

    def explain(self, model, X):
        try:
            import shap
        except ImportError as error:
            raise SystemExit("Falta la libreria shap: pip install shap") from error
        explainer = shap.TreeExplainer(model)
        values = np.asarray(explainer.shap_values(X))
        base = float(np.ravel(explainer.expected_value)[0])
        return Explanation("shap", list(X.columns), values, base, X.reset_index(drop=True))


# Registro: aqui se anaden tecnicas futuras (p. ej. "lime": LimeExplainer, "sobol": SobolExplainer)
EXPLAINERS = {"shap": ShapExplainer}
