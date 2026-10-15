"""
Modelo sustituto (surrogate): aprende resultado = f(instancia, escenario, parametros del algoritmo)
a partir de la tabla generada por explainability.dataset. Es lo que se explica en vez de re-correr el solver.

Importante para la tesis: las explicaciones solo valen tanto como la fidelidad del sustituto, asi que
train_surrogate devuelve tambien metricas de validacion cruzada:
  - r2_cv      : R2 con validacion cruzada aleatoria (interpola dentro de lo ya visto).
  - r2_group   : R2 dejando una INSTANCIA fuera cada vez (generaliza a una instancia nueva?). Con pocas
                 instancias sera bajo: es esperable y debe reportarse tal cual.
"""
import pandas as pd

FEATURE_PREFIXES = ("f_", "s_", "p_")


def load_dataset(csv_path, target="distance", algorithm=None, drop_dropped=True):
    """Devuelve X (variables), y (objetivo), groups (instancia) y la tabla filtrada."""
    df = pd.read_csv(csv_path)
    df = df[df["status"] == "ok"].copy()
    if algorithm:
        df = df[df["algorithm"] == algorithm]
    if drop_dropped:                      # corridas con clientes descartados distorsionan el objetivo
        df = df[df["n_dropped"] == 0]
    if df.empty:
        raise SystemExit("No hay filas validas en el CSV (revisa status / n_dropped).")

    if target == "distance_per_client":   # objetivo normalizado por tamaño de instancia
        y = df["distance"] / df["f_n_clients"]
    else:
        y = df[target]
    cols = [c for c in df.columns if c.startswith(FEATURE_PREFIXES)]
    X = df[cols].astype(float)
    if df["algorithm"].nunique() > 1:     # varias metaheuristicas en la misma tabla: el algoritmo es una variable mas
        for name in sorted(df["algorithm"].unique()):
            X[f"a_{name}"] = (df["algorithm"] == name).astype(float)
    X = X.loc[:, X.nunique() > 1]         # una variable constante no explica nada
    return X.reset_index(drop=True), y.reset_index(drop=True), df["instance"].reset_index(drop=True), df.reset_index(drop=True)


def train_surrogate(X, y, groups, n_estimators=300, random_state=0):
    """Entrena un Random Forest y mide su fidelidad. Devuelve (modelo, metricas)."""
    from sklearn.ensemble import RandomForestRegressor
    from sklearn.metrics import mean_absolute_error, r2_score
    from sklearn.model_selection import GroupKFold, KFold, cross_val_predict

    model = RandomForestRegressor(n_estimators=n_estimators, min_samples_leaf=2,
                                  random_state=random_state, n_jobs=-1)
    metrics = {"n_rows": len(X), "n_features": X.shape[1]}

    n_splits = min(5, len(X))
    if n_splits >= 3:
        pred = cross_val_predict(model, X, y, cv=KFold(n_splits, shuffle=True, random_state=random_state))
        metrics["r2_cv"] = float(r2_score(y, pred))
        metrics["mae_cv"] = float(mean_absolute_error(y, pred))
    n_groups = groups.nunique()
    if n_groups >= 3:
        pred_g = cross_val_predict(model, X, y, cv=GroupKFold(min(5, n_groups)), groups=groups)
        metrics["r2_group"] = float(r2_score(y, pred_g))

    model.fit(X, y)
    metrics["r2_train"] = float(r2_score(y, model.predict(X)))
    return model, metrics
