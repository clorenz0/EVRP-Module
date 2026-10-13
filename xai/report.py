"""
Informe de una Explanation: texto en consola, tablas CSV y graficos PNG.
Independiente de la tecnica: solo usa la interfaz de Explanation.
"""
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

LABELS = {
    "f_n_clients": "numero de clientes", "f_n_stations": "numero de estaciones", "f_n_vehicles": "numero de vehiculos",
    "f_vehicle_capacity": "capacidad del vehiculo", "f_demand_fill": "ocupacion de la flota (demanda/capacidad)",
    "f_depot_dist_mean": "distancia media deposito-cliente", "f_depot_dist_max": "distancia al cliente mas lejano",
    "f_nn_dist_mean": "distancia media al cliente mas cercano", "f_station_dist_mean": "distancia media a la estacion mas cercana",
    "f_station_density": "estaciones por cliente", "s_battery": "capacidad de bateria", "s_consumption": "consumo por km",
    "s_range_ratio": "alcance relativo (alcance / cliente mas lejano)",
}


def _label(name):
    if name in LABELS:
        return LABELS[name]
    if name.startswith("p_"):
        return f"parametro {name[2:]}"
    return name


def text_report(explanation, model, y, metrics, target, top=8, algorithm="ACO"):
    lines = [f"EXPLICACION GLOBAL ({explanation.method.upper()}) — {algorithm}, objetivo: {target}", ""]
    lines.append(f"Filas: {metrics['n_rows']} | variables: {metrics['n_features']} | media del objetivo: {y.mean():.2f}")
    lines.append("Fidelidad del modelo sustituto (de ella depende la validez de las explicaciones):")
    if "r2_cv" in metrics:
        lines.append(f"  R2 validacion cruzada aleatoria : {metrics['r2_cv']:.3f}  (MAE {metrics['mae_cv']:.2f})")
    if "r2_group" in metrics:
        lines.append(f"  R2 dejando una instancia fuera  : {metrics['r2_group']:.3f}  (generalizacion a instancias no vistas)")
    lines.append(f"  R2 en entrenamiento             : {metrics['r2_train']:.3f}")
    lines.append(f"  Error de aditividad de la explicacion: {explanation.additivity_error(model):.2e} (debe ser ~0)")
    lines += ["", f"Variables que mas influyen (media de |contribucion|, en unidades de {target}):"]
    importance = explanation.global_importance()
    total = importance.sum()
    arrows = {1: "un valor mayor tiende a AUMENTAR el resultado", -1: "un valor mayor tiende a REDUCIR el resultado",
              0: "efecto no monotono o mixto"}
    for name, value in importance.head(top).items():
        lines.append(f"  {name:<28}{value:>8.3f}  ({100 * value / total:>4.1f} %)  {_label(name)}: {arrows[explanation.direction(name)]}")
    return "\n".join(lines)


def local_text(explanation, model, y, i, top=5):
    pred = float(model.predict(explanation.data.iloc[[i]])[0])
    lines = [f"EXPLICACION DE LA CORRIDA #{i}: resultado real {y.iloc[i]:.2f} | prediccion del sustituto {pred:.2f} "
             f"| media del modelo {explanation.base_value:.2f}", "Contribuciones (positivo = sube el resultado):"]
    for name, value in explanation.local(i).head(top).items():
        lines.append(f"  {name:<28}{value:>+9.3f}   (valor {explanation.data.iloc[i][name]:.3f})  {_label(name)}")
    return "\n".join(lines)


def save_outputs(explanation, out_dir, top_dependence=3):
    """Guarda contribuciones (CSV) y graficos: barras de importancia, resumen y dependencias de las variables principales."""
    os.makedirs(out_dir, exist_ok=True)
    import pandas as pd
    pd.DataFrame(explanation.values, columns=explanation.feature_names).to_csv(
        os.path.join(out_dir, f"{explanation.method}_values.csv"), index=False)
    explanation.global_importance().to_csv(os.path.join(out_dir, f"{explanation.method}_importance.csv"), header=["importancia"])

    imp = explanation.global_importance().head(12)[::-1]
    plt.figure(figsize=(7, 4.5))
    plt.barh([_label(n) for n in imp.index], imp.values, color="#1F3A5F")
    plt.xlabel("media de |contribucion|")
    plt.title("Importancia global de las variables")
    plt.tight_layout(); plt.savefig(os.path.join(out_dir, f"{explanation.method}_importancia.png"), dpi=150); plt.close()

    if explanation.method == "shap":
        import shap
        shap.summary_plot(explanation.values, explanation.data, show=False, max_display=12)
        plt.tight_layout(); plt.savefig(os.path.join(out_dir, "shap_resumen.png"), dpi=150); plt.close()
        for name in list(explanation.global_importance().index[:top_dependence]):
            shap.dependence_plot(name, explanation.values, explanation.data, interaction_index=None, show=False)
            plt.tight_layout(); plt.savefig(os.path.join(out_dir, f"shap_dependencia_{name}.png"), dpi=150); plt.close()


def save_local_plot(explanation, i, out_dir):
    if explanation.method != "shap":
        return
    import shap
    shap.plots.waterfall(shap.Explanation(values=explanation.values[i], base_values=explanation.base_value,
                                          data=explanation.data.iloc[i].to_numpy(), feature_names=explanation.feature_names),
                         show=False)
    plt.tight_layout(); plt.savefig(os.path.join(out_dir, f"shap_corrida_{i}.png"), dpi=150, bbox_inches="tight"); plt.close()
