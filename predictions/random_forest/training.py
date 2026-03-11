from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import train_test_split, StratifiedKFold, RandomizedSearchCV, cross_val_score
from sklearn.metrics import balanced_accuracy_score, cohen_kappa_score, f1_score, precision_score, recall_score, \
    roc_auc_score, accuracy_score, classification_report
import pandas as pd
import joblib
from sklearn.preprocessing import label_binarize
from utils.utils import get_data_for_predictions
from sklearn.tree import export_graphviz
from PIL import Image
import plotly.express as px
from imblearn.over_sampling import SMOTE
from collections import Counter
from scipy.stats import randint
import os
from sklearn.feature_selection import SelectKBest, f_classif
from sklearn.preprocessing import OneHotEncoder
from sklearn.compose import ColumnTransformer
import re
import html


def save_results_to_txt(filename, accuracy, balanced_acc, kappa, f1, precision, recall, roc_auc,
                        best_params, cv_scores, classification_rep, importance_df):
    with open(filename, "w") as f:
        f.write(f"Accuracy: {accuracy:.4f}\n\n")
        f.write(f"Balanced Accuracy: {balanced_acc:.4f}\n\n")
        f.write(f"Cohen's Kappa: {kappa:.4f}\n\n")
        f.write(f"F1 Score: {f1:.4f}\n\n")
        f.write(f"Precision: {precision:.4f}\n\n")
        f.write(f"Recall: {recall:.4f}\n\n")
        f.write(f"ROC-AUC: {roc_auc:.4f}\n\n")
        f.write("Best Parameters:\n")
        f.write(f"{best_params}\n\n")
        f.write("Cross-Validation Scores:\n")
        f.write(f"{cv_scores}\n")
        f.write(f"Mean Cross-Validation Accuracy: {cv_scores.mean():.4f}\n\n")
        f.write("Classification Report:\n")
        f.write(f"{classification_rep}\n\n")
        f.write("Feature Importances:\n")
        f.write(importance_df.to_string(index=False))


def get_random_forest():
    results = get_data_for_predictions()
    for key, data in results.items():
        dataset = pd.DataFrame(data)
        dataset.dropna(subset=["Method"], inplace=True)

        optimal_methods = dataset.groupby(["Instance", "Distance"]).apply(
            lambda x: x.loc[x.sort_values(by=["Objective", "Time"]).index[0], "Method"],
            include_groups=False
        ).reset_index(name="Optimal_Method")

        dataset = dataset.merge(optimal_methods, on=["Instance", "Distance"])

        columns_set = (
            ["Method", "Instance", "Objective", "Time", "Routes", "Optimal_Method"]
            if key != "TSP"
            else ["Method", "Instance", "Objective", "Time", "Optimal_Method"]
        )

        x = dataset.drop(columns=columns_set)
        y = dataset["Optimal_Method"]

        numeric_features = x.select_dtypes(include=['int64', 'float64']).columns.tolist()
        categorical_features = ['Distance']

        preprocessor = ColumnTransformer(
            transformers=[
                ('num', 'passthrough', numeric_features),
                ('cat', OneHotEncoder(handle_unknown='ignore'), categorical_features)
            ]
        )

        x_processed = preprocessor.fit_transform(x)

        encoder = preprocessor.named_transformers_['cat']
        distance_categories = encoder.categories_[0]
        distance_feature_names = [f"Distance_{cat}" for cat in distance_categories]

        numeric_feature_names = numeric_features.copy()
        feature_names = numeric_feature_names + distance_feature_names

        class_distribution = Counter(y)
        print(f"Class distribution for {key}: {class_distribution}")

        classes_to_keep = [cls for cls, count in class_distribution.items() if count >= 2]
        if len(classes_to_keep) < len(class_distribution):
            print(f"Warning: Some classes have less than 2 samples in {key}. Removing those classes.")
            mask = y.isin(classes_to_keep)
            x_processed = x_processed[mask]
            y = y[mask]

        if len(y) == 0:
            print(f"Error: Not enough samples to train the model in {key}. Skipping this dataset.")
            continue

        x_train, x_test, y_train, y_test = train_test_split(
            x_processed, y, test_size=0.2, random_state=42, stratify=y
        )

        smote = SMOTE(random_state=42)
        x_train_smote, y_train_smote = smote.fit_resample(x_train, y_train)

        selector = SelectKBest(f_classif, k=min(10, x_train_smote.shape[1]))
        x_train_selected = selector.fit_transform(x_train_smote, y_train_smote)
        x_test_selected = selector.transform(x_test)

        selected_feature_indices = selector.get_support(indices=True)
        selected_feature_names = [feature_names[i] for i in selected_feature_indices]

        param_dist = {
            "n_estimators": randint(200, 800),
            "max_depth": [5, 10, 15, 20, 25],
            "min_samples_split": randint(5, 20),
            "min_samples_leaf": randint(2, 10),
            "bootstrap": [True],
            "max_features": ['sqrt', 'log2'],
            "criterion": ['gini', 'entropy']
        }

        random_search = RandomizedSearchCV(
            RandomForestClassifier(class_weight="balanced", random_state=42),
            param_distributions=param_dist,
            n_iter=100,
            cv=5,
            scoring="accuracy",
            n_jobs=-1,
            verbose=2,
            random_state=42
        )

        random_search.fit(x_train_selected, y_train_smote)
        model = random_search.best_estimator_

        joblib.dump(model, f"random_forest_model_{key}.pkl")
        joblib.dump(preprocessor, f"preprocessor_{key}.pkl")
        joblib.dump(selector, f"selector_{key}.pkl")

        y_pred = model.predict(x_test_selected)

        instances_test = dataset.loc[y_test.index, "Instance"].values
        true_methods = y_test.values
        predicted_methods = y_pred

        results_by_instance = pd.DataFrame({
            "Instance": instances_test,
            "True_Optimal_Method": true_methods,
            "Predicted_Method": predicted_methods
        })

        results_by_instance["Correct"] = results_by_instance["True_Optimal_Method"] == results_by_instance[
            "Predicted_Method"]

        features = dataset.loc[y_test.index].drop(
            columns=["Method"])
        results_by_instance = pd.concat([results_by_instance.reset_index(drop=True), features.reset_index(drop=True)],
                                        axis=1)
        summary_txt_path = f"predictions_summary_{key}.txt"
        with open(summary_txt_path, "w") as f:
            f.write(f"Per-instance prediction results for {key} saved to predictions_by_instance_{key}.csv\n")
            f.write("First few predictions:\n")
            f.write(results_by_instance.head().to_string(index=False))

        print(f"Summary saved to {summary_txt_path}")

        balanced_acc = balanced_accuracy_score(y_test, y_pred)
        kappa = cohen_kappa_score(y_test, y_pred)
        f1 = f1_score(y_test, y_pred, average="weighted", zero_division=0)
        precision = precision_score(y_test, y_pred, average="weighted", zero_division=0)
        recall = recall_score(y_test, y_pred, average="weighted", zero_division=0)

        classes = model.classes_
        y_test_bin = label_binarize(y_test, classes=classes)
        y_pred_bin = label_binarize(y_pred, classes=classes)

        roc_auc = roc_auc_score(y_test_bin, y_pred_bin, average="macro", multi_class="ovo")

        classification_rep = classification_report(y_test, y_pred, zero_division=0)

        accuracy = accuracy_score(y_test, y_pred)
        print(f"Accuracy for {key}: {accuracy:.4f}")

        cv = StratifiedKFold(n_splits=2, shuffle=True, random_state=42)
        cv_scores = cross_val_score(model, x_processed, y, cv=cv, scoring="accuracy")
        print(f"Cross-validation scores for {key}: {cv_scores}")
        print(f"Cross-validation mean accuracy for {key}: {cv_scores.mean():.4f}")

        print("Classification Report:")
        print(classification_report(y_test, y_pred, zero_division=0))

        importances = model.feature_importances_
        importance_df = pd.DataFrame({
            "Feature": selected_feature_names,
            "Importance": importances
        }).sort_values(by="Importance", ascending=False)

        importance_df = importance_df[~importance_df["Feature"].str.startswith("Distance_")]

        importance_df = importance_df[importance_df["Importance"] > 0.01]

        print(f"Feature Importances for {key}:")
        print(importance_df)

        save_results_to_txt(
            f"results_{key}.txt",
            accuracy, balanced_acc, kappa, f1, precision, recall, roc_auc,
            random_search.best_params_,
            cv_scores, classification_rep, importance_df
        )

        fig = px.bar(importance_df, x="Feature", y="Importance", title=f"Feature Importances ({key})")
        fig.update_layout(xaxis_title="Features", yaxis_title="Importance", xaxis_tickangle=-90)
        fig.write_image(f"feature_importances_{key}.jpg")

        try:
            estimator = model.estimators_[0]
            dot_data = export_graphviz(
                estimator,
                out_file=None,
                feature_names=selected_feature_names,
                class_names=model.classes_,
                filled=True,
                rounded=True,
                special_characters=True,
                impurity=False,
                proportion=True,
                precision=0,
                node_ids=True,
                rotate=False,
            )

            dot_data = html.unescape(dot_data)

            distance_pattern = re.compile(
                r"Distance_([a-zA-Z]+)\s*(?:&le;|<=|≤|&#8804;|&amp;le;)\s*([-+]?\d*\.?\d+)",
                re.IGNORECASE
            )

            def replace_distance_condition(match):
                try:
                    category = match.group(1).lower()
                    threshold = float(match.group(2))
                    print(f"Coincidencia encontrada: {match.group(0)} -> categoría: {category}, threshold: {threshold}")
                    if threshold < 0.5:
                        replacement = f"Distance != '{category}'"
                    else:
                        replacement = f"Distance == '{category}'"
                    print(f"Reemplazo: {replacement}")
                    return replacement
                except Exception as err:
                    print(f"Error al procesar la coincidencia {match.group(0)}: {err}")
                    return match.group(0)

            matches = list(distance_pattern.finditer(dot_data))
            print(f"{len(matches)} coincidencia(s) encontradas:")
            for m in matches:
                print(f" - {m.group(0)}")

            dot_data_converted = distance_pattern.sub(replace_distance_condition, dot_data)

            # Guardar archivo DOT
            dot_file = f"tree_{key}.dot"
            with open(dot_file, "w") as f:
                f.write(dot_data_converted)

            # Generar imagen PNG
            png_file = f"tree_{key}.png"
            result = os.system(f"dot -Tpng {dot_file} -o {png_file}")
            if result != 0:
                print("Error ejecutando Graphviz (dot). ¿Está instalado correctamente y en el PATH?")

            if os.path.exists(png_file):
                print(f"Visualización del árbol guardada como: {png_file}")
                img = Image.open(png_file)
                img.show()
            else:
                print(f"Error: No se pudo generar el archivo {png_file}")

        except Exception as e:
            print(f"Error visualizando el árbol para {key}: {e}")


if __name__ == "__main__":
    get_random_forest()
