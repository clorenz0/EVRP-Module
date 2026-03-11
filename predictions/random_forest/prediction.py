import pandas as pd
import joblib


def predict_with_trained_model(new_data: pd.DataFrame, key: str) -> pd.DataFrame:
    key_upper = key.upper()
    base_path = f"./results/results_{key}/"

    model = joblib.load(f"{base_path}random_forest_model_{key_upper}.pkl")
    preprocessor = joblib.load(f"{base_path}preprocessor_{key_upper}.pkl")
    selector = joblib.load(f"{base_path}selector_{key_upper}.pkl")

    x_processed = preprocessor.transform(new_data)
    x_selected = selector.transform(x_processed)
    predictions = model.predict(x_selected)

    new_data["Predicted_Method"] = predictions
    return new_data


def run_predictions(problems: list[str]):
    input_data = {
        "tsp": [["manhattan", 16], ["Distance", "Nodes"]],
        "cvrp": [["haversine", 100, 200, 17.93, 401, 0.36],
                 ["Distance", "Vehicles", "Vehicles Capacity", "Demands", "Nodes", "Load Factor"]],
        "vrptw": [["euclidean", 100, 200, 17.93, 401, 0.36, 357.47, 738.55],
                  ["Distance", "Vehicles", "Vehicles Capacity", "Demands", "Nodes", "Load Factor", "Avg TW Start", "Avg TW End"]],
        "mdvrp": [["euclidean", 6, 190, 13.37, 150, 1.76, 12.3],
                  ["Distance", "Vehicles", "Vehicles Capacity", "Demands", "Nodes", "Load Factor", "Avg Depot-Client Distance"]],
        "vrppd": [["haversine", 5, 100, 15.35, 51, 1.55, 4.34],
                  ["Distance", "Vehicles", "Vehicles Capacity", "Demands", "Nodes", "Load Factor", "Avg Pickup-Delivery Distance"]],
    }

    for problem in problems:
        key = problem.lower()
        if key in input_data:
            data_values, columns = input_data[key]
            new_data = pd.DataFrame([data_values], columns=columns)
            result = predict_with_trained_model(new_data, key)
            print(f"\nResults for {problem.upper()}:\n{result}")
        else:
            print(f"\n[WARNING] No input data defined for problem: {problem}")


if __name__ == "__main__":
    problem_list = ["TSP", "CVRP", "VRPTW", "MDVRP", "VRPPD"]
    run_predictions(problem_list)
