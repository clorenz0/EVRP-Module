import os
import re
import csv
import pandas as pd
from collections import defaultdict


def get_cost(data, routes_count, cost_matches):
    cost = 0
    for load in data:
        if load != '0':
            routes_count += 1
            if not cost_matches:
                cost += int(load)
    return cost, routes_count


def extract_info_from_txt(file_path):
    routes_count = 0
    execution_time = 0
    cost = 0
    objective = None
    instance = None

    with open(file_path, 'r') as file:
        content = file.read()
        instance_match = re.search(r'Instance:\s*(\S+)', content)
        objective_match = re.search(r'Objective:\s*(\d+)', content)
        time_match = re.search(r'Execution Time:\s*([\d\.]+)', content)
        load_matches = re.findall(r'Distance of the route:\s*(\d+)', content)
        cost_matches = re.findall(r'Total Distance of all routes:\s*(\d+)', content)
        other_cost_matches = re.findall(r'Total load of all routes:\s*(\d+)', content)
        load_route_matches = re.findall(r'Load of the route:\s*(\d+)', content)
        if load_matches:
            cost, routes_count = get_cost(load_matches, routes_count, cost_matches)
        if not cost and load_route_matches:
            cost, routes_count = get_cost(load_route_matches, routes_count, cost_matches)
        if instance_match:
            instance = instance_match.group(1)
        if objective_match:
            objective = int(objective_match.group(1))
        if time_match:
            execution_time = float(time_match.group(1))
        if cost_matches:
            cost = int(cost_matches[0])
        if objective != 0:
            if cost != objective and cost != 0:
                objective = cost
        elif other_cost_matches:
            objective = other_cost_matches[0]
        return instance, objective, execution_time, (routes_count or 1)
    return None, None, None, None


def obtain_technique(f_path, f_name):
    heuristic = None
    metaheuristic = None
    for file_name in os.listdir(f_path):
        file_path = os.path.join(f_path, file_name)
        with open(file_path, 'r') as file:
            for line in file:
                if 'Heuristic:' in line:
                    heuristic = line.split(':')[1].strip()
                elif 'Metaheuristic:' in line:
                    metaheuristic = line.split(':')[1].strip()
        if heuristic == 'None':
            metaheuristic = f_name
        elif metaheuristic == 'None':
            heuristic = f_name
    return heuristic, metaheuristic


def process_solutions_folder(base_folder):
    results = []
    for folder_name in os.listdir(base_folder):
        folder_path = os.path.join(base_folder, folder_name)
        if os.path.isdir(folder_path):
            for f_name in os.listdir(folder_path):
                f_path = os.path.join(folder_path, f_name)
                if os.path.isdir(f_path):
                    f_name = f_name[len("solutions_"):]
                    if '&' in f_name:
                        heuristic, metaheuristic = map(str.strip, f_name.split('_&_'))
                    elif f_name.replace('_', '').isdigit():
                        heuristic, metaheuristic = map(int, f_name.split('_'))
                    else:
                        heuristic, metaheuristic = obtain_technique(f_path, f_name)
                    for file_name in os.listdir(f_path):
                        if file_name.endswith('.txt'):
                            file_path = os.path.join(f_path, file_name)
                            instance, objective, execution_time, routes_count = extract_info_from_txt(file_path)
                            if instance and objective is not None:
                                results.append(
                                    (instance, objective, heuristic, metaheuristic, execution_time, routes_count))
    return results


def filter_average_solutions_per_algorithm(results):
    average_results = {}
    for instance, objective, heuristic, metaheuristic, execution_time, routes_count in results:
        algorithm = heuristic.strip() + "_and_" + metaheuristic.strip()

        if instance not in average_results:
            average_results[instance] = {}

        if algorithm not in average_results[instance]:
            average_results[instance][algorithm] = {
                "objectives": [],
                "execution_times": [],
                "routes_counts": [],
            }
        try:
            average_results[instance][algorithm]["objectives"].append(float(objective))
            average_results[instance][algorithm]["execution_times"].append(float(execution_time))
            average_results[instance][algorithm]["routes_counts"].append(int(routes_count))
        except ValueError:
            average_results[instance][algorithm]["objectives"].append(None)
            average_results[instance][algorithm]["execution_times"].append(None)
            average_results[instance][algorithm]["routes_counts"].append(0)

    final_average_results = []
    all_algorithms = set()
    for instance_data in average_results.values():
        all_algorithms.update(instance_data.keys())

    for instance, algorithms in average_results.items():
        max_objective = 0.0
        max_execution_time = 0.0
        for algorithm_data in algorithms.values():
            valid_objectives = [obj for obj in algorithm_data["objectives"] if obj is not None]
            if valid_objectives:
                max_objective = max(max_objective, max(valid_objectives))
            valid_execution_times = [time for time in algorithm_data["execution_times"] if time is not None]
            if valid_execution_times:
                max_execution_time = max(max_execution_time, max(valid_execution_times))
        if max_objective > 0:
            penalty_value_objective = max_objective * 10
        else:
            penalty_value_objective = 1000

        if max_execution_time > 0:
            penalty_value_execution_time = max_execution_time * 10
        else:
            penalty_value_execution_time = 1000

        for algorithm in all_algorithms:
            if algorithm not in algorithms:
                avg_objective = penalty_value_objective
                avg_execution_time = penalty_value_execution_time
                avg_routes_count = 0
            else:
                data = algorithms[algorithm]
                valid_objectives = [obj for obj in data["objectives"] if obj is not None]
                avg_objective = sum(valid_objectives) / len(
                    valid_objectives) if valid_objectives else penalty_value_objective
                valid_execution_times = [time for time in data["execution_times"] if time is not None]
                avg_execution_time = sum(valid_execution_times) / len(
                    valid_execution_times) if valid_execution_times else penalty_value_execution_time
                avg_routes_count = sum(data["routes_counts"]) / len(data["routes_counts"]) if data[
                    "routes_counts"] else 0
            final_average_results.append(
                (
                    instance,
                    avg_objective,
                    algorithm.split("and")[0],
                    algorithm.split("and")[1],
                    algorithm,
                    avg_execution_time,
                    avg_routes_count,
                )
            )

    return final_average_results


def filter_best_solutions(results):
    best_results = {}

    for instance, objective, heuristic, metaheuristic, execution_time, routes_count in results:
        try:
            obj_value = float(objective) if isinstance(objective, str) else objective
        except (ValueError, TypeError):
            continue

        if obj_value != 0:
            if instance not in best_results:
                best_results[instance] = [
                    (instance, obj_value, heuristic, metaheuristic, execution_time, routes_count)]
            else:
                best_results[instance].append(
                    (instance, obj_value, heuristic, metaheuristic, execution_time, routes_count))

    final_best_results = {}
    for instance, candidates in best_results.items():
        min_objective = min(candidates, key=lambda x: x[1])[1]
        min_candidates = [c for c in candidates if c[1] == min_objective]
        if len(min_candidates) > 1:
            best_candidate = min(min_candidates, key=lambda x: x[4])
        else:
            best_candidate = min_candidates[0]
        final_best_results[instance] = best_candidate

    return final_best_results


def generate_csv_files(results, best_solution, output_folder, filtered=None):
    if best_solution:
        results = filter_average_solutions_per_algorithm(results)
    data = []
    for row in results:
        instance = row[0].split(".")[0]
        heuristic = row[2]
        metaheuristic = row[3]
        objective = row[1]
        execution_time = row[5]
        algorithm = row[4]
        data.append([instance, objective, execution_time, heuristic, metaheuristic, algorithm])

    df = pd.DataFrame(
        data,
        columns=["Instance", "Objective", "Time", "Heuristic", "Metaheuristic", "Algorithm"]
    )
    if not filtered:
        metaheuristics = df["Metaheuristic"].unique()
        for metaheuristic in metaheuristics:
            df_meta = df[df["Metaheuristic"] == metaheuristic]
            objective_dict = {"Instance": []}
            time_dict = {"Instance": []}
            algorithms = df_meta["Algorithm"].unique()
            for algorithm in algorithms:
                objective_dict[algorithm] = []
                time_dict[algorithm] = []
            for instance in df_meta["Instance"].unique():
                objective_dict["Instance"].append(instance)
                time_dict["Instance"].append(instance)
                for algorithm in algorithms:
                    objective_value = df_meta.loc[
                        (df_meta["Algorithm"] == algorithm) & (df_meta["Instance"] == instance), "Objective"]
                    objective_dict[algorithm].append(objective_value.iloc[0] if not objective_value.empty else "")
                    time_value = df_meta.loc[
                        (df_meta["Algorithm"] == algorithm) & (df_meta["Instance"] == instance), "Time"]
                    time_dict[algorithm].append(time_value.iloc[0] if not time_value.empty else "")
            df_objective = pd.DataFrame(objective_dict)
            df_time = pd.DataFrame(time_dict)
            objective_file = f"{output_folder}/{metaheuristic}_objective.csv"
            time_file = f"{output_folder}/{metaheuristic}_time.csv"
            df_objective.to_csv(objective_file, sep=";", index=False)
            df_time.to_csv(time_file, sep=";", index=False)
            print(f"Generated files: {objective_file}, {time_file}")
    else:
        df_filtered = df[df["Algorithm"].isin(filtered)]
        result_dict = {"Instance": []}
        for algorithm in filtered:
            result_dict[algorithm] = []
        for instance in df_filtered["Instance"].unique():
            result_dict["Instance"].append(instance)
            for algorithm in filtered:
                value = df_filtered.loc[
                    (df_filtered["Algorithm"] == algorithm) & (df_filtered["Instance"] == instance), "Time"]
                result_dict[algorithm].append(value.iloc[0] if not value.empty else "")
        df_result = pd.DataFrame(result_dict)
        file = f"{output_folder}/best_algorithms_time_chebyshev.csv"
        df_result.to_csv(file, sep=";", index=False)
        print(f"Archivo generado: {file}")


def write_solutions(output_file, results, best_solution, is_csv, filtered):
    if not is_csv:
        with open(output_file, 'w', newline='') as file:
            writer = csv.writer(file, delimiter=';')
            writer.writerow(['Instance', 'Objective', 'Heuristic', 'Metaheuristic', 'Time', 'Routes'])
            if best_solution:
                results = filter_best_solutions(results).values()
            for row in results:
                writer.writerow(row)
    else:
        generate_csv_files(results, best_solution, output_file, filtered)


def compute_average_objectives(instance_name, base_path, output_file):
    results = defaultdict(list)

    for exec_folder in os.listdir(base_path):
        exec_path = os.path.join(base_path, exec_folder)
        if not os.path.isdir(exec_path):
            continue

        for algo_folder in os.listdir(exec_path):
            algo_path = os.path.join(exec_path, algo_folder)
            if not os.path.isdir(algo_path):
                continue

            for file_name in os.listdir(algo_path):
                if file_name.endswith(".txt"):
                    file_path = os.path.join(algo_path, file_name)
                    with open(file_path, "r") as f:
                        content = f.read()
                        if f"Instance: {instance_name}" not in content:
                            continue

                        lines = content.splitlines()
                        data = {}
                        for line in lines:
                            if ":" in line:
                                key, value = line.split(":", 1)
                                data[key.strip()] = value.strip()

                        try:
                            heuristic = data["Heuristic"]
                            metaheuristic = data["Metaheuristic"]
                            algorithm = f"{heuristic}_and_{metaheuristic}"
                            objective = float(data["Objective"])
                            results[algorithm].append(objective)
                        except KeyError:
                            continue

    averages = {
        algorithm: sum(values) / len(values)
        for algorithm, values in results.items()
    }

    with open(output_file, "w", newline="") as csvfile:
        columns = ["Instance"] + list(averages.keys())
        writer = csv.DictWriter(csvfile, fieldnames=columns)
        writer.writeheader()
        row = {"Instance": instance_name}
        row.update(averages)
        writer.writerow(row)

    print(f"CSV file saved to: {output_file}")


def main(best_solution=False, csv=False, filtered_by_instance=False, filtered=None):
    base_folder = '../problems/solutions/euclidean/solutions_vrppd'
    output_file = '../problems/solutions/euclidean/solutions_vrppd/averages_CVRP_3.csv'
    instance_name = "CVRP_3.txt"

    if filtered_by_instance:
        compute_average_objectives(instance_name, base_folder, output_file)
    else:
        results = process_solutions_folder(base_folder)
        write_solutions(output_file, results, best_solution, csv, filtered)


if __name__ == "__main__":
    main(
        best_solution=True,
        csv=False,
        filtered_by_instance=True,
        filtered=None
        # [
        #   "FIRST_UNBOUND_MIN_VALUE_and_SIMULATED_ANNEALING",
        #  "SAVINGS_and_GREEDY_DESCENT",
        # "ALL_UNPERFORMED_and_GENERIC_TABU_SEARCH",
        # "PARALLEL_CHEAPEST_INSERTION_and_GREEDY_DESCENT",
        # "FIRST_UNBOUND_MIN_VALUE_and_GENERIC_TABU_SEARCH"
        # ]

    )
