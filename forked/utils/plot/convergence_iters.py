import json
import matplotlib.pyplot as plt

# 1. Load data from JSON
with open("convergence_iters.json", "r") as f:
    data = json.load(f)   # Example: {"MethodA": 120, "MethodB": null, "MethodC": 200}

# 2. Filter invalid or missing values
valid_data = {}
for method, cnt in data.items():
    if cnt is None:
        print(f"Warning: '{method}' has no iteration count, skipping")
        continue
    try:
        cnt = int(cnt)
    except Exception:
        print(f"Warning: '{method}' count '{cnt}' is not an integer, skipping")
        continue
    valid_data[method] = cnt

# 3. Prepare data for plotting
methods    = list(valid_data.keys())
iterations = list(valid_data.values())

if not methods:
    raise RuntimeError("No valid data to plot!")

# 4. Plot bar chart
plt.figure(figsize=(8, 5))
plt.bar(methods, iterations)
plt.xlabel("Algorithm")
plt.ylabel("Convergence Iterations")
plt.title("Comparison of Convergence Iterations for Different Algorithms")
plt.tight_layout()
plt.show()
