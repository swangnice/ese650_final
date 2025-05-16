import json
import matplotlib.pyplot as plt

# Read JSON file
with open('training_times.json', 'r') as f:
    data = json.load(f)

# Extract model names and times
models = list(data.keys())
times = list(data.values())

# Plot bar chart
plt.figure()
plt.bar(models, times)      # Use default colors
plt.xlabel("Model")
plt.ylabel("Training Time (hours)")
plt.title("Comparison of Training Times for Different Models")
plt.xticks(rotation=45, ha='right')
plt.tight_layout()
plt.show()
