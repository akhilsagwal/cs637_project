import pandas as pd

# Load log files
baseline_log = pd.read_csv('chandleraz_baseline_simulation_log.csv')
smart_log = pd.read_csv('chandleraz_smart_simulation_log.csv')
baware_log = pd.read_csv('chandleraz_baware_simulation_log.csv')

# Function to calculate metrics
def calculate_avg_signal_strength(log_df):
    return log_df['Signal_Strength'].mean()

# Calculate average signal strength for each algorithm
baseline_avg_signal_strength = calculate_avg_signal_strength(baseline_log)
smart_avg_signal_strength = calculate_avg_signal_strength(smart_log)
baware_avg_signal_strength = calculate_avg_signal_strength(baware_log)

# Compile results into a summary table
summary_table = pd.DataFrame({
    'Algorithm': ['Baseline', 'Smart', 'B-AWARE'],
    'Avg Signal Strength': [
        baseline_avg_signal_strength,
        smart_avg_signal_strength,
        baware_avg_signal_strength
    ]
})

# Display the results
print("Comparison Summary Table")
print(summary_table)
