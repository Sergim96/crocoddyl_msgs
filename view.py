import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("/tmp/solver_trajectory_debug.csv")

# Filter events
ref_df = df[df['event'] == 'get_current_reference'].reset_index(drop=True)
proc_df = df[df['event'] == 'process_queue']

# Find regressions: where t0 goes backward
regressions = []
for i in range(1, len(ref_df)):
    if ref_df.loc[i, 't0'] < ref_df.loc[i - 1, 't0']:
        regressions.append(i)

for i in regressions:
    prev = ref_df.loc[i - 1]
    curr = ref_df.loc[i]
    print(f"⚠️ t0 regression at t_rel={curr['t_rel']:.3f}s: t0 {curr['t0']:.6f} < prev t0 {prev['t0']:.6f}")


# Plotting
plt.figure(figsize=(12, 6))
plt.plot(ref_df['t_rel'], ref_df['t0'], label='t0 served', marker='o')
plt.plot(ref_df['t_rel'], ref_df['t_now'], label='t_now', linestyle='--')
plt.plot(ref_df['t_rel'], ref_df['queue_end'], label='queue_end', linestyle=':')

# Highlight regressions in red
if regressions:
    plt.plot(
        ref_df.loc[regressions, 't_rel'],
        ref_df.loc[regressions, 't0'],
        'ro',
        label='⚠️ t0 regression'
    )

plt.xlabel("Relative time (s)")
plt.ylabel("Timestamps (s)")
plt.title("Reference timing vs queue")
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

