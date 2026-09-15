import numpy as np

k_max = 100
eps_max = 0.80
eps_min = 0.15
alpha = 2.0

print('Lyapunov Threshold Schedule:')
print('Iteration | Decay Factor | Relative Threshold | Required V Decrease')
print('-' * 70)

for k in [0, 1, 5, 10, 20, 30, 40, 50]:
    decay = np.exp(-alpha * k / k_max)
    total = eps_max * decay + eps_min
    print(f'{k:9} | {decay:12.4f} | {total:18.4f} | {total*100:18.1f}%')
