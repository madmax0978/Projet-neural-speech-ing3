import numpy as np
import matplotlib.pyplot as plt

# Charger les signaux
before = np.loadtxt("pre_before.txt")
after = np.loadtxt("pre_after.txt")

plt.figure(figsize=(10, 4))
plt.plot(before, label="Avant Pre-emphasis", alpha=0.7)
plt.plot(after, label="Après Pre-emphasis", alpha=0.7)
plt.legend()
plt.title("Effet du filtre Pre-emphasis")
plt.xlabel("Échantillons")
plt.ylabel("Amplitude")
plt.grid(True)
plt.tight_layout()
plt.show()
