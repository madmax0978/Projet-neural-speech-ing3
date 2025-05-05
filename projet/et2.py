from scipy.signal import firwin
import numpy as np

fs = 32000  # Fréquence d'échantillonnage
cutoff = 4000  # Fréquence de coupure
numtaps = 21  # Nombre de coefficients (ordre + 1)

# Génère les coefficients
coeffs = firwin(numtaps, cutoff, fs=fs)

# Affiche les coefficients formatés pour Arduino
print(", ".join([f"{c:.6f}" for c in coeffs]))
