import re

# Lire le fichier brut contenant les poids et biais
with open("poids_bias.txt", "r") as f:
    text = f.read()

# Extraire les valeurs avec regex
weights = [float(w) for w in re.findall(r"W:\s*(-?[\d.]+(?:e[+-]?\d+)?)", text)]
biases = [float(b) for b in re.findall(r"bias:\s*(-?[\d.]+(?:e[+-]?\d+)?)", text)]

# Afficher les tailles pour vérification
print(f"Nombre de poids : {len(weights)}")
print(f"Nombre de biais : {len(biases)}")

# Formater en texte avec 7 décimales
weights_str = "poids : " + ", ".join(f"{w:.7f}" for w in weights)
biases_str = "bias : " + ", ".join(f"{b:.7f}" for b in biases)

# Sauvegarder dans un fichier propre
with open("weights_biases_formatted.txt", "w") as out:
    out.write(weights_str + "\n")
    out.write(biases_str + "\n")
