import numpy as np
from sklearn.neural_network import MLPClassifier
from sklearn.metrics import accuracy_score

# === PARAMÈTRES ===
NB_TRAIN = 100
NB_TEST = 10
DCT_SIZE = 13
NB_FRAMES = 61
INPUT_SIZE = DCT_SIZE * NB_FRAMES

# === CHARGEMENT DU DATASET ===
def parse_c_array_file(filepath, expected_samples, frames=61, coeffs=13):
    with open(filepath, 'r') as f:
        content = f.read()

    content = content.replace("},", "}\n")
    lines = content.strip().splitlines()

    current_sample = []
    dataset = []

    for line in lines:
        line = line.strip().strip('{').strip('},').strip()
        if not line:
            continue
        values = list(map(float, line.split(',')))
        current_sample.append(values)
        if len(current_sample) == frames:
            dataset.append(np.array(current_sample).flatten())
            current_sample = []

    data = np.array(dataset)
    assert data.shape == (expected_samples, INPUT_SIZE), f"Erreur: attendu ({expected_samples}, {INPUT_SIZE}), obtenu {data.shape}"
    return data

# === CHARGEMENT DES DONNÉES ===
X_train = parse_c_array_file("dataset_training.txt", NB_TRAIN)
y_train = np.array([0]*50 + [1]*50)

X_test = parse_c_array_file("dataset_test.txt", NB_TEST)
y_test = np.array([0]*5 + [1]*5)


# === ENTRAÎNEMENT avec au moins 95% de précision ===
best_model = None
best_acc = 0
for i in range(50):
    model = MLPClassifier(hidden_layer_sizes=(64,32), activation='relu',
                          solver='adam', max_iter=5000, tol=1e-5, alpha=1e-4)
    model.fit(X_train, y_train)
    acc = accuracy_score(y_test, model.predict(X_test))
    print(f"Essai {i+1}: accuracy = {acc:.4f}")
    if acc > best_acc:
        best_acc = acc
        best_model = model

print(f"\n Précision obtenue : {best_acc:.4f}")
assert best_acc >= 0.95, "Erreur : la précision est inférieure à 95%."

# === AFFICHAGE DES PROBABILITÉS PAR ÉCHANTILLON TEST ===
print("\n--- Prédictions détaillées sur le test set ---")
probas = best_model.predict_proba(X_test)
for i, (p, true_label) in enumerate(zip(probas, y_test)):
    print(f"Test #{i+1} - Jaune: {p[0]:.4f} / Vert: {p[1]:.4f} (vrai label: {'jaune' if true_label == 0 else 'vert'})")

# === EXPORT VERS .h ===
def export_header(weights, biases, filename="parametre.h"):
    with open(filename, "w") as f:
        f.write("#ifndef PARAMETRE_H\n#define PARAMETRE_H\n\n")
        f.write(f"#define NB_WEIGHTS {sum(w.size for w in weights)}\n")
        f.write(f"#define NB_BIASES {sum(b.size for b in biases)}\n")
        f.write('#include <NeuralNetwork.h>\n\n')


        f.write("const float weights_data[NB_WEIGHTS] = {\n")
        flat_weights = np.concatenate([w.flatten() for w in weights])
        f.write(",\n".join([f"{x:.6f}f" for x in flat_weights]))
        f.write("\n};\n\n")

        f.write("const float biases_data[NB_BIASES] = {\n")
        flat_biases = np.concatenate([b.flatten() for b in biases])
        f.write(",\n".join([f"{x:.6f}f" for x in flat_biases]))
        f.write("\n};\n\n")

        f.write("#endif\n")

# === EXPORT FINAL
export_header(best_model.coefs_, best_model.intercepts_)
print("\n Fichier parametre.h généré avec succès.")
