import os

def extract_data_from_txt(file_path):
    """
    Extrait les données après la ligne '== DATASET FORMAT ==' dans un fichier texte.
    Retourne une matrice 61x13 si le format est correct, sinon None.
    """
    with open(file_path, 'r', encoding='utf-8') as file:
        content = file.read()
        split_data = content.split("== DATASET FORMAT ==")
        if len(split_data) < 2:
            return None  # La balise n'est pas présente

        raw_data = split_data[1].strip()
        lines = raw_data.splitlines()
        data = []
        for line in lines:
            if line.strip():
                values = [float(val.strip()) for val in line.split(',') if val.strip()]
                data.append(values)

        # Vérifie si la taille est correcte
        if len(data) == 61 and all(len(row) == 13 for row in data):
            return data
        else:
            return None

def build_dataset(folder_path):
    """
    Parcourt tous les fichiers .txt du dossier donné et construit un dataset propre.
    """
    all_data = []
    for filename in sorted(os.listdir(folder_path)):
        if filename.endswith('.txt'):
            file_path = os.path.join(folder_path, filename)
            sample_data = extract_data_from_txt(file_path)
            if sample_data:
                all_data.append(sample_data)
            else:
                print(f"⚠️ Format invalide ou balise manquante dans : {filename}")
    return all_data

def format_dataset_to_c_array(data):
    """
    Formate les données en syntaxe C : const float inputs[x][61][13] = {...};
    """
    formatted = f"const float inputs[{len(data)}][61][13] = \n{{\n"
    for sample in data:
        formatted += "  {\n"
        for row in sample:
            formatted += "    {" + ", ".join(f"{val:.4f}" for val in row) + "},\n"
        formatted += "  },\n"
    formatted += "};\n"
    return formatted

def main():
    folder_path = "./audio_test"  # Dossier contenant les fichiers .txt
    output_file = "dataset_test.txt"  # Fichier de sortie

    if not os.path.exists(folder_path):
        print(f"Le dossier {folder_path} n'existe pas. Crée-le et place tes fichiers .txt dedans.")
        return

    dataset = build_dataset(folder_path)
    if not dataset:
        print("Aucun fichier valide trouvé.")
        return

    result = format_dataset_to_c_array(dataset)

    with open(output_file, "w", encoding="utf-8") as f:
        f.write(result)

    print(f"Fichier '{output_file}' généré avec succès avec {len(dataset)} échantillons.")

# Lancer le script
if __name__ == "__main__":
    main()
