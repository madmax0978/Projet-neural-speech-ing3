import struct

# Ouvre le fichier texte contenant les valeurs envoyées par l'Arduino
with open('chuchoter.txt', 'r') as f:
    # Ignore les lignes vides ou les lignes avec des espaces
    values = [int(x.strip()) for x in f.readlines() if x.strip().isdigit()]

# Ouvre un fichier binaire en écriture
with open('chuchoter.bin', 'wb') as f:
    for value in values:
        # Convertit chaque valeur en entier 16 bits (little endian)
        binary = struct.pack('<H', value)  # 16 bits non signés
        f.write(binary)
