#!/bin/bash

# Chemin vers ton interpréteur Python dans le venv
SHEBANG="#!/home/gabin/venvs/pyside-env/bin/python"

# Parcours tous les fichiers .py du dossier
for file in *.py; do
  # Ajoute le shebang en première ligne s’il n’existe pas déjà
  grep -q "$SHEBANG" "$file" || sed -i "1i $SHEBANG" "$file"
  # Rend le fichier exécutable
  chmod +x "$file"
  echo "Préparé : $file"
done
