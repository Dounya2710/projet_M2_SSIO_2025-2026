import xml.etree.ElementTree as ET
import math

# -----------------------------
# Paramètres de la grille
# -----------------------------
cell_size = 1          # taille d'une cellule (1m x 1m)
room_size = 8          # salle 8m x 8m
grid_size = room_size  # grille 8x8

# Grille initialisée à 0 (vide)
matrix = [[0 for _ in range(grid_size)] for _ in range(grid_size)]

# -----------------------------
# Chargement du fichier SDF
# -----------------------------
tree = ET.parse('../env/env.sdf')
root = tree.getroot()

# -----------------------------
# Fonctions utilitaires
# -----------------------------

def add_obstacle_to_matrix(x, y, sx, sy, model_name=""):
    """
    Marque les cellules intersectant un obstacle centré en (x, y)
    de taille (sx, sy) en 1.
    """

    # Bornes de l'obstacle
    x_min = x - sx / 2
    x_max = x + sx / 2
    y_min = y - sy / 2
    y_max = y + sy / 2

    for i in range(grid_size):
        for j in range(grid_size):
            # Coordonnées de la cellule (on centre la salle en (0,0) : [-4, 4])
            cell_x_min = j * cell_size - room_size / 2
            cell_x_max = (j + 1) * cell_size - room_size / 2
            cell_y_min = i * cell_size - room_size / 2
            cell_y_max = (i + 1) * cell_size - room_size / 2

            # Test d'intersection entre la cellule et l'obstacle
            if (x_min < cell_x_max and x_max > cell_x_min and
                y_min < cell_y_max and y_max > cell_y_min):
                matrix[i][j] = 1

# -----------------------------
# Filtrage des "faux" obstacles
# -----------------------------

# Noms de modèles à ignorer (sol, plan du monde...)
IGNORE_MODEL_NAMES = {
    "ground_plane", "ground", "floor", "world", "plane", "room"
}

# Taille au-delà de laquelle on considère que c’est un "gros" objet structurel
BIG_SIZE_THRESHOLD = room_size * 0.9  # ex : > 7.2m sur X ou Y

def is_big_structure(sx, sy):
    """Retourne True si la box est tellement grande qu'on la considère comme sol/mur."""
    return sx >= BIG_SIZE_THRESHOLD or sy >= BIG_SIZE_THRESHOLD

# -----------------------------
# Lecture des modèles statiques (obstacles)
# -----------------------------

for model in root.findall(".//model"):
    name = model.get("name", "").lower()

    # 1) Ignorer explicitement certains modèles (sol, ground_plane, etc.)
    if any(ignore in name for ignore in IGNORE_MODEL_NAMES):
        continue

    static = model.find('static')
    if static is None or static.text.strip().lower() != 'true':
        continue  # on ne prend que les obstacles statiques

    pose = model.find('pose')
    if pose is not None:
        try:
            x, y, z, roll, pitch, yaw = map(float, pose.text.split())
        except ValueError:
            x = y = z = 0.0
    else:
        x = y = z = 0.0

    link = model.find('link')
    if link is None:
        continue

    collision = link.find('collision')
    if collision is None:
        continue

    geometry = collision.find('geometry')
    if geometry is None:
        continue

    # ---------- BOX ----------
    box = geometry.find('box')
    if box is not None:
        size = box.find('size')
        if size is None:
            continue
        sx, sy, sz = map(float, size.text.split())

        # 2) Ignorer les très grosses boxes (sol / murs)
        if is_big_structure(sx, sy):
            continue

        add_obstacle_to_matrix(x, y, sx, sy, model_name=name)
        continue

    # ---------- CYLINDER ----------
    cylinder = geometry.find('cylinder')
    if cylinder is not None:
        radius_elem = cylinder.find('radius')
        length_elem = cylinder.find('length')
        if radius_elem is None or length_elem is None:
            continue
        radius = float(radius_elem.text)
        length = float(length_elem.text)

        # On approxime le cylindre par une box (diamètre sur X/Y, longueur sur Z)
        sx = sy = 2 * radius

        if is_big_structure(sx, sy):
            continue

        add_obstacle_to_matrix(x, y, sx, sy, model_name=name)
        continue

# -----------------------------
# Affichage de la matrice
# -----------------------------
for row in matrix:
    print(" ".join(str(cell) for cell in row))
 
# -----------------------------   
# Résultat
# -----------------------------
# 0 0 0 0 0 0 0 0
# 0 1 0 1 1 1 1 0
# 0 0 0 0 0 0 1 1
# 0 1 1 1 1 1 1 1
# 0 0 0 0 0 1 1 0
# 0 0 1 1 1 1 1 0
# 0 1 0 0 0 0 1 0
# 0 0 0 0 0 0 0 0