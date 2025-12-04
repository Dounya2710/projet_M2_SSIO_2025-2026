import xml.etree.ElementTree as ET
import math

# Taille de chaque cellule dans la matrice (1m x 1m)
cell_size = 1

# Dimensions de la salle (8m x 8m)
room_size = 8

# Création de la matrice vide (initialement tout est vide)
grid_size = room_size  # 8x8
matrix = [[0 for _ in range(grid_size)] for _ in range(grid_size)]  # 8x8 grid de 0 (vide)

# Charger le fichier SDF
tree = ET.parse('env.sdf')
root = tree.getroot()

# Fonction pour ajouter un obstacle à la matrice
def add_obstacle_to_matrix(x, y, sx, sy, type='box'):
    # Calcul des limites de la box ou cylindre
    x_min = x - sx / 2
    x_max = x + sx / 2
    y_min = y - sy / 2
    y_max = y + sy / 2

    # Vérifier chaque case de la matrice
    for i in range(grid_size):
        for j in range(grid_size):
            # Calcul des coordonnées de chaque cellule de la matrice
            cell_x_min = j * cell_size - room_size / 2  # -4 à 4 pour centrer la matrice
            cell_x_max = (j + 1) * cell_size - room_size / 2
            cell_y_min = i * cell_size - room_size / 2
            cell_y_max = (i + 1) * cell_size - room_size / 2

            # Vérifier si la cellule est dans l'obstacle (intersecte)
            if (x_min < cell_x_max and x_max > cell_x_min and
                y_min < cell_y_max and y_max > cell_y_min):
                matrix[i][j] = 1  # Marquer cette cellule comme occupée par un obstacle

# Récupérer tous les modèles statiques (obstacles)
obstacles = []

for model in root.findall(".//model"):
    static = model.find('static')
    if static is not None and static.text == 'true':
        link = model.find('link')
        if link is not None:
            collision = link.find('collision')
            if collision is not None:
                geometry = collision.find('geometry')
                pose = model.find('pose')
                x, y, z = 0, 0, 0
                if pose is not None:
                    x, y, z, _, _, _ = map(float, pose.text.split())
                
                if geometry.find('box') is not None:
                    size = geometry.find('box/size')
                    sx, sy, sz = map(float, size.text.split())
                    add_obstacle_to_matrix(x, y, sx, sy, 'box')
                
                elif geometry.find('cylinder') is not None:
                    radius = float(geometry.find('cylinder/radius').text)
                    length = float(geometry.find('cylinder/length').text)
                    # Traiter les cylindres comme des boîtes avec un rayon et une longueur (approximé)
                    add_obstacle_to_matrix(x, y, 2 * radius, length, 'cylinder')

# Affichage de la matrice
for row in matrix:
    print(" ".join(str(cell) for cell in row))
