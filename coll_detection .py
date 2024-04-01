'''
Tarea 05 - Collision Detection: Hierarachical Algorithm
	En este programa buscamos implementar el algoritmo recursivo de detecccion de colisiones, generando un arbol
	en el cual se ven las particiones en triangulo del objeto con sus respectivos circulo que encierra la figura

	Hay comentarios dentro del codigo para cambiar de ejemplos entre uno que si hay colision y otro no, aparte 
	como es parte de la tarea, nos da la distancia aproximada a la que se encuentran los obastaculos.

	El algoritmo se probo con diferentes obstaculo, en el caso de este script solo se dejaron dos por cuestion
	de compactacion.

Por Luis Enrique Ruiz Fernandez
'''

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
from shapely.geometry import *
from shapely.ops import *

# Funcion para generar las dimensiones del entorno y obstaculos
def environmentGen():
	limits = [ ( 0,  0), # Definimos los limites del entorno
			   (10,  0),
			   (10, 10),
			   ( 0, 10),
			   ( 0,  0) ] 

	codes = {'map' : [Path.MOVETO,
			          Path.LINETO,
			     	  Path.LINETO,
			     	  Path.LINETO,
			     	  Path.CLOSEPOLY] }

	obs_vertex = {}

	obs_vertex['1'] = [ ( 6.2,  7.1), # Ejemplo SIN colision
						( 7.4,  7.1),
						( 7.4,  5.3),
						( 6.2,  5.3),
						( 6.2,  7.1) ]

	# obs_vertex['1'] = [ ( 6.2,  7.1), # Ejemnplo CON colision
	# 					( 7.4,  7.1),
	# 					( 7.4,  5.4),
	# 					( 6.2,  5.4),
	# 					( 6.2,  7.1) ]

	codes['1'] = [ Path.MOVETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.CLOSEPOLY ]

	obs_vertex['2'] = [ ( 2.9,  5.0), # Ejemplo SIN colision
						( 6.9,  5.0),
						( 6.9,  2.8),
						( 2.9,  2.8),
						( 2.9,  5.0) ]

	# obs_vertex['2'] = [ ( 2.9,  5.6), # Ejemplo CON colision
	# 					( 6.9,  5.6),
	# 					( 6.9,  2.8),
	# 					( 2.9,  2.8),
	# 					( 2.9,  5.6) ]

	codes['2'] = [ Path.MOVETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.CLOSEPOLY ]

	obs_vertex['3'] = [ ( 2.2,  6.9),
						( 3.7,  6.9),
						( 3.7,  5.7),
						( 2.2,  5.7),
						( 2.2,  6.9) ]

	codes['3'] = [ Path.MOVETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.CLOSEPOLY ]

	return limits, obs_vertex, codes

# Funcion para obtener el arbol
def Tree(poly, min_radius=0.1):

	tree = {}

	# Funciones para la construccion
	def root(circle, poly):

		children = ['1', '2']
		tree['0'] = [circle, poly, None, children, False]

		tree[children[0]] = None
		tree[children[1]] = None

	def set_right(circle, poly, parent):

		children = [str(((int(parent)*2) + 2)*2+1), str(((int(parent)*2) + 2)*2+2)]

		tree[str(parent)][4] = False
		tree[str((int(parent)*2) + 2)] = [circle, poly, parent, children, True]

	def set_left(circle, poly, parent):

		children = [str(((int(parent)*2) + 1)*2+1), str(((int(parent)*2) + 1)*2+2)]

		tree[str(parent)][4] = False
		tree[str((int(parent)*2) + 1)] = [circle, poly, parent, children, True]

	def get_circle(poly):

		centroid = poly[0].centroid

		far_vertex = centroid
		for i in range(len(list(poly[1].coords))):

			d1 = centroid.distance(Point(poly[1].coords[i]))
			d2 = centroid.distance(far_vertex)

			if d1 > d2:
				far_vertex = Point(poly[1].coords[i])
				radius = d1

		circle = Point(centroid.coords[:][0]).buffer(radius)

		return circle

	def div_polygon(poly, count):

		def cut_polygon_by_line(polygon, line):

			merged = linemerge([polygon.boundary, line])
			borders = unary_union(merged)
			polygons = polygonize(borders)

			return list(polygons)

		root = True if count == 0 else False
		coords = poly[1].coords[:]

		if root:
			xy_max = [0, 0]
			xy_min = [1000, 1000]
			for i in range(len(coords)):

				if coords[i][0] > xy_max[0]:
					xy_max[0] = coords[i][0]

				if coords[i][1] > xy_max[1]:
					xy_max[1] = coords[i][1]

				if coords[i][0] < xy_min[0]:
					xy_min[0] = coords[i][0]

				if coords[i][1] < xy_min[1]:
					xy_min[1] = coords[i][1]

			div_line = LineString([tuple(xy_min), tuple(xy_max)])
			aux_1, aux_2 = cut_polygon_by_line(poly[0], div_line)

		else:

			d1 = Point(coords[0]).distance(Point(coords[1]))
			d2 = Point(coords[1]).distance(Point(coords[2]))
			d3 = Point(coords[2]).distance(Point(coords[0]))

			if d1 >= d2 and d1 >= d3:

				vertex_1 = coords[2]
				line = LineString((coords[0], coords[1]))
				center = line.centroid
				vertex_2 = center.coords[0]
				vertex_left = coords[0]
				vertex_right = coords[1]

			elif d2 >= d1 and d2 >= d3:

				vertex_1 = coords[0]
				line = LineString((coords[1], coords[2]))
				center = line.centroid
				vertex_2 = center.coords[0]
				vertex_left = coords[1]
				vertex_right = coords[2]

			else:

				vertex_1 = coords[1]
				line = LineString((coords[2], coords[0]))
				center = line.centroid
				vertex_2 = center.coords[0]
				vertex_left = coords[2]
				vertex_right = coords[0]

			aux_1 = Polygon([vertex_1, vertex_2, vertex_left])
			aux_2 = Polygon([vertex_1, vertex_2, vertex_right])

		poly_1 = [aux_1, LinearRing(tuple(list(aux_1.exterior.coords)))]
		poly_2 = [aux_2, LinearRing(tuple(list(aux_2.exterior.coords)))]

		return poly_1, poly_2


	root_circle = get_circle(poly)
	root(root_circle, poly)

	count = 0
	while count < 16:

		poly_1, poly_2 = div_polygon(tree[str(count)][1], count)
		circle_1 = get_circle(poly_1)
		circle_2 = get_circle(poly_2)

		set_left(circle_1, poly_1, count)
		set_right(circle_2, poly_2, count)

		del poly_1, poly_2

		count += 1

	return tree

# Funcion para detectar colision ------------------------Principal de la Tarea------------------------------
def collision_detection(T1, T2):

	def collide(r1, r2, T1, T2):

		if not r1[0].intersects(r2[0]):

			min_dist = 1000
			for i in list(r1[0].exterior.coords):
				for j in list(r2[0].exterior.coords):
					
					if Point(i).distance(Point(j)) < min_dist:
						min_dist = Point(i).distance(Point(j))

			return False, min_dist

		if r1[4] or r2[4]:

			if r1[1][0].intersects(r2[1][0]) or r1[1][0].touches(r2[1][0]):
				return True, 0

			else:
				min_dist = 1000
				for i in list(r1[0].exterior.coords):
					for j in list(r2[0].exterior.coords):
						
						if Point(i).distance(Point(j)) < min_dist:
							min_dist = Point(i).distance(Point(j))

				return False, min_dist


		if r1[0].area > r2[0].area:

			aux_T = T1
			T1 = T2
			T2 = aux_T

			aux = r1
			r1 = r2
			r2 = aux

		coll, dist = collide(r1, T2[r2[3][0]], T1, T2)
		if coll:
			return True, 0
		else:
			coll, dist = collide(r1, T2[r2[3][1]], T1, T2)

			if coll:
				return True, 0
			else:

				min_dist = 1000
				for i in list(r1[0].exterior.coords):
					for j in list(r2[0].exterior.coords):
						
						if Point(i).distance(Point(j)) < min_dist:
							min_dist = Point(i).distance(Point(j))

				return False, min_dist

	coll, dist = collide(T1['0'], T2['1'], T1, T2)

	return coll, dist



# Inicio del programa...............................................................................
if __name__=='__main__':
	
	# Definimos el entorno
	environment_limits, obs_vertex, codes = environmentGen()
	tri_codes = [ Path.MOVETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.CLOSEPOLY ]

	cuad_codes = [ Path.MOVETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.CLOSEPOLY ]

	# Iniciamos la figura
	fig, ax = plt.subplots()
	ax.set_title('Poligono de Visibilidad')

	# Iniciando el entorno
	ax.set_xlim(environment_limits[3])
	ax.set_ylim(environment_limits[3])

	map_path = Path(environment_limits, codes.get('map'))
	map_patch = patches.PathPatch(map_path, facecolor='white', lw=2)
	ax.add_patch(map_patch)

	# Iniciando los obstaculos
	obs_path = {}
	obs_patch = {}
	T = {}
	for i in obs_vertex:

		poly = [Polygon(obs_vertex.get(i)), LinearRing(obs_vertex.get(i))]
		obs_path[i] = Path(obs_vertex.get(i), codes.get(i))
		obs_patch[i] = patches.PathPatch(obs_path.get(i), facecolor='black', lw=2)
		ax.add_patch(obs_patch.get(i))

		T[i] = Tree(poly)

	triangles_path = [{}, {}, {}]
	triangles_patch = [{}, {}, {}]
	circle_X = []
	circle_Y = []
	for i in T:
		for j in T[i]:

			if T[i][j]:

				circle_x, circle_y = T[i][j][0].exterior.coords.xy
				circle_X.append(circle_x[:])
				circle_Y.append(circle_y[:])

				if len(list(T[i][j][1][0].exterior.coords)) == 4:

					triangles_path[int(i)-1][j] = Path(list(T[i][j][1][0].exterior.coords), tri_codes)

				else:

					triangles_path[int(i)-1][j] = Path(list(T[i][j][1][0].exterior.coords), cuad_codes)

				triangles_patch[int(i)-1][j] = patches.PathPatch(triangles_path[int(i)-1].get(j), facecolor='red', lw=2)
				ax.add_patch(triangles_patch[int(i)-1].get(j))

	collision, distance = collision_detection(T['1'], T['2'])

	print('Existe colision? : ', collision, '\n La distancia aproximada entre objetos: ', distance)

	for i in range(len(circle_X)):
		plt.plot(circle_X[i], circle_Y[i])

	plt.show()