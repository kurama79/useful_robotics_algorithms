'''
Tarea 02 - Poligono de visibilidad
	En este programa obtenemos el poligono de visibilidad de acuerdo a la posicion en la que se
	encuentre el "robot".

Por Luis Enrique Ruiz Fernandez
'''

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
from shapely.geometry import Point, Polygon, LineString

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

	obs_vertex['1'] = [ ( 1.8,  0.0),
						(10.0,  0.0),
						(10.0, 10.0),
						( 9.3, 10.0),
						( 9.3,  8.7),
						( 8.2,  8.7),
						( 8.2,  6.5),
						( 9.3,  6.5),
						( 9.3,  1.5),
						( 7.0,  1.5),
						( 7.0,  4.0),
						( 6.2,  4.0),
						( 6.2,  1.5),
						( 1.8,  1.5),
						( 1.8,  0.0) ]

	codes['1'] = [ Path.MOVETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.CLOSEPOLY ]

	obs_vertex['2'] = [ ( 0.0,  0.0),
						( 0.7,  0.0),
						( 0.7,  4.0),
						( 1.8,  4.0),
						( 1.8,  4.9),
						( 0.7,  4.9),
						( 0.7,  7.5),
						( 1.8,  7.5),
						( 1.8,  8.0),
						( 3.7,  8.0),
						( 3.7,  8.1),
						( 4.9,  8.1),
						( 4.9,  6.0),
						( 5.5,  6.0),
						( 5.5,  8.1),
						( 7.5,  8.1),
						( 7.5, 10.0),
						( 0.0, 10.0),
						( 0.0,  0.0) ]

	codes['2'] = [ Path.MOVETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.CLOSEPOLY ]

	obs_vertex['3'] = [ ( 6.2,  7.1),
						( 7.4,  7.1),
						( 7.4,  5.4),
						( 6.2,  5.4),
						( 6.2,  7.1) ]

	codes['3'] = [ Path.MOVETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.CLOSEPOLY ]

	obs_vertex['4'] = [ ( 2.9,  4.4),
						( 4.9,  4.4),
						( 4.9,  2.8),
						( 2.9,  2.8),
						( 2.9,  4.4) ]

	codes['4'] = [ Path.MOVETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.CLOSEPOLY ]

	obs_vertex['5'] = [ ( 2.2,  6.9),
						( 3.7,  6.9),
						( 3.7,  5.7),
						( 2.2,  5.7),
						( 2.2,  6.9) ]

	codes['5'] = [ Path.MOVETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.CLOSEPOLY ]

	return limits, obs_vertex, codes

# Funcion para verificar si el punto esta fuera de los obstaculos
def inObstacles(point, obs):
	within_obs = False
	p = Point(point)

	for i in obs:
		poly = Polygon(obs[i])

		aux = p.within(poly)
		if aux:
			within_obs = aux
			break

	return within_obs

# Funcion principal para determinar el poligono
def visibileVertex(pos_vis, obs_vertex):
	visible_vertex = []
	not_visible_vertex = []

	# Obtenemos los vertices
	vertex = []
	for i in obs_vertex:
		vertex = obs_vertex.get(i)
		lines = []
		way_points = {}
		aux_obs = obs_vertex.copy()

		for j in range(len(vertex)):
			lines.append(LineString([pos_vis[0], vertex[j]]))
			way_points[j] = createWayPoints(lines[-1])
		
		poly = Polygon(vertex)

		for j in way_points: # El numero de lineas desde el origen respecto punto seleccionado
			intersected_points = np.transpose(way_points.get(j))
			points = {}

			for l in aux_obs: # Para verificcar interseccion con los obstaculos
				obst_vertex = aux_obs.get(l)
				poly_obs = Polygon(obst_vertex)

				for k in range(len(intersected_points)-1): # Verificar cada punto de la linea
					points = Point(intersected_points[k]) # Generando los puntos

					if points.intersects(poly_obs): # Verificamos si el punto actual intersecta con algo
						not_visible_vertex.append(intersected_points[-1]) # Agregamos el ultimo (-1)
						break

	for i in obs_vertex:
		aux_vertex = []
		aux_copy = obs_vertex.get(i)
		vertex = aux_copy[:][:]
		for k in vertex:
			for j in not_visible_vertex:
				aux_not_vis = j

				if k[0] == aux_not_vis[0] and k[1] == aux_not_vis[-1]:
					aux_vertex.append(k)
					break

		vertex.pop(-1)
		for j in aux_vertex:
			for indx, k in enumerate(vertex):
				if k[0] == j[0] and k[1] == j[1]:
					vertex.pop(indx)

		visible_vertex += vertex
	return visible_vertex

# Creando los way_points
def createWayPoints(line):
	aux_x = np.linspace(line.coords.xy[0][0], line.coords.xy[0][1], num=200, endpoint=True)
	aux_y = np.linspace(line.coords.xy[1][0], line.coords.xy[1][1], num=200, endpoint=True)
	way_points = np.array([aux_x, aux_y])

	return way_points 

# Funcion para ordenar los vertices
def orderVertex(vertex, pos): # Resolver problema de acomodo de vertices (no grave)
	centroid = np.array([pos[0][0], pos[0][1]])
	# centroid = np.mean(vertex, axis=0)
	vertex = np.array(vertex)
	d = vertex - centroid
	angles = np.arctan2(d[:,0], d[:,1])

	oder = vertex[np.argsort(angles)]

	for i in range(1, len(oder)-1):
		if np.arctan((centroid[1]-oder[i+1][1])/(centroid[0]-oder[i+1][0])) == np.arctan((centroid[1]-oder[i][1])/(centroid[0]-oder[i][0])):
			angle = np.arctan2(oder[i-1][1]-oder[i][1], oder[i-1][0]-oder[i][0])
			if angle == np.pi/2 or angle == -np.pi/2 or angle == np.pi or angle == -np.pi or angle == 0:
				continue
			else:
				temp = [oder[i+1][0], oder[i+1][1]]
				oder[i+1], oder[i] = oder[i], temp

	return oder

# Funcion para generar los nuevos vertices del poligono de visibilidad
def genNewVertex(vis_vertex, pos, limits, obs_vertex):
	slopes = []
	area = Polygon(limits)
	new_vertex = []

	for i in vis_vertex:
		x = i[0] - pos[0][0]
		y = i[1] - pos[0][1]
		angle = np.arctan2(y, x)
		slopes.append(angle)

	lines = {}
	for j in range(len(vis_vertex)):
		line = LineString([vis_vertex[j], (vis_vertex[j][0] + 15*np.cos(slopes[j]), vis_vertex[j][1] + 15*np.sin(slopes[j]))])
		lines[j] = createWayPoints(line)

	line_del = []
	for j in lines:
		obs_intersected = []
		intersected_points = np.transpose(lines.get(j))

		for l in obs_vertex:
			vertex = obs_vertex.get(l)
			poly = Polygon(vertex)

			for k in intersected_points[1:]:
				ref_point = Point(intersected_points[1])
				point = Point(k)

				obs_touched = []
				for i in obs_vertex:
					obs = obs_vertex.get(i)
					obs_int = Polygon(obs)

					if ref_point.intersects(obs_int) or ref_point.within(obs_int):
						obs_touched.append(True)
					else:
						obs_touched.append(False)

				if any(line_del) == j:
					break
				elif any(obs_touched) or (not point.within(area)):
					line_del.append(j)
					break
				elif point.intersects(poly) or point.within(poly):
					obs_intersected.append(k)
					break

		distances = []
		for i in obs_intersected:
			distances.append(distanceFunction([ref_point.x, ref_point.y], i))
		if distances:
			idx = distances.index(min(distances))
			new_vertex.append(obs_intersected[idx])

	vis_polygon = vis_vertex + new_vertex
	vis_polygon = orderVertex(vis_polygon, pos)
	return vis_polygon

# Funcion para obtener distancia
def distanceFunction(p1, p2):
	x = (p2[0] - p1[0])**2
	y = (p2[1] - p1[0])**2
	d = x + y

	return np.sqrt(d)

# Inicio del programa...............................................................................
if __name__=='__main__':
	# Definimos el entorno
	environment_limits, obs_vertex, codes = environmentGen()

	# Iniciamos la figura
	fig, ax = plt.subplots()
	ax.set_title('Poligono de Visibilidad')

	count = 0
	while True:
		ax.cla()

		# Iniciando el entorno
		ax.set_xlim(environment_limits[3])
		ax.set_ylim(environment_limits[3])

		map_path = Path(environment_limits, codes.get('map'))
		map_patch = patches.PathPatch(map_path, facecolor='white', lw=2)
		ax.add_patch(map_patch)

		# Iniciando los obstaculos
		obs_path = {}
		obs_patch = {}
		for i in obs_vertex:
			obs_path[i] = Path(obs_vertex.get(i), codes.get(i))
			obs_patch[i] = patches.PathPatch(obs_path.get(i), facecolor='black', lw=2)
			ax.add_patch(obs_patch.get(i))

		# Seleccion del punto para generar el poligono de visibilidad
		pos_vis = plt.ginput(n=1, timeout=0, show_clicks=True)
		ax.plot(pos_vis[0][0], pos_vis[0][1], 'o', color='red')

		if not inObstacles(pos_vis, obs_vertex):
			vis_polygon = visibileVertex(pos_vis, obs_vertex)
			vis_polygon = genNewVertex(vis_polygon, pos_vis, environment_limits, obs_vertex)
		else:
			break

		ax.plot(pos_vis[0][0], pos_vis[0][1], 'o', color='red')

		for i in range(len(vis_polygon)):
			ax.plot(vis_polygon[i][0], vis_polygon[i][1], 'o', color='blue')

		visible_polygon = patches.Polygon(vis_polygon, closed=True, color='yellow')
		ax.add_patch(visible_polygon)

		plt.pause(0.1)

		# Condicion de paro......................................................................
		if count >= 10:
			ans = input('¿Quiere salir? [S/N]: ')
			if ans == 'S' or ans == 's':
				break
			count = 0
		count += 1