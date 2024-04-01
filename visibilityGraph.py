'''
Tarea 03 - Grafo de visibilidad
	En este programa obtenemos el grafo de visibilidad de acuerdo a la posicion en la que se
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

	obs_vertex['1'] = [ ( 6.2,  7.1),
						( 7.4,  7.1),
						( 7.4,  5.4),
						( 6.2,  5.4),
						( 6.2,  7.1) ]

	codes['1'] = [ Path.MOVETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.LINETO,
				   Path.CLOSEPOLY ]

	obs_vertex['2'] = [ ( 2.9,  4.4),
						( 4.9,  4.4),
						( 4.9,  2.8),
						( 2.9,  2.8),
						( 2.9,  4.4) ]

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

				for k in range(len(intersected_points)-2): # Verificar cada punto de la linea
					points = Point(intersected_points[k+1]) # Generando los puntos

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
	aux_x = np.linspace(line.coords.xy[0][0], line.coords.xy[0][1], num=100, endpoint=True)
	aux_y = np.linspace(line.coords.xy[1][0], line.coords.xy[1][1], num=100, endpoint=True)
	way_points = np.array([aux_x, aux_y])

	return way_points 	

# Inicio del programa...............................................................................
if __name__=='__main__':
	# Definimos el entorno
	environment_limits, obs_vertex, codes = environmentGen()

	# Iniciamos la figura
	fig, ax = plt.subplots()
	ax.set_title('Poligono de Visibilidad')

	visible_vertex = {}
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

		# Seleccion del puntos de inicio y final
		initial_pos = plt.ginput(n=1, timeout=0, show_clicks=True)
		ax.plot(initial_pos[0][0], initial_pos[0][1], 'o', color='green')

		goal_pos = plt.ginput(n=1, timeout=0, show_clicks=True)
		ax.plot(goal_pos[0][0], goal_pos[0][1], 'o', color='red')

		if not inObstacles(initial_pos, obs_vertex) and not inObstacles(goal_pos, obs_vertex):
			pos = {}
			visible_vertex['0'] = [visibileVertex(initial_pos, obs_vertex)]
			pos['0'] = initial_pos
			vertex2plot = visible_vertex.get('0')
			for i in range(len(vertex2plot)):
				ax.plot(vertex2plot[i][0], vertex2plot[i][1], 'o', color='blue')

			count = 1
			for i in obs_vertex:
				actual_obs = obs_vertex.get(i)
				aux_visible = []
				pos_aux = []

				for j in actual_obs:
					pos_aux += [(j[0], j[1])]
					aux_visible.append(visibileVertex([(j[0], j[1])], obs_vertex))

				visible_vertex[i] = aux_visible
				pos[i] = pos_aux

				count += 1

			visible_vertex[str(count)] = [visibileVertex(goal_pos, obs_vertex)]
			pos[str(count)] = goal_pos
		else:
			break

		ax.plot(initial_pos[0][0], initial_pos[0][1], 'o', color='green')
		ax.plot(goal_pos[0][0], goal_pos[0][1], 'o', color='red')

		for i in visible_vertex:
			actual_vertex = visible_vertex.get(i)
			actual_pos = pos.get(i)

			for j in range(len(actual_vertex)):
				for k in actual_vertex[j]:
					ax.plot(k[0], k[1], 'o', color='blue')

			for j in range(len(actual_pos)):
				if len(actual_pos) == 1 or j < len(actual_pos):
					for k in actual_vertex[j]:
						x = [actual_pos[j][0], k[0]]
						y = [actual_pos[j][1], k[1]]

						ax.plot(x, y, 'y-')
				
		plt.pause(0.1)

		# Condicion de paro......................................................................
		if count > 10:
			ans = input('¿Quiere salir? [S/N]: ')
			if ans == 'S' or ans == 's':
				break
			count = 0
		count += 1