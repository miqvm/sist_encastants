# AUTOR: MIQUEL VIVES MARCUS

import skimage.io as io
import skimage.filters as filters
import matplotlib.pyplot as pyplot
import matplotlib.patches as patches
import matplotlib.colors as colors
import skimage.color as color
import skimage.feature as feature
from skimage import img_as_ubyte
import skimage.exposure as exposure
import math
import numpy as np

# -----------------------
# GLOBAL PARAMETERS
# -----------------------
# Hough Transform parameters
delta_theta  = 1
delta_rho = 1
N_columns = 3


# Image source
fnames = ["image1.JPG", "image2.JPG", "image3.JPG", "image4.JPG", "image5.JPG"]
srcBase = "images/"

# Pla del perceptro


# Troba el valor màxim d'una matriu de dues dimensions
# INPUT:
#	H: 2D Matrix
#
# OUTPUT:
#	size: Radi de les casselles q es posaran a 0
#	coords: array(3) tal que 
#		[0]: coordenades X
#		[1]: coordenades Y
#       [2]: valor màxim de la matriu
def find_max(H):
	# Coord X, Coord Y, MAX
	max_coords_M = [-1,-1,-1]
	for s in range(len(H)):
		for t in range(len(H[0])):
			if(H[s][t]>max_coords_M[2]):
				max_coords_M[0] = s
				max_coords_M[1] = t
				max_coords_M[2] = H[s][t]
	
	return max_coords_M


# Posar un els NxN (neighbours_size) a 0 del pic mes alt
# INPUT:
#	H: 2D Matrix
#	size_theta: casselles verticals que es posaran a 0
#   size_rho: casselles horizontals que es posaran a 0
#	coords: array(2) tal que 
#		[0]: coordenades X
#		[1]: coordenades Y
# OUTPUT:
#   H: 2D Matrix
def reset_neighbour(H, size_theta, size_rho, coords):

	for i in range(-size_theta, size_theta+1):
		for j in range(-size_rho, size_rho+1):
			if((coords[0]+i >= 0 or coords[0]+i < len(H)) and (coords[1]+j >= 0 or coords[1]+j < len(H[0]))):
				H[coords[0]+i][coords[1]+j]=0
	return H


# Classificar la imatge segons el pla proporcionat per el perceptro
# INPUT:
#	patch d'una imatge
# OUTPUT:
#   	1: es porta
#	2: es pared
#	0: indeterminat
def perceptro_classify(patch):

	# Pla del perceptro
	w = np.array([707.0348077,-265.21500625, -353.34853089, -65.66563576], dtype=float)
	
	# Passar la imatge a HSV per calcular la saturacio
	patch_hsv = color.rgb2hsv(patch)
	
	# Passar la imatge a to de gris per calcular l'energia i homogeneitat
	patch_gray = color.rgb2gray(patch)
	
	# Obtenir saturacio
	saturacio = np.mean(patch_hsv[:,:,1])

	# Calcul de la GLCM
	GLCM = feature.graycomatrix((patch_gray*255).astype(int), [1], [0], levels=256,normed=True)
	# Obtenir energia
	energia = feature.graycoprops(GLCM, prop='energy')
	# Obtenir homogeneitat
	homogeneitat = feature.graycoprops(GLCM, prop='homogeneity')
	
	# Determinar a quin costat del pla es troba el punt
	xi = np.array([saturacio,energia[0][0],homogeneitat[0][0],1.0], dtype=float)

	# Es porta
	if(np.dot(np.transpose(w),xi) > 0):
		return 1
	# Es pared
	elif(np.dot(np.transpose(w),xi) < 0):
		return 2
	# El punt es troba en el pla, es indeterminat
	else:
		return 0


# CARREGAR IMATGES, EQUALITZAR EL SEU HISTOGRAMA I OBTENIR LA IMATGE DE CONTORNS
# Array de les imatges a carregar
images = []
# Array de les imatges de contorns
edge_img = []

for fname in fnames:
	# Llegir la imatge
	img = io.imread(srcBase+fname, as_gray=False, plugin=None)
	images.append(img)
	
	# Passar a escala de grisos
	img_gray = color.rgb2gray(img)
	
    	# Usar CLAHE per equalitzar l'histograma
	img_bytes = img_as_ubyte(img_gray)
	hist_clahe = exposure.equalize_adapthist(img_bytes, nbins=256)
	
    	# Aplicar Canny per obtenir la imatge de contorns
    	# S'usa una sigma de 1.5, ja que es la que millors resultats ha donat despres de varies proves
	filter_canny = feature.canny(hist_clahe, sigma=1.5)
	edge_img.append(filter_canny)

# Visualitzar les 5 imatges de contorns
idx = 1
fig = pyplot.figure()
for img in edge_img:
	ax = fig.add_subplot(2, 3, idx)
	imgplot = pyplot.imshow(img, cmap='gray')
	ax.set_title('Imatge ' + str(idx))
	idx+=1

pyplot.show()

# CALCULAR LA TRANSFORMADA DE HOUGH I TROBAR LES N LINIES RECTES

idx = 0

# Punts on es troben les linies rectes de les imatges
u_classificacio = [[] for i in range(len(edge_img))]

for img in edge_img:

	# Hough Transform
	width = len(img[0])						# Amplada de l'imatge
	height = len(img)						# Altura de l'imatge

	D = int(math.sqrt(width**2+height**2))			# Obtenir la diagonal de l'imatge 
	angle_theta = 10
	
	p = int(angle_theta*2+1/delta_theta)				# nº de graus a provar -angle_theta:angle_theta/delta_theta
	q = int(D/delta_rho+1)

	# Inicialize Hough matrix [p][q] to 0
	H = [[0 for y in range(q)] for x in range(p)]

	for h in range(height):
		# Per evitar que es pintin les contorns falsos que ha afegit la camara s'afegeix el 5:width-5
		for w in range(5, width-5):
			# Si es un pixel de contorn
			if img[h][w]:
				for theta in range(-angle_theta,angle_theta+1,delta_theta):
					rho = w*math.cos(math.radians(theta)) + h*math.sin(math.radians(theta))

					s = int(theta/delta_theta + 0.5) + angle_theta
					t = int(rho/delta_rho + 0.5)
					H[s][t] = H[s][t] + 1

	# Pintar les N columnes a partir dels N pics mes alts de la transformada de Hough
	for i in range(N_columns):
		# Coord X, Coord Y, MAX
		max_coords_M = find_max(H)                      # Obtenir valor maxim de tota la matriu
		 
		theta = (max_coords_M[0]-10)*delta_theta
		rho = (max_coords_M[1])*delta_rho

		# Calcular el punt on creuen la recte amb major pic de la transformada i la horizontal en el centre de l'imatge
		u = int((rho - int(width/2)*math.sin(math.radians(theta)))/math.cos(math.radians(theta)))
		u_classificacio[idx].append(u)

		# Pintar una linia vertical en l'anterior punt
		pyplot.plot([u, u],[0, height], color="red", linewidth=2)

		# Posar a 0 els veinats del pic mes alt, +-3 per theta i +-50 per rho per eliminar les 
		# linies de l'altre part marc de la porta ja collit
		reset_neighbour(H, 3, 50, [max_coords_M[0], max_coords_M[1]])

	# Mostrar la imatge amb les linies damunt
	pyplot.imshow(images[idx])

	idx+=1

	# Guardar i mostrar imatge resultant
	pyplot.savefig("image"+str(idx)+"_liniesVerticals.JPG")
	pyplot.show()


# CLASSIFICACIO DE LES IMATGES
idx = 0
for img in images:
	# Inicialitzar parametres

	img_width = len(img[0])
	img_height = len(img)
	
	# Afegir maxim i minim i ordenar-ho per tal de collir per blocs en el recorregut d'abaix
	u_classificacio[idx].append(0)
	u_classificacio[idx].append(img_width)
	u_classificacio[idx].sort()

	# Inicialitzar els resultats del perceptro
	perceptro_result= [[] for i in range(len(u_classificacio[idx])-1)]
	
	# Recorrer les diferents seccions de la imatge
	for u_c in range(len(u_classificacio[idx])-1):
		# Recorrer els la imatge de dalt a baix de 30 a 30 pixels
		for h in range(0,img_height-30,30):
			# Recorrer la imatge del principi d'una seccio fins a la seguent de 30 en 30 pixels
			for w in range(u_classificacio[idx][u_c],u_classificacio[idx][u_c+1]-30,30):
				# Obtenir un patch de la imatge
				img_patch = img[h:h+30,w:w+30]
				
				# Classificar el patch a partir del perceptro
				perceptro_result[u_c].append(perceptro_classify(img_patch))
	
	
	# Una vegada s'ha classificat tota la imatge, posar un quadrat negre damunt de les pareds, un blanc damunt els blocs indeterminats
	# i deixar les portes netes
	fig, ax = pyplot.subplots()
	for u_c in range(len(u_classificacio[idx])-1):
		# Recompte de patches classificats com a portes en cada seccio
		count_door = perceptro_result[u_c].count(1)
		# Recompte de patches classificats com a portes en cada seccio
		count_wall = perceptro_result[u_c].count(2)
		# Nombre total de patches classificats
		total = len(perceptro_result[u_c])

		# Si mes del 70% dels patchs ha estat classificat com a porta no pintar res
		if(count_door/total > 0.7):
			pass
		
		# Si mes del 60 dels patchs ha estat classificat com a pared pintar un rectangle negre damunt la seccio
		elif(count_wall/total > 0.6):
			rect_len = u_classificacio[idx][u_c+1]-u_classificacio[idx][u_c]
			ax.add_patch(patches.Rectangle((u_classificacio[idx][u_c],0),rect_len,img_height,fc='black'))
		
		# Sino pintar-hi un rectangle blanc damunt la seccio
		else:
			rect_len = u_classificacio[idx][u_c+1]-u_classificacio[idx][u_c]
			ax.add_patch(patches.Rectangle((u_classificacio[idx][u_c],0),rect_len,img_height,fc='white'))

	# Mostrar imatge tractada
	ax.imshow(img)

	idx+=1

	# Guardar i mostrar imatge resultant
	pyplot.savefig("image"+str(idx)+"_processed.JPG")
	pyplot.show()