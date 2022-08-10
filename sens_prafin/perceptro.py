# AUTOR: MIQUEL VIVES MARCUS

import os, os.path
import skimage.io as io
import matplotlib.pyplot as pyplot
from skimage.color import rgb2hsv, rgb2gray
from numpy import mean, amax, dot, transpose
import numpy as numpy
import skimage.feature as feature
from random import seed
from random import random as rand
from mpl_toolkits import mplot3d

# -----------------------------------
# GLOBAL PARAMETERS
# -----------------------------------
# Perceptro
n_iterations = 5000				# Si son mes el resultat final no varia, pero si son menys el pla resultant te major nombre d'incorrectament classificats
rho = 0.7

# Ubicacio dels fitxers
DIR_door = '/home/miq/Documents/sens_pra3/patches/door'
DIR_wall = '/home/miq/Documents/sens_pra3/patches/wall'


# -----------------------------------
# OBTENIR CARACTERISTIQUES
# -----------------------------------

# Carregar tots els noms de des imatges
images_door = [name for name in os.listdir(DIR_door)]
images_wall = [name for name in os.listdir(DIR_wall)]

# Calcular les caracteristiques: saturacio, energia i homogeneitat i assignar les dues classes
data = []
for image_door in images_door:
	img = io.imread(DIR_door+"/"+image_door, as_gray=False, plugin=None)
	# Passar la imatge a HSV per calcular la saturacio
	img_hsv = rgb2hsv(img)
	# Passar la imatge a to de gris per calcular l'energia i homogeneitat
	img_gray = rgb2gray(img)
	
	# Obtenir saturacio
	saturacio = mean(img_hsv[:,:,1])


	# Calcul de la GLCM
	GLCM = feature.graycomatrix((img_gray*255).astype(int), [1], [0], levels=256,normed=True)
	# Obtenir energia
	energia = feature.graycoprops(GLCM, prop='energy')
	# Obtenir homogeneitat
	homogeneitat = feature.graycoprops(GLCM, prop='homogeneity')
	
	# Afegir-ho indicant que son de la classe 1
	data.append([saturacio,energia[0][0],homogeneitat[0][0],1])

for image_wall in images_wall:
	img = io.imread(DIR_wall+"/"+image_wall, as_gray=False, plugin=None)
	# Passar la imatge a HSV per calcular la saturacio
	img_hsv = rgb2hsv(img)
	# Passar la imatge a to de gris per calcular l'energia i homogeneitat
	img_gray = rgb2gray(img)
	
	# Obtenir saturacio
	saturacio = mean(img_hsv[:,:,1])


	# Calcul de la GLCM
	GLCM = feature.graycomatrix((img_gray*255).astype(int), [1], [0], levels=256,normed=True)
	# Obtenir energia
	energia = feature.graycoprops(GLCM, prop='energy')
	# Obtenir homogeneitat
	homogeneitat = feature.graycoprops(GLCM, prop='homogeneity')
	
	# Afegir-ho indicant que son de la classe 2
	data.append([saturacio,energia[0][0],homogeneitat[0][0],2])



# -----------------------------------
# CALCUL PERCEPTRO
# -----------------------------------
# Comptar nombre de portes i nombre de pared
n_door = len([name for name in os.listdir(DIR_door) if os.path.isfile(os.path.join(DIR_door, name))])
n_wall = len([name for name in os.listdir(DIR_wall) if os.path.isfile(os.path.join(DIR_wall, name))])

# Total
n = n_door + n_wall

# Inizialitzar a un nombre aleatori W
seed(1)
w0 = rand()
w1 = rand()
w2 = rand()
w3 = rand()

w = numpy.array([w1,w2,w3,w0], dtype=float)

# Copiar a la matriu X les dades de les caracteristiques posant 1 a la darrera columna
x=[]
for d in range(len(data)):
	x.append([data[d][0],data[d][1],data[d][2],1])

# Inicialtizar la butxaca
ic_menor = n
w_butxaca = numpy.zeros(4)

# Calcul del perceptro
for i in range(n_iterations):
	# Inicialtizar el vector sum i el nombre d'incorrectament classificades
	suma = numpy.zeros(4)
	ic = 0
	
	# Calcular la suma
	for k in range(n):
		
		xi = numpy.array([x[k][0], x[k][1], x[k][2], x[k][3]], dtype=float)

		if(dot(transpose(w),xi) < 0 and data[k][3] == 1):

			suma = suma + rho*xi
			ic = ic + 1
		elif(dot(transpose(w),xi) > 0 and data[k][3] == 2):

			suma = suma - rho*xi
			ic = ic + 1
	
	# Actualitzar W
	w = w + suma
	
	# Si el nombre d'incorrectament classificades es 0 aturar
	if(ic == 0):
		ic_menor = 0
		w_butxaca = w
		break
	# Si el nombre d'incorrectament classificades es menor al minim vist, actualitzar la butxaca
	if(ic<ic_menor):
		ic_menor = ic
		w_butxaca = w

# Guardar la butxaca
w = w_butxaca
print("Nombre total d'incorrectament classificats: ", ic_menor, "  -  vector w:", w)

# Obtenir pla que talla els dos nuvols de punts
ax = pyplot.axes(projection='3d')
x = y = numpy.arange(0.0, 1.0, 0.1)
[xx,yy]=numpy.meshgrid(x,y)
z=(-w[0]*xx - w[1]*yy-w[3])/w[2]
ax.plot_surface(xx,yy,z)

# Els punts porta seran vermells
for i in range(n_door):
	ax.scatter(data[i][0],data[i][1],data[i][2], c="red")

# Els punts pared seran verds
for i in range(n_door,n):
	ax.scatter(data[i][0],data[i][1],data[i][2], c="green")
pyplot.show()




















