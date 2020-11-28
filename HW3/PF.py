import pygame
import random, math
from numpy import dot, sum, tile, linalg, array, random, eye, zeros, diag, transpose, mean, var, cov
from numpy.linalg import inv, norm
import matplotlib.pyplot as plt
import itertools as it

#Basic Options
Noise = True
Linear = True
Ellipse = False

#Initialize the pygame
pygame.init()

#Create the screen
scale = 600
screen = pygame.display.set_mode((scale, scale))
sc_f = int(scale/20)
cov_size = 20
cov_scale = cov_size/sc_f

#Title, icon, text
pygame.display.set_caption("Particle Filter")
font = pygame.font.Font('freesansbold.ttf', 20)
X_text = font.render("X:", True, (255, 255, 255))
Y_text = font.render("Y:", True, (255, 255, 255))
if Linear == True:
    part = "a"
else:
    part = "b"
textX = 10
textY = 10

#Load robot
robot = pygame.Surface((3, 3))
robot.fill((255, 0, 0))

#Landmark
Landmark = pygame.Surface((10, 10))
Landmark.fill((255, 255, 255))
L = array([[10, 10]]).T

#Covariance ellipse
image = pygame.Surface([scale,scale], pygame.SRCALPHA, 32)
image = image.convert_alpha()

#---------------------------------------------------------------ROBOT PARAMETERS-------------------------------------------------------------------

#Initial Conditions
M = 100  #number of samples
bins = int(M/10)
x0 = 6.0
y0 = 10.0
theta = math.pi/2.0
thetaprev = 0
X = array([[x0, y0]]).T
P = zeros(2)

#Parameters
r = 0.1
rL = 0.2
T = float(1/8)
F = eye(2) #F remains identity matrix here

#Variables
ur = 4.75
ul = 5.25
u_w = (ur+ul)/2.0
u_phi = (ur-ul)*(r/rL)
U = array([[r*u_w*math.cos(theta),r*u_w*math.sin(theta)]]).T

#Process Noise
ww = 0.1
wphi = 0.01
Q = diag([ww+wphi, ww+wphi])

#Linear measurement parameters
C = diag([1, 1])
rx, ry = 0.05, 0.075
R = diag([rx, ry])
phi = theta + math.pi/2.0
dist_norm = 4.0

#True line
true_line = [(x0*sc_f, y0*sc_f), (x0, y0)]

def sample(X_prev, U):
    samples = []
    for i in range(M):
        X = X_prev
        noise_x = random.normal(0, ww)
        noise_y = random.normal(0, ww)
        noise = array([[noise_x, noise_y]]).T
        X_dot = (U + noise).reshape(2,)
        X = dot(F, X).reshape(2,) + (T*X_dot)#Xk equation 2 from HW2
        samples.append(X.reshape(2,))  
    return samples

def create_bins(x_dist, y_dist):
    X_max = max(x_dist)
    X_min = min(x_dist)
    Y_max = max(y_dist)
    Y_min = min(y_dist)
    max_min = (X_max, X_min, Y_max, Y_min)
    delta_x = (X_max - X_min)/10
    delta_y = (Y_max - Y_min)/10
    delta = [delta_x, delta_y]
    return max_min, delta

def create_dist(x_dist, y_dist, delta, *max_min):
    X_max, X_min, Y_max, Y_min = max_min
    delta_x, delta_y = delta[0], delta[1]
    px = [0]*bins
    py = [0]*bins
    p = []
    k_list = []
    for i in range(M):
        kx = int(math.ceil((x_dist[i]-X_min)/delta_x))
        ky = int(math.ceil((y_dist[i]-Y_min)/delta_y))
        if x_dist[i] == X_min:
            px[0] += 1
            kx = 1
        else:
            px[kx-1] += 1
        if y_dist[i] == Y_min:
            py[0] += 1
            ky = 1
        else:
            py[ky-1] += 1
        k_list.append((kx-1, ky-1))
        p = list(zip(px, py))
    return p, k_list

def probability(x_p, y_p, p_dist, delta, *max_min):
    X_max, X_min, Y_max, Y_min = max_min
    delta_x, delta_y = delta[0], delta[1]
    p = []
    for i in range(M):
        kx = (x_p[i]-X_min)/delta_x
        ky = (y_p[i]-Y_min)/delta_y
        if (kx < 0) or (kx > 10):
            px = 0
        else:
            kx = int(math.ceil(kx))
            if kx == 0:
                px = p_dist[0][0]
            else:
                px = p_dist[kx-1][0]
        if (ky < 0) or (ky > 10):
            py = 0
        else:
            ky = int(math.ceil(ky))
            if ky == 0:
                py = p_dist[0][1]
            else:
                py = p_dist[ky-1][1]
        prob = (px, py)
        p.append(prob)
    return p

def correct(pred):
    correct_samples = []
    for i in range(M):
        #Predict the no-noise model
        if Linear is True:
            n_x = random.normal(0, rx)
            n_y = random.normal(0, ry)
            n = array([n_x, n_y]).T
            Z = dot(C, pred[i]).reshape(2,) + n
            correct_samples.append(Z)
        else:
            dist = pred[i] - L.reshape(2,)
            n_w = random.normal(0, ww)
            n_phi = random.normal(0, wphi)
            phi = theta + math.pi/2.0
            phi_n = phi + n_phi
            
            dist_norm = norm(dist, ord = 2) + n_w
            x = 10 + (dist_norm*math.cos(phi_n))
            y = 10 + (dist_norm*math.sin(phi_n))
            Z = array([x, y])
            correct_samples.append(Z)
    return correct_samples

def get_weights(prob_p, k_p, prob_c, k_c):
    w = []
    for i in range(M):
        wx = (prob_c[i][0])/(prob_p[k_p[i][0]][0])
        wy = (prob_c[i][1])/(prob_p[k_p[i][1]][1])
        w.append((wx, wy))
    return w

def resample(w, x_prior, y_prior):
    wx = [x[0] for x in w]
    wy = [y[1] for y in w]
    beta, X_post = [], []
    sumx = sum(wx)
    sumy = sum(wy)
    for i in range(M):
        b_x = sum(wx[0:i+1])/sumx
        b_y = sum(wy[0:i+1])/sumy
        beta.append((b_x, b_y))
    rhox = random.uniform(0, 1)
    rhoy = random.uniform(0, 1)
    index_x = [0]*M
    index_y = [0]*M
    for i in range(M):
        indx = len(list(it.takewhile(lambda x: x < rhox, [x[0] for x in beta])))
        x_new = x_prior[indx]
        indy = len(list(it.takewhile(lambda x: x < rhoy, [y[1] for y in beta])))
        y_new = y_prior[indy]
        X_post.append((x_new, y_new))
        rhox += 1/M
        rhoy += 1/M
        if rhox >= 1:
            rhox = random.uniform(0, 1)
        if rhoy >= 1:
            rhoy = random.uniform(0, 1)
        index_x[indx] += 1
        index_y[indy] += 1
    index = list(zip(index_x, index_y))
    return beta, X_post, index

def update_text(X, Y, coord):
    X_text = font.render("X:" + str(coord), True, (255, 255, 255))
    screen.blit(X_text, (X, Y))

if Noise == False:
    ww, wphi, rx, ry = 0, 0, 0, 0
i = 0
running = True
screen.fill((0, 0, 0))
screen.blit(Landmark, (int(scale/2), int(scale/2)))
R_text = font.render("Radius of circle: 4m", True, (255, 255, 255))
Q_text = font.render("HW3 Part " + str(part), True, (255, 255, 255))
C_text = font.render("Covariance Ellipse Scale: " + '{:01.2f}'.format(cov_scale), True, (255, 255, 255))
B_text = font.render("Bearing: 90", True, (255, 255, 255))
screen.blit(R_text, (textX, textY))
screen.blit(Q_text, (textX, textY+20))
screen.blit(B_text, (textX, textY+60))

while running:

    theta_noise = random.normal(0, wphi)
    theta_dot = (r/rL)*u_phi + theta_noise
    theta = (T*theta_dot) + theta
    U = array([[r*u_w*math.cos(theta),r*u_w*math.sin(theta)]]).T

    predict_samples = sample(X, U)

    x_pred = [x[0] for x in predict_samples]
    y_pred = [y[1] for y in predict_samples]
    max_min, delta = create_bins(x_pred, y_pred)
    prob_p, k_p = create_dist(x_pred, y_pred, delta, *max_min)

    correct_samples = correct(predict_samples)
    x_cor = [x[0] for x in correct_samples]
    y_cor = [y[1] for y in correct_samples]
    max_min_c, delta_c = create_bins(x_cor, y_cor)
    prob_c, k_c = create_dist(x_cor, y_cor, delta_c, *max_min_c)

    prob_post = probability(x_pred, y_pred, prob_c, delta_c, *max_min_c)
    weights = get_weights(prob_p, k_p, prob_post, k_c)
    beta, posterior, indices = resample(weights, x_pred, y_pred)
    x_post = [x[0] for x in posterior]
    y_post = [y[1] for y in posterior]

    indx = [x[0] for x in indices]
    indy = [y[1] for y in indices]

    max_x = max(indx)
    i_xmax = [i for i, j in enumerate(indx) if j == max_x]

    max_y = max(indy)
    i_ymax = [i for i, j in enumerate(indy) if j == max_y]

    avgx = 0
    for i in i_xmax:
        avgx += x_pred[i]
    avgx = avgx/len(i_xmax)

    avgy = 0
    for i in i_ymax:
        avgy += y_pred[i]
    avgy = avgy/len(i_ymax)

    X = array([[avgx, avgy]]).T

    if theta < 0:
        theta = math.pi*2.0

    # i += 1 
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    pygame.time.wait(125)
    
    true_line[1] = (int(avgx*sc_f), int(avgy*sc_f))
    pygame.draw.lines(screen, (0, 0, 255), False, true_line, 3)
    true_line[0] = true_line[1]

    dist = X - L
    dist = dist[:,0]
    dist_norm = norm(dist, ord = 2)

    # #print('Angle %2.2f Norm: %2.2f X: %2.2f Y: %2.2f' %(theta*180.0/math.pi, dist_norm, X[0][0], X[1][0]))

    screen.blit(robot, (int(avgx*sc_f), int(avgy*sc_f)))

    n_phi = random.normal(0, wphi)
    Bearing = (theta + math.pi/2.0 + n_phi)*180.0/math.pi
    if (theta*180.0/math.pi) > 270:
        Bearing = Bearing - 360
    Bearing_print = '{:03.2f}'.format(Bearing)
    N_print = '{:01.2f}'.format(dist_norm)
    N_text = font.render("Norm: " + N_print, True, (255, 255, 255))
    B_text = font.render("Bearing: " + Bearing_print, True, (255, 255, 255))
    screen.fill((0, 0, 0), (0, scale//12, scale, scale//6))
    screen.blit(Landmark, (int(scale/2), int(scale/2)))
    screen.blit(N_text, (textX, textY+40))
    screen.blit(B_text, (textX, textY+60))
    if Ellipse is True:
        screen.blit(C_text, (textX, textY+80))

    pygame.display.update()
