import pygame
import random, math
from numpy import dot, sum, tile, linalg, array, random, eye, zeros, diag, transpose, mean, var, cov
from numpy.linalg import inv, norm
import matplotlib.pyplot as plt

#Basic Options
Noise = True
Linear = False
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
pygame.display.set_caption("Kalman Filter")
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

#Initial Conditions
M = 10  #number of samples
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
delf_delw = array([T, T])
delf_delphi = (T**2)*r*u_w* array([-math.sin(theta), math.cos(theta)])
w_k = array([ww*delf_delw[0]+wphi*delf_delphi[0], ww*delf_delw[1]+wphi*delf_delphi[1]])
Q_p = diag([w_k[0], w_k[1]])

#Measurement parameters
C = diag([1, 2])
rx, ry = 0.05, 0.075
R = diag([rx, ry])
phi = theta + math.pi/2.0
dist_norm = 4.0

#True line
true_line = [(x0*sc_f, y0*sc_f), (x0, y0)]

def update_text(X, Y, coord):
    X_text = font.render("X:" + str(coord), True, (255, 255, 255))
    screen.blit(X_text, (X, Y))

def kf_predict(X, P, F, Q_p, U):
    noise_x = random.normal(0, ww)
    noise_y = random.normal(0, ww)
    noise = array([[noise_x, noise_y]]).T
    X_dot = U + noise
    X = dot(F, X) + (T*X_dot)
    P = dot(F, dot(P, F.T)) + Q_p
    return X, P

#Basic correction
def kf_correct(X, P, Y, C, R):
    IM = dot(C, X)
    IS = R + dot(C, dot(P, C.T))
    K = dot(P, dot(C.T, inv(IS)))
    X = X + dot(K, (Y-IM))
    P = P - dot(K, dot(C, P))
    return (X, P, K, IM, IS)

def range_bearing(d_norm, phi):
    return array([10+(d_norm*math.cos(phi)), 10+(d_norm*math.sin(phi))])

if Noise == False:
    ww, wphi, rx, ry = 0, 0, 0, 0
i = 0
running = True
screen.fill((0, 0, 0))
screen.blit(Landmark, (int(scale/2), int(scale/2)))
R_text = font.render("Radius of circle: 4m", True, (255, 255, 255))
Q_text = font.render("HW2 Part " + str(part), True, (255, 255, 255))
C_text = font.render("Covariance Ellipse Scale: " + '{:01.2f}'.format(cov_scale), True, (255, 255, 255))
B_text = font.render("Bearing: 90", True, (255, 255, 255))
screen.blit(R_text, (textX, textY))
screen.blit(Q_text, (textX, textY+20))
screen.blit(B_text, (textX, textY+60))

while running:
    i += 1 
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    pygame.time.wait(125)

        theta_noise = random.normal(0, wphi)
    theta_dot = (r/rL)*u_phi + theta_noise
    theta = (T*theta_dot) + theta
    theta_no_noise = (T*(r/rL)*u_phi) + theta
    U = array([[r*u_w*math.cos(theta),r*u_w*math.sin(theta)]]).T

    delf_delphi = (T**2)* array([-r*u_w*math.sin(theta_no_noise), r*u_w*math.cos(theta_no_noise)])
    w_k = array([ww*delf_delw[0]+wphi*delf_delphi[0], ww*delf_delw[1]+wphi*delf_delphi[1]])
    Q_p = diag([w_k[0], w_k[1]])

    X, P = kf_predict(X, P, F, Q_p, U)

    if i%8 == 0:
        if Linear == True:
            n_x = random.normal(0, rx)
            n_y = random.normal(0, ry)
            n = diag([n_x, n_y])
            Z = dot(C, X) + n
            X, P, K, IM, IS = kf_correct(X, P, Z, C, R)
            true_line[1] = (int(X[0][0]*sc_f), int(X[1][0]*sc_f))
            pygame.draw.lines(screen, (0, 0, 255), False, true_line, 3)
            true_line[0] = true_line[1]
        else:
            dist = X - L
            dist = dist[:, 0]
            n_w = random.normal(0, ww)
            n_phi = random.normal(0, wphi)
            dist_norm = norm(dist, ord = 2)
            dist_norm_n = dist_norm + n_w
            phi = theta + math.pi/2.0
            phi_n = phi + n_phi
            #print('Before %2.2f %2.2f %2.2f \n' %(X[0, 0], X[1, 0], dist_norm))

            #Update covariance matrix R
            delg_delw = array([math.cos(phi), math.sin(phi)])
            delg_delphi = dist_norm* array([-math.sin(phi), math.cos(phi)])
            #n_kp = transpose(array([delg_delw, delg_delphi]))
            #R_p = cov(n_kp, bias=True)

            n_k = array([ww*delg_delw[0]+wphi*delg_delphi[0], ww*delg_delw[1]+wphi*delg_delphi[1]])
            R_p = diag([n_k[0], n_k[1]])

            Z = range_bearing(dist_norm_n, phi_n)
            IM = range_bearing(dist_norm, phi)
            G1 = array([(X[0, 0] - 10.0)*math.cos(phi), (X[1,0] - 10.0)*math.cos(phi)])
            G2 = array([(X[0, 0] - 10.0)*math.sin(phi), (X[1,0] - 10.0)*math.sin(phi)])
            G = (1.0/dist_norm)*array([G1, G2])
            #G = G.reshape(2, 2)
            X, P, K, IM, IS = kf_correct_bm(X, P, Z, G, R_p, IM)
            true_line[1] = (int(X[0, 0]*sc_f), int(X[1, 0]*sc_f))
            pygame.draw.lines(screen, (0, 0, 255), False, true_line, 3)
            true_line[0] = true_line[1]

    Xx = (X[0][0]*sc_f)
    Xy = (X[1][0]*sc_f)

    Px = (P[0][0])
    Py = (P[1][1])

    dist = X - L
    dist = dist[:,0]
    dist_norm = norm(dist, ord = 2)

    #print('Angle %2.2f Norm: %2.2f X: %2.2f Y: %2.2f' %(theta*180.0/math.pi, dist_norm, X[0][0], X[1][0]))
    
    if theta < - 0:
        theta = math.pi*2.0

    screen.blit(robot, (int(Xx), int(Xy)))

    #print('CovX %2.4f CovY %2.4f' %(Px, Py))
    if Ellipse == True:
        try:
            pygame.draw.ellipse(screen, (0, 255, 0), (int(Xx-(Px*cov_size/2)), int(Xy-(Py*cov_size/2)), int(Px*cov_size), int(Py*cov_size)), 1)
            # ellipse_r = pygame.transform.rotate(image, 45)
            # screen.blit(ellipse_r, (int(rx), int(ry)))
        except:
            print('CovX %2.4f CovY %2.4f' %(Px, Py))

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
