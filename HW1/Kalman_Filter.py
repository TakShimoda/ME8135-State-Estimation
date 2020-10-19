import pygame
import random, math
from numpy import dot, sum, tile, linalg, array, random, eye, zeros, diag
from numpy.linalg import inv
import matplotlib.pyplot as plt

#Initialize the pygame
pygame.init()

#Create the screen
scale = 600
screen = pygame.display.set_mode((scale, scale))

#Title, icon, text
pygame.display.set_caption("Kalman Filter")
font = pygame.font.Font('freesansbold.ttf', 32)
score = font.render("Kalman", True, (255, 255, 255))
textX = 10
textY = 10

#Load robot
robot = pygame.Surface((5, 5))
robot.fill((255, 0, 0))
error = pygame.Surface((5, 5))
error.fill((255, 0, 0))
rx, ry = 0, 0

#Covariance ellipse
image = pygame.Surface([700,700], pygame.SRCALPHA, 32)
image = image.convert_alpha()

#Parameters
r = 0.1
velocity = 0.1
T = float(1/8)
ur = 1
ul = 1
U = array([[1,1]]).T
A = eye(2)
Qx, Qy = 0.15, 0.15
Qxy = Qx*Qy
Q = array([[Qx, Qxy], [Qxy, Qy]])

#Measurement parameters
C = diag([1, 2])
rx, ry = 0.05, 0.075
rxy = rx*ry
R = array([[rx, rxy], [rxy, ry]])

#Initial Conditions
x0 = 0
y0 = 0
X = array([[x0, y0]]).T
P = zeros(2)

#True line
true_line = [(0, 0), (0, 0)]

def kf_predict(X, P, A, Q, U):
    noise_x = random.normal(0, Q[0, 0])
    noise_y = random.normal(0, Q[1, 1])
    noise = array([[noise_x, noise_y]]).T
    X_dot = r/2 * (U + noise)
    X = dot(A, X) + (T*X_dot)
    P = dot(A, dot(P, A.T)) + Q
    return X, P

def kf_correct(X, P, Y, C, R):
    IM = dot(C, X)
    IS = R + dot(C, dot(P, C.T))
    K = dot(P, dot(C.T, inv(IS)))
    X = X + dot(K, (Y-IM))
    P = P - dot(K, dot(C, P))
    #LH = gauss_pdf(Y, IM, IS)
    return (X, P, K, IM, IS)#, LH)

def gausspdf(X, M, S):
    if M.shape()[1] == 1: #if column of CX is 1
        DX = X - tile(M, X.shape()[1])  #repeat M as many times as Y has columns, then do Y - CX innovation term
        E = 0.5 * sum(DX * (dot(inv(S), DX)), axis=0)   # IS^-1 * Innovation, then element-wise mul
        E = E + 0.5 * M.shape()[0] * log(2 * pi) + 0.5 * log(det(S))
        P = exp(-E)
    elif X.shape()[1] == 1:
        DX = tile(X, M.shape()[1])- M
        E = 0.5 * sum(DX * (dot(inv(S), DX)), axis=0)
        E = E + 0.5 * M.shape()[0] * log(2 * pi) + 0.5 * log(det(S))
        P = exp(-E)
    else:
        DX = X-M
        E = 0.5 * dot(DX.T, dot(inv(S), DX))
        E = E + 0.5 * M.shape()[0] * log(2 * pi) + 0.5 * log(det(S))
        P = exp(-E)
    return (P[0],E[0]) 

i = 0
running = True
screen.fill((0, 0, 0))
while running:
    i += 1     
    #screen.fill((0, 0, 0))
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    pygame.time.wait(125)
    if i<81:
        X, P = kf_predict(X, P, A, Q, U)

        rx = (X[0][0]*scale)
        ry = (X[1][0]*scale)
        Px = (P[0][0])
        Py = (P[1][1])

        if i%8 == 0:
            n_x = random.normal(0, R[0, 0])
            n_y = random.normal(0, R[1, 1])
            n = array([[n_x, n_y]]).T
            Z = dot(C, X) + R
            X, P, K, IM, IS = kf_correct(X, P, Z, C, R)
            true_line[1] = (int(X[0][0]*scale), int(X[1][0]*scale))
            pygame.draw.lines(screen, (0, 0, 255), False, true_line, 3)
            true_line[0] = true_line[1]

        screen.blit(robot, (int(rx), int(ry)))

        try:
            pygame.draw.ellipse(screen, (0, 255, 0), (int(rx-(Px*63)), int(ry-(Py*63)), int(Px*126), int(Py*126)), 2)
            # ellipse_r = pygame.transform.rotate(image, 45)
            # screen.blit(ellipse_r, (int(rx), int(ry)))
        except:
            print(Px, Py)

    pygame.display.update()
