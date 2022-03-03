import numpy as np
import cvxopt
from cvxopt import matrix, solvers
import cv2
from config import *

# cvxopt.solvers.options['maxiters'] = 1
cvxopt.solvers.options['show_progress'] = False


def plot(x0, y0, xg, yg, Vx, Vy, n, dt, OBSTACLES):
    canvas = 255 * np.ones((600, 600, 3), np.uint8)

    cv2.circle(canvas, (int(x0), int(y0)), 5, (0, 0, 255), -1)
    cv2.circle(canvas, (int(xg), int(yg)), 5, (0, 255, 0), -1)

    for obs in OBSTACLES:
        cv2.circle(canvas, (int(obs.x), int(obs.y)),
                   obs.R, (50, 220, 220), -1)

    x, y = x0, y0
    path_coordinates = [(int(x),int(y))]
    for i in range(n):
        x = x + Vx[i]*dt
        y = y + Vy[i]*dt
        path_coordinates.append((int(x),int(y)))
        cv2.line(canvas, path_coordinates[-2], path_coordinates[-1], (255,100,100), 1)
        cv2.circle(canvas, (int(x), int(y)), 3, (255, 100, 100), -1)

    canvas = cv2.flip(canvas, 0)

    cv2.namedWindow('MPC Path', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('MPC Path', 600, 600)
    cv2.imshow('MPC Path', canvas)
    cv2.waitKey()
    # cv2.imwrite("./images/no-obstracle.jpg", canvas)


def main():
    '''
     Driver function for MPC
     Convex Optimization Standard Form:
        min: (1/2)X.TPX + QX 
        st: 
            GX < H
            AX = B  [We don't have equality constraints in our use case]

    The formulation of P and Q matrices are similar to what is done here:   
        - https://courses.iiit.ac.in/pluginfile.php/122112/mod_resource/content/1/MPCNotesAvijit.pdf
    Only difference is that we have compined X and Y vectors to get a single variable vector

    '''
    # [OPTIMIZATION] Setting objective function
    p1 = np.ones((n, n))
    p2 = np.zeros((n, n))

    P = np.vstack([np.hstack([p1, p2]), np.hstack([p2, p1])]) * \
        (dt**2)
    # Multiplying by 2 because standard form has (1/2) in the quad term
    P = 2 * matrix(P, tc='d')

    q1 = 2 * (x0 - xg)
    q2 = 2 * (y0 - yg)

    Q = np.vstack([q1 * np.ones((n, 1)), q2 * np.ones((n, 1))]
                  ) * dt  # Do we have to use dt term?
    Q = matrix(Q)

    # [OPTIMIZATION] Setting inequality constraints (Velocity)
    G = np.vstack([np.diag(np.ones(2*n)), -1 * np.diag(np.ones(2*n))])
    H = np.vstack([vmax*np.ones((2*n, 1)), np.zeros((2*n, 1)), ])

    # [OPTIMIZATION] Initializing initial solution to zeros
    # V represents the control signals (Velocities) at each instant
    V = np.zeros(2*n)

    if debug:
        print('\n[INFO] Shape of Matrices:')
        print('Q:', Q.size)
        print('P:', P.size)
        print('G:', G.size)
        print('H:', H.size)

    for i in range(mpciters):
        # [OPTMIZATION] Adding obstacle constraints to G and H
        obstracles_g_list = []
        obstracles_h_list = []
        for obs in OBSTACLES:
            x_locs = np.expand_dims(
                np.cumsum(V[:n]), axis=1)*dt + x0-obs.x
            x_lower_triag = np.tril(np.ones(n)) * (-2)*dt*x_locs

            y_locs = np.expand_dims(
                np.cumsum(V[n:]), axis=1)*dt + y0-obs.y
            y_lower_triag = np.tril(np.ones(n)) * (-2)*dt*y_locs

            obstracle_g = np.hstack([x_lower_triag, y_lower_triag])

            obstracle_g_a = obstracle_g @ V

            obstracle_h = (x_locs**2 + y_locs**2 - obs.R**2)

            obstracle_h_final = obstracle_h + \
                np.expand_dims(obstracle_g_a, axis=1)

            if debug:
                print("[info] H final stats:", obstracle_h_final)
                print("[info] H stats:", obstracle_h)
                print("[info] Ga stats:", obstracle_g_a)
                print("[info] y stats:", y_locs, y_lower_triag)
                print("[info] x stats:", x_locs, x_lower_triag)

            obstracles_g_list.append(obstracle_g)
            obstracles_h_list.append(obstracle_h_final)

        G_loc = matrix(np.vstack([G, *obstracles_g_list]))
        H_loc = matrix(np.vstack([H, *obstracles_h_list]))

        if debug:
            print('\n[INFO] Running Solver..')
        # , initvals={"x":matrix(V)})

        # [OPTMIZATION] Running solver
        sol = solvers.qp(P, Q, G_loc, H_loc)

        V = np.array(sol['x'])
        Vx = V[0:n]
        Vy = V[n:]
        V = np.squeeze(V)

        plot(x0, y0, xg, yg, Vx, Vy, n, dt, OBSTACLES=OBSTACLES)


if __name__ == '__main__':
    main()
