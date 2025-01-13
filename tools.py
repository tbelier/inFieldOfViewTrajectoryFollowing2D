import numpy as np
def cross_product(v1, v2):
    """Calcul du produit vectoriel en 2D."""
    return v1[0] * v2[1] - v1[1] * v2[0]

def is_same_side(pt, a, b, c):
    """
    Vérifie si le point 'pt' et le point 'c' sont du même côté de la droite (a, b).
    """
    ab = np.array([b[0] - a[0], b[1] - a[1]])
    ac = np.array([c[0] - a[0], c[1] - a[1]])
    ap = np.array([pt[0] - a[0], pt[1] - a[1]])
    
    cross_ap_ab = cross_product(ap, ab)
    cross_ap_ac = cross_product(ap, ac)
    return cross_ap_ab > 0 and  cross_ap_ac<0


def to_local_frame(x_global, y_global, x_robot, y_robot, theta_robot):
    dx = x_global - x_robot
    dy = y_global - y_robot
    cos_theta = np.cos(-theta_robot)
    sin_theta = np.sin(-theta_robot)
    x_local = cos_theta * dx + sin_theta * dy
    y_local = -sin_theta * dx + cos_theta * dy
    return x_local, y_local