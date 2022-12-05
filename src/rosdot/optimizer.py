import numpy as np
from scipy.optimize import minimize


def optimize(pi, pi_k, c, w, eta, ub, my_ub):
    
    func = lambda u: np.matmul(c,u.T) + np.matmul(w,u.T) + (eta/2) * np.sum((u - pi_k)**2)
    
    cons = ({'type': 'ineq', 'fun': lambda u: ub - u},
            {'type': 'ineq', 'fun': lambda u: my_ub - np.sum(u)})
    
    res = minimize(func, pi, method='SLSQP', constraints = cons)

    return res.x, w + (eta/2) * (res.x - pi_k)

