
def getParameterMap(n, m, nbObst, mObst, slack):
    pm = {}
    pm['wu'] = list(range(0, n))
    nPar = n
    pm['wvel'] = list(range(nPar, nPar + n))
    nPar += n
    pm['w'] = list(range(nPar, nPar + m))
    nPar += m
    if slack:
        pm['ws'] = list(range(nPar, nPar + 1))
        nPar += 1
    pm['g'] = list(range(nPar, nPar + m))
    nPar += m
    pm['obst'] = list(range(nPar, nPar + (mObst+1) * nbObst))
    nPar += (mObst+1) * nbObst
    pm['r_body'] = list(range(nPar, nPar + 1))
    nPar += 1
    pm['lower_limits'] = list(range(nPar, nPar + n))
    nPar += n
    pm['upper_limits'] = list(range(nPar, nPar + n))
    nPar += n
    nx = n * 2
    nu = n
    if slack:
        ns = 1
    else:
        ns = 0
    return pm, nPar, nx, nu, ns
