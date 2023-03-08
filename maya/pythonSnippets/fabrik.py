def fabrik(seg, n, s, b):
    """
    seg - segment length,
    n - number of iterations
    s - samples 
    b - bend value
    """
    l = len(s) - 1
    # copy input
    orig = []
    for i in range(len(pSq)):
        orig.append(Vec3(pSq[i].x, pSq[i].y, pSq[i].z))
        
    # comput FABRIK
    for i in range(n*l):
        v0 = 0.5*(i - l)/l
        v1 = 0.5*(i+1-l)/l
        k0 = round(abs(v0 - math.floor(v0)- 0.5)*2*l)
        k1 = round(abs(v1 - math.floor(v1)- 0.5)*2*l)
        d = s[k1] - s[k0]
        
        dLen = d.length()
        dN = d.normal()
        dS = dN*seg 
        dif = dS - d
        s[k1] = s[k0] + d + dif*(1.0/math.e)*0.5
        s[k0] = s[k1] - d - dif*(1.0/math.e)*0.5

        # do bend constraint
        if(k0 > 0 and k0 < l):
            vb0 = 0.5*(i - 1 - l)/l
            vb1 = 0.5*(i + 1 - l)/l
            b0 = round(abs(vb0 - math.floor(vb0)- 0.5)*2*l)
            b1 = round(abs(vb1 - math.floor(vb1)- 0.5)*2*l)
            db = s[b1] - s[b0]
            dbLen = db.length()
       
            r = 1.0 - dbLen/(2.0*seg)
            m = s[b0] + db*0.5
            dV = (m - s[k0])*r
            
            s[k0] = s[k0] + dV*(0.1)*b
            s[b0] = s[b0] - dV*(0.1)*b
            s[b1] = s[b1] - dV*(0.1)*b
            
        # constraint ends
        s[0] = orig[0]
        s[-1] = orig[-1]
