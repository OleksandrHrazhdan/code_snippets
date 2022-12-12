from maya import cmds
import math

# Create control points in maya scene
# Do it once, than update the curve
def createControlPoints(cpPos):
    controls = []
    for i in range(len(cpPos)):
        cp = cmds.createNode("joint", n = "cp_"+str(i))
        cmds.setAttr(cp+".radius", 0.5)
        cmds.setAttr(cp+".tx", cpPos[i][0])
        cmds.setAttr(cp+".ty", cpPos[i][1])
        controls.append(cp)
    return controls        
    
# Get control points positions
def getCpPositions(cvs):
    cvPos = []
    for cp in cvs:
        cv = [0,0]
        cv[0] = cmds.getAttr(cp+".tx")
        cv[1] = cmds.getAttr(cp+".ty")
        cvPos.append(cv)
    return cvPos        
                
# B-spline basis functions (De Boor's algorithm)
def B(x, k, i, t):
    if k == 0:
        return 1.0 if t[i] <= x < t[i+1] else 0.0
    if t[i+k] == t[i]:
        c1 = 0.0
    else:
        c1 = (x - t[i])/(t[i+k] - t[i])*B(x, k-1, i, t)
    if t[i+k+1] == t[i+1]:
        c2 = 0.0
    else:
        c2 = (t[i+k+1]-x)/(t[i+k+1] - t[i+1])*B(x, k-1, i+1, t)
    return c1+c2

# B-spline sampler
def bspline(x, t, c, k):
    n = len(c)
    assert(n >= k+1) and (len(c) >= n)
    s = [0.0,0.0]
    if x == 1.0:
        return list(c[-1])
    for i in range(n):
        bVal = B(x, k, i, t)
        s[0] += c[i][0]*bVal
        s[1] += c[i][1]*bVal
    return s

# B-spline knot vector creator
def createKnotVector(numCp, degree):
    tLen = numCp+degree+1
    knots = [0]*(tLen)
    # generage a knot vector
    for i in range(tLen):
        if i <= degree:
            knots[i] = 0
        elif i > tLen - degree - 2:
            knots[i] = tLen-2*degree - 1
        else:
            knots[i] = (i - degree)
    # normalize a knot vector
    m = max(knots)
    for i in range(tLen):
        knots[i] = knots[i]/float(m)
    return knots       
    
# B-spline sampler
def sampleBSpline(numSamples, degree, cvPos, knots):
    if cmds.objExists("bSpline"):
        cmds.delete("bSpline")
    curveGrp = cmds.createNode("transform", n = "bSpline")
    pSequence = []
    for i in range(numSamples+1):
        x = float(i)/numSamples
        bP = bspline(x, knots, cvPos, degree)
        pSequence.append(bP)
        jnt = cmds.createNode("joint")
        cmds.setAttr(jnt+".tx", bP[0])
        cmds.setAttr(jnt+".ty", bP[1])
        cmds.setAttr(jnt+".radius", 0.1)
        cmds.setAttr(jnt+".overrideEnabled", 1)
        cmds.setAttr(jnt+".overrideColor",13)
        cmds.parent(jnt, curveGrp)
    cmds.select(cl = 1)
    return pSequence   

# differentiate pSequence
def differentiate(pSequence):
    deltaSequence = []
    delta = [0.0, 0.0]
    for i in range(len(pSequence)-1):
        dx = pSequence[i+1][0] - pSequence[i][0]
        dy = pSequence[i+1][1] - pSequence[i][1]
        delta = [dx, dy]
        deltaSequence.append(delta)
    deltaSequence.append(delta)
    return deltaSequence
 
# computing the lengh sequence
def lengthSequence(deltaSequence):
    lengthSq = [0.0]
    for i in range(len(deltaSequence) - 1):
        lengthSq.append(lengthSq[-1] + math.sqrt(deltaSequence[i][0]**2+deltaSequence[i][1]**2))
    return lengthSq    
        
# computing closest index in lenght sequence
# binary search
def closestIndexInLengthSq(lengthSq, val):
    # baseCase
    mid = int(len(lengthSq)/2)
    if val > lengthSq[-1]:
        return len(lengthSq) - 1
    if val < lengthSq[0]:
        return 0 
    if val == lengthSq[mid]:
        return mid
    # recCase
    if val > lengthSq[mid]:
        return mid + closestIndexInLengthSq(lengthSq[mid+1:], val)
    elif val < lengthSq[mid]:
        return closestIndexInLengthSq(lengthSq[:mid], val)         
        
def interpolateAtLength(pSq, lengthSq, lengthVal):
    closestSample = closestIndexInLengthSq(lengthSq, lengthVal)
    lenDif = lengthVal - lengthSq[closestSample]
    if lengthVal < lengthSq[-1]:
        p0 = pSq[closestSample]
        p1 = pSq[closestSample+1]
        delta = [p1[0]-p0[0], p1[1]-p0[1]]
        deltaLen = math.sqrt(delta[0]**2+delta[1]**2)
        deltaNorm = [0.0, 0.0]
        if deltaLen > 0.0:
            deltaNorm = [delta[0]/deltaLen,delta[1]/deltaLen]
        deltaRescaled = [deltaNorm[0]*lenDif, deltaNorm[1]*lenDif]
        return [p0[0]+deltaRescaled[0], p0[1]+deltaRescaled[1]] 
    # boundary conditions
    if lengthVal > lengthSq[-1]:
        p0 = pSq[closestSample]
        p1 = pSq[closestSample-1]
        delta = [p0[0]-p1[0], p0[1]-p1[1]]
        deltaLen = math.sqrt(delta[0]**2+delta[1]**2)
        deltaNorm = [0.0, 0.0]
        if deltaLen > 0.0:
            deltaNorm = [delta[0]/deltaLen,delta[1]/deltaLen]
        deltaRescaled = [deltaNorm[0]*lenDif, deltaNorm[1]*lenDif]
        return [p0[0]+deltaRescaled[0], p0[1]+deltaRescaled[1]] 
         
# Run this code only once
# Than update the cureve
cpPos = [
(0,0),
(0.479, 1.475),
(2.536, 2.041),
(3.928, 0.675),
(6.049, 1.172),
(6.408, 2.863),
(5.421, 4.179),
]
cvNames = createControlPoints(cpPos)

# Build and update the curve
# Run this chunc only if you want to update the curve
# You can move control points, change number of samples or a degree
numSamples = 50
degree = 4 # Degree of a curve, you can try 1, 2, 3, 4 or 5
knots = createKnotVector(len(cvNames), degree)
cvPos = getCpPositions(cvNames)
pSq = sampleBSpline(numSamples, degree, cvPos, knots)
diffSq = differentiate(pSq)
lengthSq = lengthSequence(diffSq)
if cmds.objExists("InterpolJnt"):
    cmds.delete("InterpolJnt")
jntAtLen = cmds.createNode("joint", n = "InterpolJnt")

lengthVal = 3.4
posAtLen = interpolateAtLength(pSq, lengthSq, lengthVal)
     
cmds.setAttr(jntAtLen+".tx", posAtLen[0])
cmds.setAttr(jntAtLen+".ty", posAtLen[1])
cmds.setAttr(jntAtLen+".radius", 0.4)
cmds.setAttr(jntAtLen+".overrideEnabled", 1)
cmds.setAttr(jntAtLen+".overrideColor",17)

cmds.select(cl = 1)

