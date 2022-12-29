from maya import cmds
import math
import maya.api.OpenMaya as om 
cmds.file(f = 1, new = True)

# 3d vector class 
class Vec3(object):
    def __init__(self, x = 0.0, y = 0.0, z = 0.0):
        self.x = x 
        self.y = y 
        self.z = z
         
    # add in place += 
    def __iadd__(self, other):
        if isinstance(other, Vec3):
            self.x += other.x 
            self.y += other.y 
            self.z += other.z
            return self
        else:
            raise Exception("{} is not a vector".format(other))
    
    # add two vectors +
    def __add__(self, other):
        if isinstance(other, Vec3):
            return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)
        else:
            raise Exception("{} is not a vector".format(other))
    
    # subtract in place -=
    def __isub__(self, other):
        if isinstance(other, Vec3):
            self.x -= other.x 
            self.y -= other.y 
            self.z -= other.z 
        else:
            raise Exception("{} is not a vector".format(other))
    
    # subtract two vectors - 
    def __sub__(self, other):
        if isinstance(other, Vec3):
            return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)
        else:
            raise Exception("{} is not a vector".format(other))        
    
    # length of a vector 
    def length(self):
        return math.sqrt(self.sqLen())
    
    # vector dot with itself v*v
    def sqLen(self):
        return self.x**2 + self.y**2 + self.z**2 

    # dot product *
    def __mul__(self, other):
        if isinstance(other, Vec3):
            return self.x*other.x + self.y*other.y + self.z*other.z
        elif isinstance(other, float):
            return Vec3(self.x*other, self.y*other, self.z*other)
        return None 
    
    # cross product ^    
    def __xor__(self, other):
        return Vec3(
        self.y*other.z - self.z*other.y,
        self.z*other.x - self.x*other.z,
        self.x*other.y - self.y*other.x
        )
    
    # vector string representation
    def __str__(self): 
        return str("[{}, {}, {}]".format(self.x, self.y, self.z))
    
    # normalize vector (returns normalized version of a vector)
    def normal(self):
        vLen = self.length()
        if vLen > 0:
            return Vec3(self.x/vLen, self.y/vLen, self.z/vLen)
        return Vec3()

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
    s = Vec3()
    if x == 1.0:
        return c[-1]
    for i in range(n):
        bVal = B(x, k, i, t)
        s += c[i]*bVal
    return s
    
# Create control points in maya scene
# Do it once, than update the curve
def createControlPoints(cpPos, radius = 0.5):
    controls = []
    for i in range(len(cpPos)):
        cp = cmds.createNode("joint", n = "cp_"+str(i))
        cmds.setAttr(cp+".radius", radius)
        cmds.setAttr(cp+".tx", cpPos[i].x)
        cmds.setAttr(cp+".ty", cpPos[i].y)
        cmds.setAttr(cp+".tz", cpPos[i].z)
        controls.append(cp)
    return controls        
    
# B-spline sampler
def sampleBSpline(numSamples, degree, cvPos, knots):
    pSequence = []
    for i in range(numSamples):
        x = float(i)/(numSamples-1)
        bP = bspline(x, knots, cvPos, degree)
        pSequence.append(bP)
    return pSequence 
        
# Get control points positions
def getCpPositions(cvs):
    cvPos = []
    for cp in cvs:
        cv = Vec3()
        cv.x = cmds.getAttr(cp+".tx")
        cv.y = cmds.getAttr(cp+".ty")
        cv.z = cmds.getAttr(cp+".tz")
        cvPos.append(cv)
    return cvPos        

# create maya joint for every sample point with radius parameter
def samplePoints(points, radius = 0.02, colorId = 13, hostGroup = ""):
    if hostGroup:
        if cmds.objExists(hostGroup):
            cmds.delete(hostGroup)
        curveGrp = cmds.createNode("transform", n = hostGroup)
    preJnt = ""
    for p in points:
        jnt = cmds.createNode("joint")
        cmds.setAttr(jnt+".tx", p.x)
        cmds.setAttr(jnt+".ty", p.y)
        cmds.setAttr(jnt+".tz", p.z)
        cmds.setAttr(jnt+".radius", radius)
        cmds.setAttr(jnt+".overrideEnabled", 1)
        cmds.setAttr(jnt+".overrideColor",colorId)
        if preJnt:
            cmds.parent(jnt, preJnt)
        else:
            if hostGroup:
                cmds.parent(jnt, curveGrp)
        preJnt = jnt
    cmds.select(cl = 1)    
    
# differentiate pSequence
def differentiate(pSequence):
    deltaSequence = []
    delta = Vec3()
    for i in range(len(pSequence)-1):
        delta = pSequence[i+1] -  pSequence[i]
        deltaSequence.append(delta)
    deltaSequence.append(delta)
    return deltaSequence
 
# computing the lengh sequence
def lengthSequence(deltaSequence):
    lengthSq = [0.0]
    for i in range(len(deltaSequence) - 1):
        lengthSq.append(lengthSq[-1] + deltaSequence[i].length())
    return lengthSq    
        
# computing closest index in lenght sequence (binary search)
def closestIndexInLengthSq(lengthSq, val):
    # baseCase
    mid = int((len(lengthSq))/2)
    if val >= lengthSq[-1]:
        return len(lengthSq) - 1
    if val <= lengthSq[0]:
        return 0
    # recCase
    if val > lengthSq[mid]:
        return mid + closestIndexInLengthSq(lengthSq[mid:], val) 
    if val < lengthSq[mid]:
        return closestIndexInLengthSq(lengthSq[:mid], val)
    return mid

# interpolate point at length
def interpolateAtLength(pSq, lengthSq, lengthVal):
    closestSample = closestIndexInLengthSq(lengthSq, lengthVal)
    lenDif = lengthVal - lengthSq[closestSample]
    if lengthVal < lengthSq[-1]:
        p0 = pSq[closestSample]
        p1 = pSq[closestSample+1]
        delta = p1-p0
        deltaLen = delta.length()
        deltaNorm = delta.normal()
        return p0 + deltaNorm*lenDif
    # boundary conditions
    if lengthVal >= lengthSq[-1]:
        p0 = pSq[closestSample]
        p1 = pSq[closestSample-1]
        delta = p0-p1
        deltaLen = delta.length()
        deltaNorm = delta.normal()
        return p0 + deltaNorm*lenDif

# get tangent vector at length
def getTangentAtLength(deltaSq,lengthSq, lengthVal):
    closestSample = closestIndexInLengthSq(lengthSq, lengthVal)
    # inside the curve
    if lengthVal < lengthSq[-1] and lengthVal > 0:
        lenRatio = (lengthVal - lengthSq[closestSample])/(lengthSq[closestSample+1] - lengthSq[closestSample])
        tangentA = (deltaSq[closestSample] + deltaSq[closestSample-1])*0.5
        tangentB = (deltaSq[closestSample] + deltaSq[closestSample+1])*0.5
        tangentBlend = tangentA*(1-lenRatio) + tangentB*(lenRatio)
        tangentBlendNorm = tangentBlend.normal()
        return tangentBlendNorm
    if lengthVal > 0 and lengthVal < lengthSq[1]:
        lenRatio = (lengthVal - lengthSq[closestSample])/(lengthSq[closestSample+1] - lengthSq[closestSample])
        tangentB = (deltaSq[closestSample] + deltaSq[closestSample+1])*0.5
        tangentBlend = tangentA*(1-lenRatio) + tangentB*(lenRatio)
        tangentBlendNorm = tangentBlend.normal()
        return tangentBlendNormal
    if lengthVal >= lengthSq[-1]:
        tangentA = deltaSq[-1]
        return tangentA.normal()
    if lengthVal < 0:
        tangentA = deltaSq[0]
        return tangentA.normal()
    return Vec3()

def drawTangent(tangent, posAtLen):
    if cmds.objExists("tgA"):
        cmds.delete("tgA")
    jntA = cmds.createNode("joint", n = "tgA")
    jntB = cmds.createNode("joint", n = "tgB")
    jntC = cmds.createNode("joint", n = "tgC")
    cmds.setAttr(jntA+".overrideEnabled", 1)
    cmds.setAttr(jntA+".overrideColor",14)
    cmds.setAttr(jntB+".tx", tangent.x)
    cmds.setAttr(jntB+".ty", tangent.y)
    cmds.setAttr(jntB+".tz", tangent.z)
    cmds.setAttr(jntC+".tx", -tangent.x)
    cmds.setAttr(jntC+".ty", -tangent.y)
    cmds.setAttr(jntC+".tz", -tangent.z)
    cmds.setAttr(jntB+".radius", 0.01)
    cmds.setAttr(jntC+".radius", 0.01)
    cmds.setAttr(jntA+".radius", 0.01)
    cmds.parent(jntB, jntA)
    cmds.parent(jntC, jntA)
    cmds.setAttr(jntA+".tx", posAtLen.x)
    cmds.setAttr(jntA+".ty", posAtLen.y)
    cmds.setAttr(jntA+".tz", posAtLen.z)
    cmds.select(cl = 1)

    
# Run this code only once
# Then update the curve
cpPos = [
Vec3(0,0,0),
Vec3(0.479, 1.475, 0.0),
Vec3(2.536, 2.041, 0.0),
Vec3(3.928, 0.675, 0.0),
Vec3(6.049, 1.172, 0.0),
Vec3(6.408, 2.863, 0.0),
Vec3(5.421, 4.179, 0.0),
]
cvNames = createControlPoints(cpPos, 0.3)

# Build and update the curve
# Run this chunc only if you want to update the curve
# You can move control points, change number of samples or a degree
def updateCurve(cvNames, numSamples = 120, degree = 3, pAtLength = 2.18):
    knots = createKnotVector(len(cvNames), degree)
    cvPos = getCpPositions(cvNames)
    pSq = sampleBSpline(numSamples, degree, cvPos, knots)
    diffSq = differentiate(pSq)
    lengthSq = lengthSequence(diffSq)
    samplePoints(pSq, 0.002, 3, "bSpline")
    # Interpolate at length and tangent at length
    posAtLen = interpolateAtLength(pSq, lengthSq, pAtLength)
    tangent = getTangentAtLength(diffSq, lengthSq, pAtLength)
    samplePoints([posAtLen], 0.1, 17, "pointAtLength")
    drawTangent(tangent, posAtLen)
    
updateCurve(cvNames, numSamples = 120, degree = 3, pAtLength = 2.18)
