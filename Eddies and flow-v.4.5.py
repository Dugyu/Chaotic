#=================================#
#============Kanagawa==============#
#=======For Tokyo Studio=====#
#==========Written by Dugy========#
#===========Apr 25th 2017==========#
#==Do not use without permission==#
#=================================#
from math import *
import random
import Rhino.Geometry as rg
import Rhino.Geometry.Vector3d as PVector
import Rhino.Geometry.Point3d as Point 
import Rhino.Geometry.Curve as Curve
import Rhino.Geometry.Line as Line
import rhinoscriptsyntax as rs
import scriptcontext as sc
import ghpythonlib.components as ghc

#=======================#
#For List to Datatree
#=======================#


import System.Collections.Generic as SCG
from Grasshopper import DataTree
from Grasshopper.Kernel.Data import GH_Path

def AddToPath(l,dTree,pathRoot=''):
	if type(l)==list:
		for i in xrange(len(l)):
			if pathRoot !='':
				AddToPath(l[i],dTree,pathRoot+','+str(i))
			else:
				AddToPath(l[i],dTree,str(i))
	else:
		exec('path = GH_Path('+pathRoot+')')
		dTree.Add(l,path)
#=======================#
#Random Seed
#=======================#
random.seed(18)

#=======================#
#Basic Functions
#=======================#
def toplimit(vct,speed):
	if vct.Length <= speed:
		pass
	else:
		vct.Unitize()
		vct = PVector.Multiply(vct,speed)
	return vct

def lowerlimit(vct,speed):
	if vct.Length >= speed:
		pass
	else:
		vct.Unitize()
		vct = PVector.Multiply(vct,speed)
	return vct


#=================================#
#Single Classes
#=================================#


class Eddy_config():
	"""config for Class Eddy """
	def __init__(self, loc_, gravity_, oricharge_, affrange_, id_):
		self.id = id_
		self.gravity = gravity_
		self.oricharge = oricharge_          		# orient angle:  0 attract, 1 reject (0-pi)
		self.buffer = affrange_
		self.loc = loc_
class Eddy():
	"""provide gravity"""
	rangedemp = 0.5
	fieldcharge = 0.25
	boundaryZ = 30.0
	def __init__(self,config):
		self.config = config
		self.id = config.id 
		self.gravity = config.gravity
		self.oricharge = config.oricharge    		# 0 attract   1  reject
		self.loc = config.loc
		self.buffer = config.buffer    				#  buffer area
		self.core = self.rangedemp * self.buffer   	#core area


class Flow_config(object):
	"""config for Class Flow """
	def __init__(self, loc_, vel_, acc_ ):
		self.loc = loc_
		self.vel = vel_
		self.acc = acc_
class Flow(object):
	"""vector field"""
	toplimit = 2.0
	lowerlimit = 2.0
	boundaryZ = 15.0

	def __init__(self, config):

		self.config = config
		self.loc = config.loc
		self.vel = config.vel
		self.acc = config.acc
		self.ptlist = [Point(self.loc)]
		self.trail = None
		self.target = PVector(0,0,0)
		self.orient = PVector(0,0,0)
		
		self.lolist = [self.loc]   					#track location 
		self.velist = [self.vel]					#track velocity
		self.aclist = [self.acc]					#track accleration
		
		self.id = None								#waiting for 
		self.mood = None							#the group id this flow belongs to
		self.retrail = None							#simplified trailcrv
		self.reptlist = []
	def inradius(self,eddy):
		dist = PVector.Subtract(self.loc,eddy.loc)
		dist2d = PVector(dist.X,dist.Y,0).Length
		if dist2d >= eddy.buffer: 
			return 0
		elif dist2d <= eddy.core:
			return -1
		else:
			return dist2d
	def getoricharge(self,eddies):
		ori = 0
		orisum = []
		for eddy in eddies:
			sts = self.inradius(eddy)
			if sts == 0:
				pass
			else:
				if sts == -1:
					ori = eddy.oricharge
				else:
					ori = (sts -eddy.core)/(eddy.buffer-eddy.core)*(eddy.oricharge-eddy.fieldcharge)+eddy.fieldcharge
				orisum.append(ori)
		if len(orisum) == 0:
			return Eddy.fieldcharge
		else:
			return sum(orisum)/len(orisum)
	def gettarget_2d(self,eddies):
		target_2d  = PVector(0,0,0)
		for i in xrange(len(eddies)):
			v = PVector.Subtract(eddies[i].loc,self.loc)
			v2d = PVector(v.X,v.Y,0)  			# 2d planer target, X Y are the most important factors
			dist2d = v2d.Length
			# compute the sum of the arraction vector / distance
			v2d.Unitize()
			v2d = PVector.Multiply(v2d, float(eddies[i].gravity/pow(dist2d,1.0)))
			# add vector to attraction vector collection
			target_2d = PVector.Add(target_2d,v2d)
		#Limit the target length
		target_2d = toplimit(target_2d,self.toplimit)   
		target_2d = lowerlimit(target_2d,self.lowerlimit)
		self.target = target_2d
	def getorient(self,eddies):
		orient_2d = self.target 
		angle = self.getoricharge(eddies)*pi
		orient_2d.Rotate(angle, PVector(0,0,1))
		self.orient = orient_2d 

	def getacc_z(self):
		acc_z = self.target.Length*0.05
		self.acc = PVector(0,0,acc_z)
		self.acc = toplimit(self.acc,toplimit)
	#def boundarycheck(self):
		#if self.loc.Z > self.boundaryZ:
			#self.acc = PVector(0,0,-abs(self.acc.Z))
		#elif self.loc.Z < -0.2*self.boundaryZ:
			#self.acc = PVector(0,0,abs(self.acc.Z))
		#else:
			#pass
	def boundarycheck(self):
		if self.loc.Z > self.boundaryZ or self.loc.Z < -0.2*self.boundaryZ:
			self.acc = -self.acc
		else:
			pass
	def warp(self,crv):
		if rs.PointInPlanarClosedCurve(self.loc,crv) == 0:
			self.vel = PVector(-self.vel.X,-self.vel.Y,self.vel.Z)
	def getvel(self):
		self.vel = PVector.Add(self.vel,self.orient)
		self.vel = toplimit(self.vel,self.toplimit)
		self.vel = PVector.Add(self.vel,self.acc)
	def update(self,eddies,crv):
		self.gettarget_2d(eddies)
		self.getorient(eddies) 
		self.getacc_z()
		self.boundarycheck()
		self.getvel()
		self.loc = PVector.Add(self.loc,self.vel) 
		#self.lolist.append(self.loc)
		#self.velist.append(self.vel)
		#self.aclist.append(self.acc)
	def drawtrial(self):
		self.ptlist.append(Point(self.loc))
	def drawsimple(self):
		self.trail = Curve.CreateInterpolatedCurve(self.ptlist,3)
		self.retrail = Curve.Rebuild(self.trail,30,3,True)
		self.retrail.Domain = rg.Interval(0,1)
		crvts = Curve.DivideByCount(self.retrail,29,True)
		self.reptlist = []
		for i in xrange(len(crvts)):
			self.reptlist.append(Curve.PointAt(self.retrail,crvts[i])) 
	def run(self,eddies,crv):
		self.update(eddies,crv)
		self.drawtrial()



#=================================#
#Parameters
#=================================#

n_flow = 800


#=================================#
#Collections
#=================================#
eddyconfig = []
flowconfig = []

eddycollection = []
flowcollection = []

#input eddies
for i in xrange(len(eddyinput)):
	loc = PVector(eddyinput[i].X,eddyinput[i].Y,eddyinput[i].Z)
	gravity = gravityinput[i]
	oricharge = orichargeinput[i]
	affrange = affrangeinput[i]

	eddyconfig.append(Eddy_config(loc,gravity,oricharge,affrange,i))
	eddycollection.append(Eddy(eddyconfig[i]))

#generate flow particles
crv = sc.doc.Objects.AddCurve(sitecrv)
j = -1
for i in xrange(n_flow):
	u = random.uniform(0.0,1.0)
	v = random.uniform(0.0,1.0)
	a = sitesrf.PointAt(u,v)
	vel = PVector(random.uniform(0.0,1.0),random.uniform(0.0,1.0),0)
	particle = PVector(a.X,a.Y,a.Z)
	if rs.PointInPlanarClosedCurve(a,crv) == 0:
		pass
	else:
		flowconfig.append(Flow_config(particle,PVector(0,0,0),PVector(0,0,0)))

for config in flowconfig:
	flowcollection.append(Flow(config))



#=================================#
#Draw
#=================================#
for i in xrange(100):
	for particle in flowcollection:
		particle.run(eddycollection,crv)
for particle in flowcollection:
	particle.drawsimple()

#=================================#
#Storage Output
#=================================#

fieldcollection = []
pointcollection_list = []
for particle in flowcollection:
	fieldcollection.append(particle.retrail)
	pointcollection_list.append(particle.reptlist)
pointcollection =  DataTree[object]()
AddToPath(pointcollection_list,pointcollection)



#=================================#
#Group Classes
#=================================#

class Silk(object):
	"""shortlines"""
	def __init__(self,flow1,flow2):
		self.flow = [flow1,flow2]
		self.silks = []
	def drawsilk(self):
		t1 = Curve.DivideByCount(self.flow[0].retrail,99,True)
		t2 = Curve.DivideByCount(self.flow[1].retrail,99,True)
		for i in xrange(99):
			pt1 = Curve.PointAt(self.flow[0].retrail,t1[i])
			pt2 = Curve.PointAt(self.flow[1].retrail,t2[i])
			self.silks.append(Line(pt1,pt2))

class Wave(object):
	"""contains a group of similar flow objects"""
	def __init__(self, mood, flow):
		self.mood = mood
		self.basecrv = flow.retrail
		self.crvlist = [self.basecrv]
		self.srflist = []
		self.flow = [flow]
		self.silkbranch = []
def moodbelong(flow,wave):
	devi = rg.Curve.GetDistancesBetweenCurves(flow.retrail,wave.basecrv,0.0001)
	if devi[0] == False:
		return False
	elif devi[1] >15.0 or devi[4]> 7.0:
		return False
	else:
		return True



wavecollection = []

for i in xrange(len(flowcollection)):
	flowcollection[i].id = i
	if i == 0 :
		wavecollection.append(Wave(0,flowcollection[i]))
		flowcollection[i].mood = 0

	else:
		for j in xrange(len(wavecollection)):
			condi = moodbelong(flowcollection[i],wavecollection[j])
			if condi == True:
				wavecollection[j].crvlist.append(flowcollection[i].retrail)
				wavecollection[j].flow.append(flowcollection[i])
				flowcollection[i].mood = j
				break
			else:
				pass
		if flowcollection[i].mood == None :
			wavecollection.append(Wave(len(wavecollection)+1,flowcollection[i]))
			flowcollection[i].mood = j
		else:
			pass

waveflow = []
wavesrf = []
trailsolid = []
silkcollection = []
silklines = []

sidecrv = []    # the side crvs beside tween crvs 
sidesim = []

for wave in wavecollection:
	if len(wave.crvlist) == 1:
		trailsolid.append(wave.basecrv)
	elif len(wave.crvlist) == 2:
		wavesrf.append(list(Curve.CreateTweenCurves(wave.crvlist[0],wave.crvlist[1],7)))
		sidecrv.append([wave.crvlist[0],wavesrf[-1][0],wavesrf[-1][-1],wave.crvlist[1]])
	elif len(wave.crvlist) >= 3:
		num1 = random.choice(range(len(wave.crvlist)))
		num2 = random.choice(range(len(wave.crvlist)))
		while num1 == num2:
			num2 = random.choice(range(len(wave.crvlist)))
		num3 = random.choice(range(len(wave.crvlist)))
 		while num2 == num3 or num1 == num3:
			num3 = random.choice(range(len(wave.crvlist))) 			
		crv1 = wave.crvlist[num1]
		crv2 = wave.crvlist[num2]
		crv3 = wave.crvlist[num3]
		wavesrf.append(list(Curve.CreateTweenCurves(crv1,crv2,7)))
		sidecrv.append([crv1,wavesrf[-1][0],wavesrf[-1][-1],crv2])
		wavesrf.append(list(Curve.CreateTweenCurves(crv2,crv3,7)))
		sidecrv.append([crv2,wavesrf[-1][0],wavesrf[-1][-1],crv3])
		if len(wave.crvlist) >= 5:
			wave.silkbranch = range(len(wave.crvlist))
			wave.silkbranch.remove(num1)
			wave.silkbranch.remove(num2)
			wave.silkbranch.remove(num3)
			
			n1 = random.choice(wave.silkbranch)
			n2 = random.choice(wave.silkbranch)

			while n1 == n2:
				n2 = random.choice(wave.silkbranch)

			silkcollection.append(Silk(wave.flow[n1],wave.flow[n2]))
			silkcollection[-1].drawsilk()
			silklines.append(silkcollection[-1].silks)
			trailsolid.append(wave.flow[n1].retrail)
			trailsolid.append(wave.flow[n2].retrail)
	waveflow.append(wave.crvlist)


def drawside(crv,sim):
	for i in xrange(len(crv)):
		t = [[],[],[],[]]
		for j in xrange(len(crv[i])):
			t[j] = Curve.DivideByCount(crv[i][j],29,True)
		for k in xrange(len(t[j])):
			pt0 = Curve.PointAt(crv[i][0],t[0][k])
			pt1 = Curve.PointAt(crv[i][1],t[1][k])
			pt2 = Curve.PointAt(crv[i][2],t[2][k])
			pt3 = Curve.PointAt(crv[i][3],t[3][k])
			sim.append(Line(pt0,pt1)) 
			sim.append(Line(pt2,pt3)) 
	return sim

sidesim = drawside(sidecrv,sidesim)

flowgroup = DataTree[object]()
wavesrfline = DataTree[object]()
trailsolid_1 = DataTree[object]()
silklinessolid = DataTree[object]()
sidesimsolid =  DataTree[object]()
sidecrv_1 = DataTree[object]()

AddToPath(waveflow,flowgroup)
AddToPath(wavesrf,wavesrfline)
AddToPath(trailsolid,trailsolid_1)
AddToPath(silklines,silklinessolid)
AddToPath(sidecrv,sidecrv_1)
AddToPath(sidesim,sidesimsolid)

srfnum = len(wavesrf)