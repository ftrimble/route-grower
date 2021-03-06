#!/usr/bin/python2
#
################################################################################
#
# Cycles for cycling using OSM data and extending pyroutelib2.
#
################################################################################
#
# Copyright 2013, Forest Trimble
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
################################################################################

import sys
import math
import random

sys.path.append("../pyroute")
sys.path.append("../pyroute/pyroutelib2")
from loadOsm import *
from route import *

class Grower:
  def __init__(self, data):
    self.data = data

  def distance(self,n1,n2):
    """Calculate distance between two nodes"""
    lat1 = self.data.nodes[n1][0]
    lon1 = self.data.nodes[n1][1]
    lat2 = self.data.nodes[n2][0]
    lon2 = self.data.nodes[n2][1]
    return self.coordDist(lat1,lon1,lat2,lon2)

  def coordDist(self, lat1, lon1, lat2, lon2):
    rlat1 = math.radians(lat1)
    rlat2 = math.radians(lat2)
    rlon1 = math.radians(lon1)
    rlon2 = math.radians(lon2)

    dlat = rlat2 - rlat1
    dlon = rlon2 - rlon1

    h = math.sin(dlat/2)**2 + math.cos(rlat1)*math.cos(rlat2)*math.sin(dlon/2)**2
    dist = 2 * self.EARTH_RADIUS *math.asin(math.sqrt(h))
    return dist

  def eleDist(self, node1, node2):
    # TODO implement this
    return 0


  NORTH = math.pi/2
  SOUTH = -math.pi/2
  WEST = math.pi
  EAST = 0
  transport = 'cycle'
  EARTH_RADIUS = 3959 # miles

  def getWeight(self, fromNode, toNode, theta, initWeight = 1):
    """Calculates the weight of traveling to another node"""
    tau = self.getAngleFromCoords(self.data.nodes[fromNode][0],
                                  self.data.nodes[fromNode][1],
                                  self.data.nodes[toNode][0],
                                  self.data.nodes[toNode][1])
    ang = abs(theta - tau)
    while ang > math.pi: ang -= math.pi

    # weight from distance gain
    return initWeight*(2**(1-2/math.pi*ang))


  def getAngleFromCoords(self, lat, lon, newlat, newlon):
    """Gets the angle of the ray between two nodes"""
    # avoid divide by 0
    if newlat == lat:
      if newlon > lon: return 0
      return math.pi
    else: return math.atan((newlon - lon)/(newlat-lat))

  def growCycleAsLL(self, start, distanceGoal, elevationGoal, initDirection):
    """Grows the cycle as a list of lat/lon pairs"""
    result, cycle = self.growCycle(start, distanceGoal, elevationGoal,
                                  initDirection)
    if result != True: return(result,[])

    pos = []
    for node in cycle:
      lat, lon = self.data.nodes[node]
      pos.append((lat,lon))
    return result, pos

  def growCycle(self, start, distanceGoal, elevationgoal, initdirection):
    """Generates the Cycle"""

    router = Router(self.data)
    startLat = self.data.nodes[start][0]
    startLon = self.data.nodes[start][1]
    degDist = distanceGoal/69.0

    def getIncrementedXOnIdealLoop(distIncrement):
      """Gets the location on the ideal loop more distance is travelled"""
      # loop is getting too long; weight against the start
      if distIncrement + dist > distanceGoal: return startLat, startLon
      return (degDist / (2 * math.pi) *
              (math.cos(2 * math.pi / distanceGoal * (dist + distIncrement)
                        + circledir) - math.cos(circledir)) + startLat,
              degDist / (2 * math.pi) *
              (math.sin(2 * math.pi / distanceGoal * (dist + distIncrement)
                        + circledir) - math.sin(circledir)) + startLon)

    def brent(a, b, node):
      """Uses Brent's method to find the ideal x_j"""
      epsilon = .0001
      delta = .01


      lat1, lon1 = self.data.nodes[node][0], self.data.nodes[node][1]
      def f(increment):
        lat2, lon2 = getIncrementedXOnIdealLoop(increment)
        return self.coordDist(lat1, lon1, lat2, lon2) - increment

      def inverseQuadraticInterpolation(x1, x2, x3):
        if not f(x1) == f(x3) and not f(x2) == f(x3):
          return x1*f(x2)*f(x3)/((f(x1)-f(x2))*(f(x1)-f(x3))) + \
            x2*f(x1)*f(x3)/((f(x2)-f(x1))*(f(x2)-f(x3))) + \
            x3*f(x1)*f(x2)/((f(x3)-f(x1))*(f(x3)-f(x2)))
        else:
          return x2 - f(x2)*(x2-x1)/(f(x2)-f(x1))

      def needsBisection(left, prev2, prev1, prev, guess, bisectedLastTime):
        if guess > max(prev,(3*left + prev)/4) or \
           guess < min(prev,(3*left + prev)/4):
          return True
        elif bisectedLastTime:
          return abs(guess - prev) >= abs(prev - prev1)/2 \
            and abs(prev - prev1) < delta
        else:
          return abs(guess - prev) >= abs(prev1 - prev2)/2 \
            and abs(prev1 - prev2) < delta

      # require that we have valid bounds
      if f(a)*f(b) > 0: return None

      if abs(f(a)) < abs(f(b)): a,b = b,a
      prev = a
      prev1 = a
      prevBisect = True

      while not f(b) == 0 and abs(b - a) > epsilon:
        if abs(f(a)) < abs(f(b)): a, b = b, a

        guess = inverseQuadraticInterpolation(a, b, prev)
        prevBisect = needsBisection(a,prev1,prev,b,guess,prevBisect)

        if prevBisect: guess = (a + b)/2

        prev1 = prev
        prev = b
        if f(guess)*f(a) < 0: b = guess
        else: a = guess

      if abs(f(a)) < abs(f(b)): return a
      else: return b

    def getTheta():
      """Gets the ideal angle for the next node to come"""
      if prev == start: return initdirection

      increment = brent(0, distanceGoal - dist, prev)

      if not increment: return None
      else:
        latIdeal, lonIdeal = getIncrementedXOnIdealLoop(increment)
        return self.getAngleFromCoords(self.data.nodes[prev][0],
                                       self.data.nodes[prev][1],
                                       latIdeal, lonIdeal)

    cycle = []
    prev = start
    dist = 0
    circledir = initdirection + math.pi
    theta = circledir

    while prev != start or dist < 0.9*distanceGoal:
      # finds the best node to go to next
      theta = getTheta()

      # need to route back to start
      if theta == None:
        result, circuitCloser = router.doRoute(prev, start, self.transport)
        if result == 'success':
          cycle.extend(circuitCloser)
          return (True, cycle)
        else:
          return (False, [])

      info = []
      weightsum = 0
      try:
        # self.data.routing[transport][node].items() gets adjacent nodes.
        for node, initweight in self.data.routing[self.transport][prev].items():
          (res, nodes, segmentDist,
           last, weight) = self.getNextIntersection(prev,node,initweight,start)
          if res == True:
            weight = self.getWeight(prev, last, theta, weight/len(nodes))
            if node in cycle: weight /= 8
            weightsum += weight
            info.append((weight, nodes, segmentDist, last))
      except KeyError: pass

      if len(info) == 0:
        return (False,[])

      rand = random.gauss(weightsum/2,weightsum/3)
      i = 0
      while i < len(info) and rand > info[i][0]:
        rand -= info[i][0]
        i += 1

      if i >= len(info): infoItem = info[len(info)-1]
      else: infoItem = info[i]

      # add the list of nodes used
      cycle.extend(infoItem[1])
      dist += infoItem[2]
      prev = infoItem[3]

    return(True,cycle)


  def getNextIntersection(self, comingFrom, node, initWeight, start):
    prev = comingFrom
    nodeList = [node]
    dist = self.distance(comingFrom,node)
    weight = initWeight

    try:
      rnodes = self.data.routing[self.transport][node].items()

      while len(rnodes) != 0:
        numValid = 0
        indValid = 0
        for i in range(len(rnodes)):
          if (not rnodes[i][0] in nodeList) and rnodes[i][0] != comingFrom:
            numValid += 1
            indValid = i
        prev = node
        node = rnodes[indValid][0]
        weight += rnodes[indValid][1]
        nodeList.append(node)
        dist += self.distance(prev,node)

        rnodes = self.data.routing[self.transport][node].items()

        # Unroutable node; do not allow this direction
        if numValid == 0: return (False, [], 0, None, 0)
        # More than 2 possible nodes to take; we have arrived at an intersection
        elif numValid > 1: return(True, nodeList, dist, node, weight)

    # Unroutable node; do not allow this direction
    except KeyError: return (False, [], 0, None, 0)

    return False, [], 0, None, 0


if __name__ == "__main__":
  # do some testing

  print "Loading OSM Data..."
  data = LoadOsm("../pyroute/data/routing.osm")
  print "Finding a reasonable node..."
  nodebeg = data.findNode(42.728408,-73.672607,'cycle')
  grower = Grower(data)
  dist = 10 #miles
  ele = 0 #feet
  print "Growing a route from " , nodebeg
  res1, cycle1 = grower.growCycle(nodebeg,dist,ele,grower.NORTH)
  res2, cycle2 = grower.growCycleAsLL(nodebeg,dist,ele,grower.NORTH)

  if res1 == True:
    print(cycle1)

    for i in cycle1:
      node = data.nodes[i]
      print "%d: %f,%f" %(i,node[0],node[1])
  else: print "Could not grow a route normally"

  if res2 == True:
    print(cycle2)

  else: print "Could not grow a route as LL"
