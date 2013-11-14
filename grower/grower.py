#!/usr/bin/python
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
    # TODO: projection issues
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    dist2 = dlat * dlat + dlon * dlon
    dist = math.sqrt(dist2)
    return(dist)

  def eleDist(self, node1, node2):
    # TODO implement this
    return 0

  NORTH = math.pi/2
  SOUTH = -math.pi/2
  WEST = math.pi
  EAST = 0

  def getWeight(self, fromNode, toNode, theta, initWeight = 1):
    tau = self.getAngleFromCoords(self.data.nodes[fromNode][0],
                                  self.data.nodes[fromNode][1],
                                  self.data.nodes[toNode][0],
                                  self.data.nodes[toNode][1])
    ang = abs(theta - tau)
    while ang > math.pi: ang -= math.pi

    # weight from distance gain
    weight = initWeight*(2**(1-2/math.pi*ang))

    return weight

  def getAngleFromCoords(self, lat, lon, newlat, newlon):
    return math.atan((newlon - lon)/(newlat-lat))

  def growCycleAsLL(self, start, distanceGoal, elevationGoal,
                    initDirection, transport):

    result, cycle = self.growCycle(start, distanceGoal, elevationGoal,
                                  initDirection, transport)
    if ( result != 'success'):
      return(result,[])

    pos = []
    for node in cycle:
      lat, lon = self.data.nodes[node]
      pos.append((lat,lon))
    return result, pos

  ## this function behaves very similarly to router.doroute, except that
  ## it uses a different queue system and weighting algorithm.
  def growCycle(self, start, distanceGoal, elevationgoal, initdirection, transport):
    """generates the cycle"""

    cycle = [start]
    x = start
    dist = 0
    elegain = 0
    circledir = initdirection + math.pi
    theta = circledir

    def getTheta():
      lat = self.data.nodes[x][0]
      lon = self.data.nodes[x][1]

      def getIncrementedXOnIdealLoop(distIncrement):
        if distIncrement + dist > distanceGoal: return lat, lon
        else:
          return (distanceGoal / (2 * math.pi) *
                  (math.cos(2 * math.pi / distanceGoal * (dist + distIncrement)
                            + circledir) - math.cos(circledir)) + lat,
                  distanceGoal / (2 * math.pi) *
                  (math.sin(2 * math.pi / distanceGoal * (dist + distIncrement)
                            + circledir) - math.sin(circledir)) + lon)

      inc = 0
      minDiff = -1
      bestcoords = (lat, lon)
      while inc < distanceGoal - dist:
        latIdeal, lonIdeal = getIncrementedXOnIdealLoop(inc)
        dlat = self.coordDist(lat, lon, latIdeal, lonIdeal)
        diff = abs(dlat - inc)
        if diff < minDiff or minDiff < 0:
          mind = diff
          bestcoords = (latIdeal, lonIdeal)
        if diff < .01*dlat:
          return self.getAngleFromCoords(lat,lon,latIdeal,lonIdeal)

        inc += .01*distanceGoal

      if minDiff < .1*self.coordDist(lat, lon, bestcoords[0], bestcoords[1]):
        return self.getAngleFromCoords(lat,lon,bestcoords[0], bestcoords[1])
      else:
        return self.getAngleFromCoords(lat,lon,self.data.nodes[start][0],
                                       self.data.nodes[start][1])

    # self.data.routing[node].items() returns adjacent nodes for routing.

    while x != start or dist < 0.9*distanceGoal:
      # finds the best node to go to next
      maxweight = 0
      bestnode = x
      theta = getTheta()
      weights = []
      nodes = []
      weightsum = 0
      for node, initweight in self.data.routing[transport][start].items():
        weight = self.getWeight(x, node, theta, initweight)

        weightsum += weight
        nodes.append(node)
        weights.append(weight)

      rand = random.uniform(0,weightsum)
      i = 0
      while i < len(weights) and rand > weights[i]:
        rand -= weights[i]
        i += 1

      if i == len(weights): return ('no_node',[])
      else: bestnode = nodes[i]

      dist += self.distance(x,bestnode)
      elegain += self.eleDist(x,bestnode)
      cycle.append(bestnode)

    return('success',cycle)



if __name__ == "__main__":
  # do some testing

  data = LoadOsm("../pyroute/data/routing.osm")
  nodebeg = data.findNode(42.73,-73.678,'cycle')

  print( nodebeg)

  grower = Grower(data)

  dist = 1
  ele = 0

  result, cycle = grower.growCycle(nodebeg,dist,ele,grower.NORTH,'cycle')

  if result == 'success':
    print(cycle)

    for i in cycle:
      node = data.nodes[i]
      print("%d: %f,%f",i,node[0],node[1])
  else:
    print("Failed (%s)",result)
