import sys
sys.path.append('./pyroutelib2')
sys.path.append('../grower')
from route import Router
from grower import Grower

class RouteOrDirect(Router, Grower):
    def __init__(self, data):
        Router.__init__(self,data)
        Grower.__init__(self,data)
        self.route = {'valid':False}
        self.mode = 'direct';
    def setMode(self, mode):
      self.mode = mode
    def setStartNode(self,node):
        self.route['startnode'] = node
        self.route['startpos'] = self.data.nodes[node]
    def setEndNode(self,node):
        self.route['endnode'] = node
        self.route['endpos'] = self.data.nodes[node]
    def setStartLL(self,lat,lon,transport):
        self.route['startnode'] = self.data.findNode(lat,lon,transport)
        self.route['startpos'] = (lat,lon)
        self.route['transport'] = transport
    def setEndLL(self,lat,lon,transport):
        self.route['endnode'] = self.data.findNode(lat,lon,transport)
        self.route['endpos'] = (lat,lon)
        self.route['transport'] = transport
    def setDistEle(self, distGain, elevationGoal):
        self.route['distance'] = distGain
        self.route['ascent'] = elevationGoal
    def update(self):
        if(self.mode == 'route'):
            result, newroute = self.doRouteAsLL(self.route['startnode'],
                                                self.route['endnode'],
                                                self.route['transport'])
            if result == 'success':
                self.route['route'] = newroute
                self.route['valid'] = True
            else:
                print 'failed to route'
                self.route['route'] = [self.route['startpos'],self.route['endpos']]
                self.route['valid'] = True
        elif self.mode == 'direct':
            self.route['route'] = []
            self.route['route'].append(self.route['startpos'])
            self.route['route'].append(self.route['endpos'])
            self.route['valid'] = True
        elif self.mode == 'cycle':
            (self.route['valid'],
             self.route['route']) = self.growCycleAsLL(self.route['startnode'],
                                                       self.route['distance'],
                                                       self.route['ascent'],
                                                       0)
            print self.route['route']
    def valid(self):
        return(self.route['valid'])
