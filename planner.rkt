#lang dssl2

let eight_principles = [  "Know your rights.",
                          "Acknowledge your sources.",
                          "Protect your work.",
                          "Avoid suspicion.",
                          "Do your own work.",
                          "Never falsify a record or permit another person to do so.",
                          "Never fabricate data, citations, or experimental results.",
                          "Always tell the truth when discussing your work with your instructor."]

#Final project: Trip Planner
#import math
                          
                          
import cons
import 'project-lib/dictionaries.rkt'
import 'project-lib/graph.rkt'
import sbox_hash



### Basic Types ###

#  - Latitudes and longitudes are numbers:
let Lat?  = num?
let Lon?  = num?

#  - Point-of-interest categories and names are strings:
let Cat?  = str?
let Name? = str?

### Raw Item Types ###

#  - Raw positions are 2-element vectors with a latitude and a longitude
let RawPos? = VecKC[Lat?, Lon?]

#  - Raw road segments are 4-element vectors with the latitude and
#    longitude of their first endpoint, then the latitude and longitude
#    of their second endpoint
let RawSeg? = VecKC[Lat?, Lon?, Lat?, Lon?]

#  - Raw points-of-interest are 4-element vectors with a latitude, a
#    longitude, a point-of-interest category, and a name
let RawPOI? = VecKC[Lat?, Lon?, Cat?, Name?]

### Contract Helpers ###

# ListC[T] is a list of `T`s (linear time):
let ListC = Cons.ListC
# List of unspecified element type (constant time):
let List? = Cons.list?


interface TRIP_PLANNER:

    # Returns the positions of all the points-of-interest that belong to
    # the given category.
    def locate_all(
            self,
            dst_cat:  Cat?           # point-of-interest category
        )   ->        ListC[RawPos?] # positions of the POIs

    # Returns the shortest route, if any, from the given source position
    # to the point-of-interest with the given name.
    def plan_route(
            self,
            src_lat:  Lat?,          # starting latitude
            src_lon:  Lon?,          # starting longitude
            dst_name: Name?          # name of goal
        )   ->        ListC[RawPos?] # path to goal

    # Finds no more than `n` points-of-interest of the given category
    # nearest to the source position.

    def find_nearby(
            self,
            src_lat:  Lat?,          # starting latitude
            src_lon:  Lon?,          # starting longitude
            dst_cat:  Cat?,          # point-of-interest category
            n:        nat?           # maximum number of results
        )   ->        ListC[RawPOI?] # list of nearby POIs
        
        
def len(vector):   # hepler func to calcualte the length
    let count = 0
    for i in vector:
        count = count + 1
    return count

class TripPlanner (TRIP_PLANNER):
    
    #let latitude: nat?
    #let longitude: nat?
    #let category: str?
    #let name: str
    #let positions:   ListC[RawPos?]   # how do I change this to just natural numbers
    #let POI:   VecC[Lat?, Lon?, Cat?, Name?]  # how do i try to describe array of three inputs
    let road_segements
    let POIs
    let node_id_to_position
    let position_to_node_id 
    let node_id_to_POI
    let graph
    
    def __init__(self, road_segements, POIs):
    
        self.road_segements = road_segements
        self.POIs = POIs
        self.node_id_to_position = HashTable(len(road_segements), make_sbox_hash())
        self.node_id_to_POI = HashTable(len(road_segements), make_sbox_hash())
        self.position_to_node_id =  HashTable(len(road_segements), make_sbox_hash())
        
        
        

        
        #initialize an empty has table
        let index = 0
        for i in road_segements:   # retriving and storing all the positions in Trip Planner
            let posn1x = i[0]
            let posn1y = i[1]
            let posn2x = i[2]
            let posn2y = i[3]
            
            let position_one = [posn1x, posn1y]
            let position_two = [posn2x, posn2y]
            
            
            if self.position_to_node_id.mem?(position_one) == False:          # handling duplicates
                    self.position_to_node_id.put(position_one, index)
                    self.node_id_to_position.put(index, position_one)
                    index = index + 1
        
            if self.node_id_to_position.mem?(position_two) == False:          # handling duplicates
                    self.position_to_node_id.put(position_two, index)
                    self.node_id_to_position.put(index, position_two)
                    index = index + 1
                
        self.graph = self.my_new_city_positions(index)
        
        # connection: WU graph, adj matrix    
        # now try to map postions to a node in graph to positions
            
    def length_of_road_segement(self, posn1, posn2):
        
            let   a = posn1[0]- posn2[0]
            let   b = posn2[1] - posn2[1]
            let   d = a*a + b*b
            return d.sqrt()
        
        
            
    def my_new_city_positions(self, index):
        # creating the graph
        
        let graph = WUGraph(index)
        #print(self.road_segements)
        for i in range(len(self.road_segements)):
            let x1 = self.road_segements[i][0]
            let y1 = self.road_segements[i][1]
            let x2 = self.road_segements[i][2]
            let y2 = self.road_segements[i][3]
            
             
            let position_one = [x1, y1]
            let position_two = [x2, y2]
            
            
            let node1 = self.position_to_node_id.get(position_one)
            let node2 = self.position_to_node_id.get (position_two)
            
            graph.set_edge(node1, node2, self.length_of_road_segement(self.node_id_to_position.get(1),
                                                                            self.node_id_to_position.get(2)))
        return
            
        
     
        
          
        
    def locate_all(self,dst_cat):
        
        #Takes a point-of-interest category; returns the positions of all points
        #of-interest in the given category. The positions can returned be in any
        #order you want, but the result should not include duplicates.
        
        let result = None
        for poi in self.POIs:
            #print("here")
            if poi[2] == dst_cat:
                #print('here')
                result = cons([poi[0],poi[1]], result)
                
        return result
        
        
        
        
    def plan_route(self, src_lat, src_lon, dst_name):
       # Takes a starting position (latitude and longitude) and the name of
       # a point-of-interest; returns a shortest path from the starting position to
       # the named point-of-interest. You can assume the starting position is at a
       # road segment endpoint. Returns the empty list if the destination does not
       # exist or is unreachable

        if not self.pois_name.mem?(dst_name):
            return None
            
        let dest_poi = self.pois_name.get(dst_name)
        let dest_pos = [dest_poi[0],dest_poi[1]]
        let dest_vertex = self.position_id_map.get(dest_pos)

        let start = self.position_id_map.get([src_lat,src_lon])

        if start == dest_vertex:
            return cons([src_lat, src_lon], None)
        let dijkstras = self.dijkstra([src_lat,src_lon])
        
        if dijkstras.pred[dest_vertex] is None:
            return None
        
        let result = cons(dest_pos, None)
        
        let head = dijkstras.pred[dest_vertex]
        
        while head is not start:
            if dijkstras.pred[head] is None:
                return None
            result = cons(self.id_position_map.get(head),result)
            head = dijkstras.pred[head]
            
        result = cons([src_lat,src_lon],result)
        
        return result


    def dijkstra(self, start):
        let distances = [inf for i in range(self.position_id_map.len())]
        let pred = [None for i in range(self.position_id_map.len())]
        distances[self.position_id_map.get(start)] = 0
        let tovisit = BinHeap(self.road_seg.len(), lambda x,y: x.dist < y.dist)
        let visited = [False for i in range(self.position_id_map.len())]
        
        tovisit.insert(pq(self.position_id_map.get(start),0))
        
        while tovisit.len() is not 0:
            let curr = tovisit.find_min()
            tovisit.remove_min()
            if visited[curr.vertex] is False:
                visited[curr.vertex] = True
                let neighbors = Cons.to_vec(self.graph.get_adjacent(curr.vertex))
                for neighbor in neighbors:
                    if distances[curr.vertex] + self.graph.get_edge(curr.vertex,neighbor) < distances[neighbor]:
                        distances[neighbor] = distances[curr.vertex] + self.graph.get_edge(curr.vertex,neighbor)
                        pred[neighbor] = curr.vertex
                        tovisit.insert(pq(neighbor,distances[neighbor]))
                    
        let result = dijkstra(distances,pred)
        return result      

        
        
        
       
        
    def find_nearby(self, src_lat, src_lon, dst_cat, n):
       # Takes a starting position (latitude and longitude), a point-of
       # interest category, and a limit n; returns the (up to) n points-of-interest
       # in the given category nearest the starting position. You can assume the
       # starting position is at a road segment endpoint. Resolve ties however you
       # want, and order points-of-interest within the list in any order you want
        let dijkstras = self.dijkstra([src_lat, src_lon])       
         let pois_nearby = BinHeap(self.road_seg.len(), lambda x,y: x[0] < y[0])        
         for poi in self.pois:
             if poi[2] == dst_cat:
                 let poi_pos = [poi[0], poi[1]]
                 if self.position_id_map.mem?(poi_pos):
                     let poi_vertex = self.position_id_map.get(poi_pos)
                     
                     let distance = dijkstras.dist[poi_vertex]
                     if dijkstras.dist[poi_vertex] != inf:
                         pois_nearby.insert([distance, poi])
                
         let nearby_pois = None
         let count = 0
         while pois_nearby.len() > 0 and count < n:
            let removed = pois_nearby.find_min()[1]
            pois_nearby.remove_min()   
            nearby_pois = cons(removed, nearby_pois)
            count = count + 1
             
         return nearby_pois    
     
        
    
        
        
    
   


def my_first_example():
    return TripPlanner([[0,0, 0,1], [0,0, 1,0]],
                       [[0,0, "bar", "Reggie's"],
                        [0,1, "food", "Dumplings"]])

test 'My first locate_all test':
    assert my_first_example().locate_all("food") == \
        cons([0,1], None)

test 'My first plan_route test':
   assert my_first_example().plan_route(0, 0, "Dumplings") == \
       cons([0,0], cons([0,1], None))

test 'My first find_nearby test':
   assert my_first_example().find_nearby(0, 0, "food", 1) == \
        cons([0,1, "food", "Dumplings"], None)
