#lang dssl2

# Final project: Trip Planner

import cons
import 'project-lib/dictionaries.rkt'
import sbox_hash
import 'project-lib/graph.rkt'
import 'project-lib/binheap.rkt'

### Basic Types ###

#  - Latitudes and longitudes are numbers:
let Lat?  = num?
let Lon?  = num?

#  - Point-of-interest categories and names are strings:
let Cat?  = str?
let Name? = str?

### Raw Item Types ###

#  - Raw positions are 2-element vectors with a latitude and a longitude
let RawPos? = TupC[Lat?, Lon?]

#  - Raw road segments are 4-element vectors with the latitude and
#    longitude of their first endpoint, then the latitude and longitude
#    of their second endpoint
let RawSeg? = TupC[Lat?, Lon?, Lat?, Lon?]

#  - Raw points-of-interest are 4-element vectors with a latitude, a
#    longitude, a point-of-interest category, and a name
let RawPOI? = TupC[Lat?, Lon?, Cat?, Name?]

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


# implementing the items using structs        
struct position:
    let lat: Lat?
    let long: Lon?
    
struct road_seg:
    let point1_lat: Lat?
    let point1_lon: Lon?
    let point2_lat: Lat?
    let point2_lon: Lon?

struct poi:
    let lat: Lat?
    let lon: Lon?
    let cat: Cat?
    let name: Name?         

# helper struct
struct dist_id:
    let dist: num?
    let id: num? 
    
# helper functions 
def distance(x1, y1, x2, y2):
    let dist = ((x1 - x2)*(x1 - x2) +(y1 -y2)*(y1 -y2)).sqrt()
    return dist             
        
def dijkstra(graph, start):    
    let dist = [inf; graph.len()] #make everything infinity
    let pred = [None; graph.len()]
    let visited = [False; graph.len()]
    
    dist[start] =0
    
    let todo= BinHeap(2*graph.len(), λ x, y: dist[x]< dist[y])
        
    todo.insert(start)
    
    # while left is not empty
    while todo.len()!=0:

       # pick the nearest vertex
       let node1= todo.find_min()
       #println(node1)
       todo.remove_min()
       
       if visited[node1]:
           continue 
       
       visited[node1]=True
       
       let node2= graph.get_adjacent(node1) 
       #println(node2)
       while node2 is not None:
           let originalDist= dist[node2.data]
           let weight= graph.get_edge(node1, node2.data)
           let newDistance= dist[node1]+weight 
           #println(newDistance)
           if newDistance < originalDist:
               dist[node2.data]= newDistance
               pred[node2.data]= node1
               # node2 now also needs to be visited
               todo.insert(node2.data)
           node2= node2.next  
    return[pred,dist] 
    
    
    
class TripPlanner (TRIP_PLANNER):
    let roads
    let pois
    let pos_to_id
    let id_to_pos
    let id_to_pois
    let name_to_pos
    let graph
    
    def __init__(self, roads, pois):
       
        self.roads = roads
        self.pois = pois
        
    # create a dictionary to map road positions (lat and long) to numbers to create a Wugraph         
        
        self.pos_to_id= HashTable(len(roads) * 2, make_sbox_hash())
        self.id_to_pos= HashTable(len(roads) * 2, make_sbox_hash())
           
        let cnt=0
        
        for road in roads:
           
            let point1= position(road[0],road[1])
            let point2= position(road[2], road[3])
            
            # if the road position is not in the dict, add it to dictionary and the unique id is cnt
            if not self.pos_to_id.mem?(point1):
                self.pos_to_id.put(point1,cnt)
                self.id_to_pos.put(cnt, point1)
                cnt= cnt+1 
                
            if not self.pos_to_id.mem?(point2):
                self.pos_to_id.put(point2,cnt)
                self.id_to_pos.put(cnt, point2)
                cnt=cnt+1
                

    # use separate chaining hash table as multiple pois can be in one location (x)
                # create a dictionary that maps pos to poi_struct (x)
    # use linked list vector where each id is the key and the value are the pois in the position 
            
        self.id_to_pois= vec(self.pos_to_id.len()*2)
        self.name_to_pos= HashTable(len(roads) * 2, make_sbox_hash())
       
        for i in pois:
            let pos= position(i[0],i[1])
            let poi_struct= poi(i[0],i[1],i[2],i[3])
            
            # check if there's an id to the position given 
            if not self.pos_to_id.mem?(pos):
                self.pos_to_id.put(pos,cnt)
                self.id_to_pos.put(cnt,pos)
                cnt=cnt+1
            
            let id = self.pos_to_id.get(pos)
               
            self.id_to_pois[id]= cons(i, self.id_to_pois[id])
            
            #println(id, self.id_to_pois)
            

   
            # for plan_route
            self.name_to_pos.put(i[3],pos)
            
    # Create a Wugraph
        
        self.graph= WuGraph(self.pos_to_id.len())    
         
        for road in roads:
            let weight= distance(road[0],road[1],road[2],road[3])
            let id1= self.pos_to_id.get(position(road[0],road[1]))
            let id2= self.pos_to_id.get(position(road[2],road[3]))
            self.graph.set_edge(id1,id2,weight)       
                     
        #println(self.id_to_pos)    
            
    def locate_all(self,poi_cat:Cat?):
        
        let duplicates= HashTable(self.pois.len()*2, make_sbox_hash())
        let located_points= None
        for i in self.pois:
            if i[2] == poi_cat and not duplicates.mem?(position(i[0],i[1])):
                located_points= cons([i[0],i[1]], located_points)
                duplicates.put(position(i[0],i[1]),"exists")
                
        return located_points 
        
    def plan_route(self, lat_start: Lat?, long_start: Lon?, poi_name: Name?) -> ListC[RawPos?]:
        # if the poi_name is not there, return None 
        if not self.name_to_pos.mem?(poi_name):
            return None 
        
        # if the starting position is not there, return None 
        if not self.pos_to_id.mem?(position(lat_start,long_start)):
            return None 
            
        # get the id to the starting position
        let start= self.pos_to_id.get(position(lat_start,long_start))
        # get the pos from the name 
        let end_pos= self.name_to_pos.get(poi_name)
        let end= self.pos_to_id.get(end_pos)
        
        
        # if start is end, just return the position
        if start==end:
            return cons([lat_start, long_start],None)
        
        let dj= dijkstra(self.graph, start)
        
        # inf means distance not found, so return None
        if dj[1]==inf:
            return None 

        let preds= dj[0]
        #println(preds)
        
        # preds of the poi
        let poi_preds= preds[end]
        
        let path= None
        
        # if poi has no preds 
        if start is not end and poi_preds == None:
            return None
        
        # until the preds of end reaches the start 
        while end is not start and end is not None:
            let pos= self.id_to_pos.get(end)
            path=cons([pos.lat,pos.long], path)
            end= preds[end]
           
            # when the preds of end reached the start, add the path to the start 
            if end== start:
                path= cons([lat_start, long_start],path)
                
        return path 
            
            
    def find_nearby(self, lat_start: Lat?, long_start: Lon?, poi_cat: Cat?, n: nat?) -> ListC[RawPOI?]:
        
        if not self.pos_to_id.mem?(position(lat_start, long_start)):
            return None 
        
        let start= self.pos_to_id.get(position(lat_start, long_start))
        let pos_in_category= self.locate_all(poi_cat)
        # println(pos_in_category)
        
        if pos_in_category == None:
            return None
            
        let dj= dijkstra(self.graph, start)
        #println(dj)
        let dist= dj[1] 
        
        #println(dist)
       
        
#        let dist_to_id= HashTable(dist.len() * 2, make_sbox_hash())
#        let id_to_dist= HashTable(dist.len() * 2, make_sbox_hash())
        
#        for i in range(dist.len()):
#            if dist_to_id.mem?(dist[i]):
#                continue
            
#            dist_to_id.put(dist[i],i)
#            id_to_dist.put(i, dist[i])
            
        let dist_to_id=vec(dist.len())
        let cnt=0
        for i in range(dist.len()):           
            dist_to_id[i]=dist_id(dist[i],i)
            cnt=cnt+1
        #println(dist_to_id)
        
        heap_sort(dist_to_id, λ x, y: x.dist < y.dist)
       
        let results= None
        
        cnt=0
        for i in dist_to_id:
            let id= i.id
            #println(id)
            if id==None or i.dist==inf:
                continue
            let poi_struct= self.id_to_pois[id]
            #println(poi_struct)
            
#            while not pos_in_category==None:
#                let id_cat= self.pos_to_id.get(position(pos_in_category.data[0], pos_in_category.data[1]))
#                if id==id_cat:
#                    results= cons([poi_struct.data[0], poi_struct.data[1], poi_struct.data[2], poi_struct.data[3]],results)
#                    cnt=cnt+1
#                pos_in_category= pos_in_category.next  
            
            
            while True:
                if poi_struct == None or cnt==n:
                    break 
                # println(poi_struct.data[2])
                if poi_struct.data[2] == poi_cat:
                    results= cons([poi_struct.data[0], poi_struct.data[1], poi_struct.data[2], poi_struct.data[3]],results)
                    cnt=cnt+1
                poi_struct= poi_struct.next 
        
        return results
          
def my_first_example():
    return TripPlanner([[0,0, 0,1], [0,0, 1,0]],
                       [[0,0, "bar", "The Empty Bottle"],
                        [0,1, "food", "Pierogi"]])
                        
                        
# 0,0 (bar) — 0,1 (food)
#  l
#  l
# 1,0 

test 'My first locate_all test':
    assert my_first_example().locate_all("food") == \
        cons([0,1], None)

test 'My first plan_route test':
   assert my_first_example().plan_route(0, 0, "Pierogi") == \
       cons([0,0], cons([0,1], None))

test 'My first find_nearby test':
    assert my_first_example().find_nearby(0, 0, "food", 1) == \
        cons([0,1, "food", "Pierogi"], None)

def planner_example():
    return TripPlanner([[0,0,1,0], [0,0,0,1], [0,1,1,1], [1,1, 1,2], [0,1,0,2], [1,0, 1,1], [0,2, 1,2], [1,2,1,3], [1,3,-0.2,3.3]],
                                [[0,0,"food", "Sandwiches"], [0,1, "food", "Pasta"], [0,1,"clothes","Pants"],\
                                [1,1, "bank", "Local Credit Union"], [1,3, "bar", "Bar None"],\
                                [1,3, "bar", "H Bar"],[-0.2, 3.3, "food", "Burritos"]])   
                                
test 'planner_example_test- locate_all':
    assert planner_example().locate_all("food") == cons([-0.2,3.3],cons([0,1],cons([0,0], None)))
    assert planner_example().locate_all("barber") == None
    
test 'planner_example_test- plan_route':
    assert planner_example().plan_route(0,0,"Sandwiches") == cons([0,0], None)
    assert planner_example().plan_route(1,1,"Sandwiches") == cons([1,1], cons([1,0],cons([0,0],None)))
    assert planner_example().plan_route(1,1,"Sushi") == None 
    
test 'planner_example_test- find_nearby':    
    assert planner_example().find_nearby(1,3,"food",1)== cons([-0.2, 3.3, "food", "Burritos"], None)
    assert planner_example().find_nearby(0,2,"food",2)== cons([0,0, "food", "Sandwiches"], cons([0,1, "food", "Pasta"], None))
    assert planner_example().find_nearby(0,2,"bar",1)== cons([1,3, "bar", "H Bar"], None)
    assert planner_example().find_nearby(0,2,"school",5)== None 
    
test 'real_test_1':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0],
       [3, 0, 4, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [5, 0, 'barber', 'Judy'],
       [5, 0, 'barber', 'Lily']])    
    let result = tp.locate_all('barber')
    assert (Cons.to_vec(result)) == [[5, 0], [3, 0]]
    
test 'real_test_2':
    let tp = TripPlanner(
      [[0, 0, 0, 9],
       [0, 9, 9, 9],
       [0, 0, 1, 1],
       [1, 1, 2, 2],
       [2, 2, 3, 3],
       [3, 3, 4, 4],
       [4, 4, 5, 5],
       [5, 5, 6, 6],
       [6, 6, 7, 7],
       [7, 7, 8, 8],
       [8, 8, 9, 9]],
      [[7, 7, 'haberdasher', 'Archit'],
       [8, 8, 'haberdasher', 'Braden'],
       [9, 9, 'haberdasher', 'Cem']])    
    let result = tp.plan_route(0, 0, 'Cem')
    assert Cons.to_vec(result) \
      == [[0, 0], [1, 1], [2, 2], [3, 3], [4, 4], [5, 5], [6, 6], [7, 7], [8, 8], [9, 9]]

test 'real_test_3':
    let tp = TripPlanner(
      [[-2, 0, 0, 2],
       [0, 2, 2, 0],
       [2, 0, 0, -2],
       [0, -2, -2, 0]],
      [[2, 0, 'cooper', 'Dennis']])
    let result = tp.plan_route(-2, 0, 'Dennis')
    assert Cons.to_vec(result) == [[-2, 0], [0, -2], [2, 0]]
    
test 'real_test_4':
    let tp = TripPlanner(
      [[0, 0, 0, 1],
       [0, 1, 3, 0],
       [0, 1, 4, 0],
       [0, 1, 5, 0],
       [0, 1, 6, 0],
       [0, 0, 1, 1],
       [1, 1, 3, 0],
       [1, 1, 4, 0],
       [1, 1, 5, 0],
       [1, 1, 6, 0],
       [0, 0, 2, 1],
       [2, 1, 3, 0],
       [2, 1, 4, 0],
       [2, 1, 5, 0],
       [2, 1, 6, 0]],
      [[0, 0, 'blacksmith', "Revere's Silver Shop"],
       [6, 0, 'church', 'Old North Church']])
    let result = tp.plan_route(0, 0, 'Old North Church')
    assert Cons.to_vec(result) == [[0, 0], [2, 1], [6, 0]]

test 'real_test_5':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [4, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy']])
    let result = tp.find_nearby(0, 0, 'barber', 2)
    assert Cons.to_vec(result) == [[3, 0, 'barber', 'Tony']]   
    
    
test 'real_test_6':
    let tp = TripPlanner(
      [[0, 0, 1.5, 0],
       [1.5, 0, 2.5, 0],
       [2.5, 0, 3, 0],
       [4, 0, 5, 0]],
      [[1.5, 0, 'bank', 'Union'],
       [3, 0, 'barber', 'Tony'],
       [4, 0, 'food', 'Jollibee'],
       [5, 0, 'barber', 'Judy']])
    let result = tp.find_nearby(0, 0, 'food', 1)
    assert Cons.to_vec(result) == []    
         
let eight_principles = ["Know your rights.",
"Acknowledge your sources.",
"Protect your work.",
"Avoid suspicion.",
"Do your own work.",
"Never falsify a record or permit another person to do so.",
"Never fabricate data, citations, or experimental results.",
"Always tell the truth when discussing your work with your instructor."]