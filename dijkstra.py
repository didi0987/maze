import math
from graph import Graph, Node

''' Vertex extends the class Node and represents each vertex in the graph'''
class Vertex(Node):
    def __init__(self, value, neighbors=None):
        super().__init__(value, neighbors)
        self.length_from_start = math.inf
        self.previous_node = None
        self.visited = False
    

    ''' Return the distance from a given neighbor'''
    def distance_from_neighbor(self, node):
        for neighbor in self.neighbors:
            if neighbor[0].value == node.value:
                return neighbor[1]
        return None

    def __str__(self):
       return f"{self.value} {self.length_from_start} {self.previous_node} {self.visited}"



''' Represent the Dijkstra Algorithm '''
class Dijkstra:
    def __init__(self, graph, start, target):
        self.graph = graph
        self.start = start
        self.target = target
        self.intialization()

    ''' Initialize the labels of each vertex '''
    def intialization(self):
        for node in self.graph.nodes:
            if node == self.start:
                node.length_from_start = 0
    

    ''' Calculate the return the node with the minimum distance from the source node '''
    def minimum_distance(self):
        next_node = None
        min_value = math.inf
        for node in self.graph.nodes:
            if node.length_from_start < min_value and node.visited == False:
                min_value = node.length_from_start
                next_node = node
        print(f"min value: {min_value}, next node:{next_node.value}")

        return next_node                


    ''' The core of the algorithm. Execute the repetitive steps of Dijkstra'''
    def execution(self):
        target_node = self.graph.find_node(self.target)
        while not target_node.visited:
            # # Select the node with the minimun distrance from start
            selected_node = self.minimum_distance()
            # Update the status of the node (visited = True)
            selected_node.visited = True
            # Update the labels of the neighbors
            for node in selected_node.neighbors:
                connected_node = self.graph.find_node(node[0])
                
                if (selected_node.length_from_start + node[1]) < connected_node.length_from_start:
                    connected_node.length_from_start = selected_node.length_from_start + node[1]
                    connected_node.previous_node = selected_node.value

        # Calculate the path from the source node to target node
        path = [target_node.value]
        while True:
            print(f"path: {path}")
            node = self.graph.find_node(path[-1])
            if node.previous_node is None:
                break
            path.append(node.previous_node)
        
        path.reverse()    
        return path, target_node.length_from_start
        
    def iteration(self):
        target_node=self.graph.find_node(self.target)
        
        return 0
   
    def totalcost(self,path):
        sum=0
        for i in range(len(path)-1):
           curr_node=self.graph.find_node(path[i].value)
           next_node=self.graph.find_node(path[i+1].value)
           for node in curr_node.neighbors:
               if node[0].value==next_node.value:
                    sum+=node[1]
     #               print(f"sum:{sum},node:{node[0].value},weight:{node[1]}")
        return sum
    zero=0 #count how many paths are zero length (answer)
    total=0 # count how many paths in total
    def traverse(self,startnode,path):
        startnode.visited=True
        path.append(startnode)

        if startnode.value==self.target:
            self.total=self.total+1
            sum=self.totalcost(path)
            if sum==0:
                self.zero=self.zero+1
                for i in range (len(path)):
                    print(path[i].value,end="->")
                #    print(path[i].value, file=open('output.txt', 'a'))
                    if (path[i].value=='jj'):
                        print(f":{sum}")
        for node in startnode.neighbors:
            if node[0].visited == False:
                node[0].visited=True
                #print(f"{node[0].value}")
                #self.sum+=node[1]
               #print(f"{self.sum}")
                self.traverse(node[0],path)
        path.pop()
        startnode.visited=False
def main():
    # Create graph
    graph = Graph()
    # Add vertices
    populate_graph(graph)
    
    # Execute the algorithm
    alg = Dijkstra(graph, "a", "jj")
    path=[]
    path=alg.traverse(graph.find_node('a'),path)
    print(f"zero:{alg.zero},total:{alg.total}")
#    path, path_lenght = alg.execution()
    #print(" -> ".join(path))
    #print(f"Length of the path: { path_lenght }")

def populate_graph(graph):
    graph.add_node(Vertex('a'))
    graph.add_node(Vertex('b'))
    graph.add_node(Vertex('c'))
    graph.add_node(Vertex('d'))
    graph.add_node(Vertex('e'))
    graph.add_node(Vertex('f'))
    graph.add_node(Vertex('g'))
    graph.add_node(Vertex('h'))
    graph.add_node(Vertex('i'))
    graph.add_node(Vertex('j'))
    graph.add_node(Vertex('k'))
    graph.add_node(Vertex('l'))
    graph.add_node(Vertex('m'))
    graph.add_node(Vertex('n'))
    graph.add_node(Vertex('o'))
    graph.add_node(Vertex('p'))
    graph.add_node(Vertex('q'))
    graph.add_node(Vertex('r'))
    graph.add_node(Vertex('s'))
    graph.add_node(Vertex('t'))
    graph.add_node(Vertex('u'))
    graph.add_node(Vertex('v'))
    graph.add_node(Vertex('w'))
    graph.add_node(Vertex('x'))
    graph.add_node(Vertex('y'))
    graph.add_node(Vertex('z'))
    graph.add_node(Vertex('aa'))
    graph.add_node(Vertex('bb'))
    graph.add_node(Vertex('cc'))
    graph.add_node(Vertex('dd'))
    graph.add_node(Vertex('ee'))
    graph.add_node(Vertex('ff'))
    graph.add_node(Vertex('gg'))
    graph.add_node(Vertex('hh'))
    graph.add_node(Vertex('ii'))
    graph.add_node(Vertex('jj'))
    
    # Add edges
    graph.add_edge('a', 'b', 60)
    graph.add_edge('a', 'g', -20)
    graph.add_edge('b', 'h', -20)
    graph.add_edge('b', 'c', 30)
    graph.add_edge('c', 'i', -40)
    graph.add_edge('c', 'd', 30)
    graph.add_edge('d', 'e', 20)
    graph.add_edge('d', 'j', -10)
    graph.add_edge('e', 'f', 10)
    graph.add_edge('e', 'k', -10)
    graph.add_edge('f', 'l', -50)
    graph.add_edge('g', 'h', 20)
    graph.add_edge('g', 'm', -30)
    graph.add_edge('h', 'b', -40)
    graph.add_edge('h', 'i', 40)
    graph.add_edge('h', 'n', -10)
    graph.add_edge('h', 'g', 40)
    graph.add_edge('i', 'c', -30)
    graph.add_edge('i', 'h', 20)
    graph.add_edge('i', 'o', -10)
    graph.add_edge('j', 'd', -30)
    graph.add_edge('j', 'i', 40)
    graph.add_edge('j', 'p', -30)
    graph.add_edge('j', 'k', 10)
    graph.add_edge('k', 'e', -20)
    graph.add_edge('k', 'j', 10)
    graph.add_edge('k', 'l', 50)
    graph.add_edge('k', 'q', -20)
    graph.add_edge('l', 'k', 10)
    graph.add_edge('l', 'r', -60)
    graph.add_edge('m', 'n', 10)
    graph.add_edge('m', 's', -20)
    graph.add_edge('n', 'h', -20)
    graph.add_edge('n', 'm', 30)
    graph.add_edge('n', 't', -30)
    graph.add_edge('n', 'o', 10)
    graph.add_edge('o', 'i', -40)
    graph.add_edge('o', 'p', 30)
    graph.add_edge('o', 'n', 10)
    graph.add_edge('o', 'u', -50)
    graph.add_edge('p', 'j', -10)
    graph.add_edge('p', 'o', 10)
    graph.add_edge('p', 'v', -20)
    graph.add_edge('p', 'q', 20)
    graph.add_edge('q', 'k', -10)
    graph.add_edge('q', 'p', 30)
    graph.add_edge('q', 'w', -50)
    graph.add_edge('q', 'r', 60)
    graph.add_edge('r', 'q', 20)
    graph.add_edge('r', 'x', -30)
    graph.add_edge('s', 't', 30)
    graph.add_edge('s', 'y', -10)
    graph.add_edge('t', 'n', -10)
    graph.add_edge('t', 'u', 50)
    graph.add_edge('t', 'z', -40)
    graph.add_edge('t', 's', 20)
    graph.add_edge('u', 'o', -10)
    graph.add_edge('u', 'v', 20)
    graph.add_edge('u', 'aa', -30)
    graph.add_edge('u', 't', 30)
    graph.add_edge('v', 'p', -30)
    graph.add_edge('v', 'w', 50)
    graph.add_edge('v', 'bb', -20)
    graph.add_edge('v', 'u', 50)
    graph.add_edge('w', 'q', -20)
    graph.add_edge('w', 'x', 30)
    graph.add_edge('w', 'cc', -40)
    graph.add_edge('w', 'v', 20)
    graph.add_edge('x', 'w', 50)
    graph.add_edge('x', 'dd', -20)
    graph.add_edge('y', 'z', 40)
    graph.add_edge('y', 'ee', -30)
    graph.add_edge('z', 't', -30)
    graph.add_edge('z', 'aa', 30)
    graph.add_edge('z', 'ff', 20)
    graph.add_edge('z', 'y', 10)
    graph.add_edge('aa', 'u', -50)
    graph.add_edge('aa', 'bb', 20)
    graph.add_edge('aa', 'gg', -10)
    graph.add_edge('aa', 'z', 40)
    graph.add_edge('bb', 'v', -20)
    graph.add_edge('bb', 'cc', 40)
    graph.add_edge('bb', 'hh', -40)
    graph.add_edge('bb', 'aa', 30)
    graph.add_edge('cc', 'w', -50)
    graph.add_edge('cc', 'dd', 20)
    graph.add_edge('cc', 'ii', -20)
    graph.add_edge('cc', 'bb', 20)
    graph.add_edge('dd', 'cc', 40)
    graph.add_edge('dd', 'jj', 0)
    graph.add_edge('ee', 'ff', 20)
    graph.add_edge('ff', 'gg', 10)
    graph.add_edge('ff', 'z', -40)
    graph.add_edge('gg', 'hh', 40)
    graph.add_edge('gg', 'aa', -30)
    graph.add_edge('hh', 'ii', 20)
    graph.add_edge('hh', 'bb', -20)
    graph.add_edge('hh', 'gg', 10)
    graph.add_edge('ii', 'jj', 0)
    graph.add_edge('ii', 'hh', 40)
    graph.add_edge('ii', 'cc', -40)
    #graph.add_edge('jj', 'dd', -20)
    #graph.add_edge('jj', 'ii', 20)

if __name__ == '__main__':
    main()

# V1 -> V3 -> V4 -> V5 -> V6
# Length of the path: 11