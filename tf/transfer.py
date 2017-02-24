"""This code computes the transfer function of any netlist."""
from sympy import Matrix, symbols, solve_linear_system


class Node(object):
    """Stores information on one particular node."""

    def __init__(self, info, reference):
        """Build the basic data structure with one connection."""
        if reference == 1:
            connect = 2
        else:
            connect = 1
        self.name = info[reference]
        self.connections = [{
            "node": info[connect],
            "element": info[0],
            "value": info[3],
            "connect": connect
        }]

    def connect(self, info, connect):
        """Add one element to the connections list."""
        self.connections.append({
            "node": info[connect],
            "element": info[0],
            "value": info[3],
            "connect": connect
        })


class Graph(object):
    """Stores the netlist and gives useful information."""

    def __init__(self, netlist):
        """Build the basic data structure."""
        with open(netlist, 'r') as f:
            data = f.read().split('\n')[:-1]
        self.netlist = {}
        for element in data:
            if element == "":
                continue
            info = element.split(' ')
            if info[1] not in self.netlist:
                self.netlist[info[1]] = Node(info, 1)
            else:
                self.netlist[info[1]].connect(info, 2)
            if info[2] not in self.netlist:
                self.netlist[info[2]] = Node(info, 2)
            else:
                self.netlist[info[2]].connect(info, 1)

    def build_kvl(self):
        """Build the Ax=b form of KVL equations."""
        s = symbols('s')
        variables = []
        nodes = list(self.netlist.keys())
        nodes = [int(node) for node in nodes]
        nodes.sort()

        # The splicing has been done to ignore ground node
        if nodes[0] == 0:
            nodes = nodes[1:]

        # Map from node number to array index
        nmap = {}
        nmap[0] = -1
        nmap['0'] = -1
        for index, node in enumerate(nodes):
            nmap[node] = index
            nmap[str(node)] = index

        # Main processing loop
        matrix = []
        for node in nodes:
            # The last column in equations corresponds to b
            equation = [0] * (len(nodes) + 2)
            variables.append(symbols('v' + str(node)))
            info = self.netlist[str(node)]
            for c in info.connections:
                if c["element"] == 'r':
                    equation[nmap[node]] += 1.0 / float(c["value"])
                    equation[nmap[c["node"]]] -= 1.0 / float(c["value"])
                elif c["element"] == 'c':
                    equation[nmap[node]] += s * float(c["value"])
                    equation[nmap[c["node"]]] -= s * float(c["value"])
                elif c["element"] == 'l':
                    equation[nmap[node]] += 1.0 / (s * float(c["value"]))
                    equation[nmap[c["node"]]] -= 1.0 / (s * float(c["value"]))
                elif c["element"] == 'i':
                    if c["connect"] == 1:
                        equation[-2] += float(c["value"])
                    else:
                        equation[-2] += -1 * float(c["value"])
                elif c["element"] == 'v':
                    equation = [0] * (len(nodes) + 2)
                    equation[nmap[node]] = 1
                    equation[nmap[c["node"]]] = -1
                    if c["connect"] == 1:
                        equation[-2] += -1 * float(c["value"])
                    else:
                        equation[-2] += float(c["value"])
                    break
            matrix.append(equation[:-1])
        return matrix, variables, nmap

g = Graph("netlist.txt")
matrix, variables, nmap = g.build_kvl()
solution = solve_linear_system(Matrix(matrix), *variables)
node1 = input("Enter denominator :- ")
node2 = input("Enter numerator :- ")
print solution[symbols('v' + str(node2))] / solution[symbols('v' + str(node1))]
