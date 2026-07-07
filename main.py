from graph import Graph
from parser import Parser

def main():
    print("hh")
    breakpoint()
    graph: Graph = Graph();
    parser: Parser = Parser()
    parser.parseFile(graph, "./maps/challenger/01_the_impossible_dream.txt")
    print(graph.drones)
    print(graph.nodes)
    print(graph.edges)

main()
