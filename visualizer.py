from graph import Graph
from parser import Parser
import sys
import subprocess

def visualize(map_file):
    graph = Graph()
    parser = Parser()
    
    # Populate the graph
    parser.parseFile(graph, map_file)
    
    # Generate DOT syntax
    dot_content = "graph G {\n"
    # Basic layout
    dot_content += "    node [shape=circle];\n"
    
    for node_name, node in graph.nodes.items():
        # Distinguish start/end nodes
        style = ""
        if node.is_start:
            style = " [color=green, style=filled]"
        elif node.is_end:
            style = " [color=red, style=filled]"
        dot_content += f"    {node_name}{style};\n"
        
    for edge in graph.edges:
        dot_content += f"    {edge.start} -- {edge.end};\n"
    dot_content += "}\n"
    
    with open("graph.dot", "w") as f:
        f.write(dot_content)
    
    # Run dot
    subprocess.run(["dot", "-Tpng", "graph.dot", "-o", "graph.png"])
    print("Graph visualization generated: graph.png")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 visualizer.py <map_file>")
    else:
        visualize(sys.argv[1])
