"""
 Copyright 2019 by California Institute of Technology.  ALL RIGHTS RESERVED.
 United  States  Government  sponsorship  acknowledged.   Any commercial use
 must   be  negotiated  with  the  Office  of  Technology  Transfer  at  the
 California Institute of Technology.
 
 This software may be subject to  U.S. export control laws  and regulations.
 By accepting this document,  the user agrees to comply  with all applicable
 U.S. export laws and regulations.  User  has the responsibility  to  obtain
 export  licenses,  or  other  export  authority  as may be required  before
 exporting  such  information  to  foreign  countries or providing access to
 foreign persons.
 
 This  software  is a copy  and  may not be current.  The latest  version is
 maintained by and may be obtained from the Mobility  and  Robotics  Sytstem
 Section (347) at the Jet  Propulsion  Laboratory.   Suggestions and patches
 are welcome and should be sent to the software's maintainer.
 
"""

import networkx as nx
import numpy as np
import matplotlib.pyplot as plt


def build_latency_graph(G, path_distance, max_diameter):
    """ Build a graph where nodes are connected iff they are close enough """
    Gl = nx.Graph()
    Gl.add_nodes_from(G.nodes())
    Gl.add_edges_from([(u, v) for u in path_distance.keys()
                       for v in path_distance[u].keys()
                       if path_distance[u][v] <= max_diameter])
    return Gl


def partition_by_latency(G,
                         max_diameter=1,
                         distance_label='Latency',
                         mode='Recursive'):
    """ Partitions a graph in a number of connected components so that
    the diameter of each connected component is lower than a given threshold.
    Inputs:
    - G, a NetworkX 2.* graph
    - max_diameter [1], the maximum diameter of each connected component
    - distance_label ['Latency'], the edge attribute re esenting the length
      of each edge. If the label is not present, each edge is assumed to have
      length 1 (in accordance with NetworkX convention).
    - mode ['Recursive']. Two modes are implemented to partition the graph
      * Recursive: finds the largest clique with diameter max_diameter, removes
        it, and iterates.
      * Coloring: finds a clique cover through graph coloring. Cliques may have
        to communicate via nodes not belonging to the clique, which is awkward.
        On the upside, it can be faster than recursive.
    Output: a list of connected components. Each node appears in exactly one
    element of the list.
    """

    # Compute pairwise distance between all nodes
    Gc = G.copy()

    # Build connected components
    ConnectedComponents = []

    if mode is 'Recursive':
        while len(Gc):
            path_distance = dict(nx.algorithms.shortest_paths.all_pairs_bellman_ford_path_length(
                Gc, weight=distance_label))
            Gl = build_latency_graph(Gc, path_distance, max_diameter)
            Cliques = list(nx.find_cliques(Gl))
            CliqueLen = list(map(lambda x: len(x), Cliques))
            MaxCliqueIdx = np.argmax(CliqueLen)
            ConnectedComponents.append(Cliques[MaxCliqueIdx])
            Gc.remove_nodes_from(Cliques[MaxCliqueIdx])
    elif mode is 'Coloring':
        path_distance = dict(nx.algorithms.shortest_paths.all_pairs_bellman_ford_path_length(
            Gc, weight=distance_label))
        Gl = build_latency_graph(Gc, path_distance, max_diameter)
        GColor = nx.complement(Gl)
        CliqueIDColor = nx.greedy_color(GColor)
        NumCliques = set(CliqueIDColor.values())
        for clique in NumCliques:
            ConnectedComponents.append(
                [nd for nd in G.nodes() if CliqueIDColor[nd] == clique])
    else:
        raise NotImplementedError
    return ConnectedComponents


def label_connected_components(G, connected_components, plot=False, pos=None):
    cliqueix = 0
    for clique in connected_components:
        for nd in clique:
            G.node[nd]['CliqueID'] = cliqueix
        cliqueix += 1
    if plot:
        if pos is None:
            nx.draw_networkx(G, node_color=list(
                map(lambda x: G.node[x]['CliqueID'], G.nodes())), pos=nx.kamada_kawai_layout(G))
        else:
            nx.draw_networkx(G, node_color=list(
                map(lambda x: G.node[x]['CliqueID'], G.nodes())), pos=pos)


def break_up_graph(G, max_cluster_size, mode='min_cut'):
    '''
    Divides a connected graph in a collection of smaller graphs, each with a
    maximum size.
    Inputs:
    - G, a networkx graph
    - max_cluster_size, the maximum size of the cluster.
    - mode. Can be 'min_cut' (recursively removes edges across the min_cut) or
      'sparsest_cut' (removes edges across the sparsest cut).
    Output:
    - A list of networkx graphs.
    '''
    if mode == 'min_cut':
        Gi = nx.Graph()
        Gi.add_nodes_from(G.nodes)
        Gi.add_edges_from(G.edges)

        # Compute the mincut
        Mincut = nx.algorithms.connectivity.minimum_edge_cut(G)
        # Remove the edges from the mincut
        Gi.remove_edges_from(Mincut)
        # Break up the graph
        Gs_nodes = list(nx.algorithms.connected_components(Gi))
        Gs = [Gi.subgraph(c) for c in Gs_nodes]
        Gout = []
        for Gnew in Gs:
            if len(Gnew) > max_cluster_size:  # Recurse
                Gout += break_up_graph(Gnew, max_cluster_size, mode)
            else:
                Gout += [Gnew]
        return Gout
    else:
        raise NotImplementedError


if __name__ == "__main__":
    import argparse
    import networkx as nx
    import timeit
    import matplotlib.pyplot as plot

    parser = argparse.ArgumentParser(
        description='Demo for the NetworkPartitioning utility.')
    parser.add_argument('--graph', default="balanced_tree",
                        help="The type of tree to generate. Options: 'balanced_tree' (default), 'barbell', 'wheel', 'complete', 'complete_multipartite'.")
    parser.add_argument('--plot', action='store_true', help='Plot the graphs')

    graph_creator = {
        'balanced_tree': nx.balanced_tree(2, 4),
        'barbell': nx.barbell_graph(20, 3),
        'wheel': nx.wheel_graph(10),
        'complete': nx.complete_graph(30),
        'complete_multipartite': nx.complete_multipartite_graph(10, 10)
    }
    # Create a graph

    args = parser.parse_args()
    G = graph_creator.get(args.graph, nx.balanced_tree(2, 4))

    if args.plot:
        nx.draw(G, pos=nx.kamada_kawai_layout(G))

    # Partition
    ConnectedComponents = partition_by_latency(
        G, max_diameter=2, mode='Recursive')
    # print(ConnectedComponents)

    # Plot and label in the graph. After this is called, each node has a CliqueID attribute.
    if args.plot:
        plt.figure()
    label_connected_components(G, ConnectedComponents, plot=args.plot)
    if args.plot:
        limits = plt.axis('off')
        plt.show()
