"""
Functions to compute the dependencies of a prototypical MOSAIC problem
in a format suitable for visualization
"""

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
import json

def related_tasks(problem_description):
    # Build a NetworkX tree with edges pointing at dependencies
    G = task_network_to_networkx(problem_description)

    # Decompose graph in connected components
    components = nx.weakly_connected_components(G)

    # Create a dict with dependent tasks
    related_tasks = {node: [] for node in G.nodes()}
    for component in components:
        for node in component:
            related_tasks[node] = list(component.nodes())
    
    return related_tasks

def d3_friendly_dependencies(problem_description):
    G = task_network_to_networkx(problem_description)

    # Prepare the graph as a D3-friendly entity
    d3_nodes = [{'name': str(i), 'optional': G.node[i]['optional']}
         for i in G.nodes()]
    d3_links = [{'source': u[0], 'target': u[1]}
            for u in G.edges()]
    d3_graph = {'nodes': d3_nodes, 'links': d3_links}

    return json.dumps(d3_graph)

def task_network_to_networkx(problem_description):
    '''
    Create a NetworkX representation of the task network
    '''
    G = nx.DiGraph()

    # problem_description is a JSON-like dictionary. If it is JSON, deserialize it.
    if type(problem_description) is str:
        problem_description = json.loads(problem_description)

    # Load dependencies
    dependency_list=problem_description["Tasks"]["DependencyList"]
    optional_tasks = problem_description["Tasks"]["OptionalTasks"]

    # Annotate each node with whether it is optional
    for task in dependency_list.keys():
        assert task in list(optional_tasks.keys())
        G.add_node(task, optional=optional_tasks[task])

    # Add edges according to dependencies
    for disjunctive_dep in dependency_list:
        if len(disjunctive_dep)>1:
            dependency_type = 'simple'
        else:
            dependency_type = 'disjunctive'
        for pred in disjunctive_dep:
            G.add_edge(pred,task, dependency_type=dependency_type)

    return G