"""
Functions to plot MOSAIC schedules and task allocations.
To run in headless mode, you may want to

```python
import matplotlib
matplotlib.use("Agg")
```

before importing this library.
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

from matplotlib import transforms
import matplotlib.colors as plt_colors
import matplotlib.patches as patches
import matplotlib.pyplot as plt
import random
import json
import numpy as np

def _randomTaskColors(Schedule: dict = {}):
    """    Generate colors for a random list of tasks
    
    :param Schedule: a schedule produced by a MOSAIC scheduler (described in the :doc:`API`)
    :type Schedule: dict
    :return: a dictionary with a RGB color (i.e. a list of three floats in [0,1])
             for each task in the schedule
    :rtype: dict
    """
    # Come up with a list of tasks
    TasksList = set()
    for task in Schedule:
        TaskType = task['name']
        if TaskType != "transfer_data":
            TasksList.add(TaskType)
        else:
            Product = task["params"]["data_type"]
            TasksList.add(Product)
    # Generate random hex colors
    TaskColors = {}
    for task in TasksList:
        # "%06x" % random.randint(0, 0xFFFFFF)
        TaskColors[task] = [random.random() for _ in range(3)]
    return TaskColors


def _nodesLocations(Schedule):
    """ Assign nodes locations in a circle.

    :param Schedule: a schedule produced by a MOSAIC scheduler (described in the :doc:`API`)
    :type Schedule: dict
    :return: A dictionary with node names as keys. For each node, the value is a
             dictionary with two keys, 'x' and 'y', with float values.
    :rtype: dict
    """
    nodes = set()
    for task in Schedule:
        agent = task["params"]["agent"]
        nodes.add(agent)
        taskType = task['name']
        if taskType == "transfer_data":
            sender = task["params"]["transmitter"]
            receiver = task["params"]["receiver"]
            nodes.add(sender)
            nodes.add(receiver)
    num_nodes = len(nodes)
    NodesLocations = {}
    _radius = 1
    node_ix = 0
    for node in sorted(nodes):
        _x = _radius*np.cos(float(node_ix)/float(num_nodes)*2*np.pi)
        _y = _radius*np.sin(float(node_ix)/float(num_nodes)*2*np.pi)
        NodesLocations[node] = {'x': _x, 'y': _y}
        node_ix += 1
    return NodesLocations


def SchedulePlotter(JSONSchedule, TaskColors=None, TaskShortNames=None, show=True, save=False, save_path="schedule_plot.png", aspect_ratio=1., start_time_plot=None, end_time_plot=None):
    """ A function to plot the MOSAIC schedule

    :param Schedule: a schedule produced by a MOSAIC scheduler (described in the :doc:`API`)
    :type Schedule: dict
    :param TaskColors: a dictionary with a RGB color (i.e. a list of three floats in [0,1])
                       for each task in the schedule. If None (default), random colors are used
    :type TaskColors: dict, optional
    :param TaskShortNames: a dictionary with short names for every task. If None (default),
                           the full names are printed.
    :type TaskShortNames: dict, optional
    :param show: whether to show the plot by calling plt.show(), defaults to True
    :type show: bool, optional
    :param save: whether to save the plot to disk. defaults to False
    :type save: bool, optional
    :param save_path: the file name and path where to save the plot if `save` is True.
                      Defaults to "schedule_plot.png"
    :type save_path: str, optional
    :param aspect_ratio: the aspect ratio of the plot, defaults to 1.
    :type aspect_ratio: float, optional
    :param start_time_plot: the start time of the plot. If None (default), the earliest task
                            start time is used.
    :type start_time_plot: float, optional
    :param end_time_plot: the end time of the plot. If None (default), the latest task
                            end time is used.
    :type end_time_plot: float, optional
    """
    Schedule = JSONSchedule
    if type(JSONSchedule) == str:
        Schedule = json.loads(JSONSchedule)
    if type(Schedule) is dict:
        Schedule = Schedule['tasks']

    # Order the agents for plotting
    AgentsList = []
    for task in Schedule:
        AgentsList.append(task["params"]["agent"])
        if task["name"] == "transfer_data":
            AgentsList.append(task["params"]["transmitter"])
            AgentsList.append(task["params"]["receiver"])
    AgentsList = list(set(AgentsList))
    AgentsList.sort()

    if TaskColors is None:
        TaskColors = _randomTaskColors(Schedule)

    def ComputeLuminance(RGBColor):
        RGBColor = np.array(RGBColor).astype(float)
        TempLum = np.ndarray([3, ])
        for c in range(3):
            if RGBColor[c] <= 0.03928:
                TempLum[c] = RGBColor[c] / 12.92
            else:
                TempLum[c] = ((RGBColor[c] + 0.055) / 1.055)**2.4
        return np.dot([0.2126, 0.7152, 0.0722], TempLum)

    def BestTextColor(BackgroundRGBColor):
        BackgroundLuminance = ComputeLuminance(BackgroundRGBColor)
        if BackgroundLuminance < 0.179:
            return 'w'
        else:
            return 'k'

    def PlotScheduledTask(task, myax):
        Agent = task["params"]["agent"]
        AgentID = AgentsList.index(Agent)
        StartTime = task['start_time']
        Duration = task['duration']
        TaskType = task['name']
        if TaskType != "transfer_data":
            # Plot a rectangle for computing tasks
            try:
                Rect = patches.Rectangle((StartTime, float(AgentID)), Duration, 1., linewidth=1,
                                         edgecolor='k', facecolor=TaskColors.get(TaskType, [.1, .1, .1]))
            except Exception as e:
                print("Task {}. Start time: {}, agent ID: {}, Duration: {}".format(
                    TaskType, StartTime, float(AgentID), Duration))
            myax.add_patch(Rect)
            RectCx, RectCy = StartTime + \
                float(Duration) / 2., float(AgentID) + .5
            if TaskShortNames is None:
                task_short_name = TaskType
            else:
                task_short_name = TaskShortNames.get(TaskType, TaskType)
            myax.annotate(task_short_name, (RectCx, RectCy), color=BestTextColor(TaskColors.get(
                TaskType, [.1, .1, .1])), weight='bold', fontsize=6, ha='center', va='center')
        else:
            # Plot two rectangles connected by a line for communication
            Transmitter = AgentsList.index(task["params"]["transmitter"])
            Receiver = AgentsList.index(task["params"]["receiver"])
            Product = task["params"]["data_type"]
            Rect1 = patches.Rectangle((StartTime, float(Transmitter)), Duration, 1.,
                                      linewidth=1, edgecolor='k', facecolor=TaskColors.get(Product, [.1, .1, .1]))
            if Product not in TaskColors.keys():
                print("Could not find a color for {}".format(Product))
            myax.add_patch(Rect1)
            Rect1Center = (StartTime + float(Duration) /
                           2., float(Transmitter) + .5)
            Rect2 = patches.Rectangle((StartTime, float(Receiver)), Duration, 1., linewidth=1,
                                      edgecolor='k', facecolor=TaskColors.get(Product, [.1, .1, .1]))
            myax.add_patch(Rect2)
            Rect2Center = (StartTime + float(Duration) /
                           2., float(Receiver) + .5)
            ax.plot([Rect1Center[0], Rect2Center[0]], [
                    Rect1Center[1], Rect2Center[1]], color=TaskColors.get(Product, [.1, .1, .1]))

    if save is True:
        fig = plt.figure(figsize=(24, 16), dpi=120)
    else:
        fig = plt.figure()
    ax = fig.add_subplot(111)  # , aspect='equal')

    if start_time_plot is None:
        min_start_time = float("inf")
    else:
        min_start_time = start_time_plot
    if end_time_plot is None:
        max_end_time = -float("inf")
    else:
        max_end_time = end_time_plot

    for task in Schedule:
        PlotScheduledTask(task, ax)
        if start_time_plot is None:
            min_start_time = min(min_start_time, task["start_time"])
        if end_time_plot is None:
            max_end_time = max(
                max_end_time, task["start_time"]+task["duration"])

    ax.set_xlim(min_start_time, max_end_time)
    ax.set_ylim(0, len(AgentsList))
    ax.set_xlabel('Time [s]')
    ax.set_yticks(np.array(range(len(AgentsList))) + .5)
    ax.set_yticklabels(AgentsList)
    ax.set_aspect(aspect_ratio)
    if save is True:
        plt.savefig(save_path)
    if show is True:
        plt.show()


def TaskAllocationPlotter(JSONSchedule, NodesLocations=None, TaskColors=None, TaskShortNames=None, BandwidthScale=1., OffsetGap=2, show=True, save=False, save_path="schedule_plot.png"):
    """A function to plot the allocation of tasks to agents

    :param JSONSchedule: a schedule produced by a MOSAIC scheduler (described in the :doc:`API`)
    :type JSONSchedule: dict
    :param NodesLocations: A dictionary with node names as keys. For each node, the value is a
             dictionary with two keys, 'x' and 'y', with float values. If None (default), nodes
            are set in a circle.
    :type NodesLocations: dict, optional
    :param TaskColors: a dictionary with a RGB color (i.e. a list of three floats in [0,1])
                       for each task in the schedule. If None (default), random colors are used
    :type TaskColors: dict, optional
    :param TaskShortNames: a dictionary with short names for every task. If None (default),
                           the full names are printed.
    :type TaskShortNames: dict, optional
    :param BandwidthScale: a multiplier to apply to the width of the links denoting
                           bandwidth between the agents, defaults to 1.
    :type BandwidthScale: float, optional
    :param OffsetGap: the offset between parallel links denoting bandwidth, defaults to 2
    :type OffsetGap: int, optional
    :param show: whether to show the plot by calling plt.show(), defaults to True
    :type show: bool, optional
    :param save: whether to save the plot to disk. defaults to False
    :type save: bool, optional
    :param save_path: the file name and path where to save the plot if `save` is True.
                      Defaults to "schedule_plot.png"
    :type save_path: str, optional
    """

    if type(JSONSchedule) is str:
        Schedule = json.loads(JSONSchedule)
    else:
        Schedule = JSONSchedule
    if type(Schedule) is dict:
        Schedule = Schedule['tasks']

    # TaskAssignment, CommSchedule = SchedulerOutput
    fig = plt.figure()
    ax = plt.gca()
    t = ax.transData
    canvas = ax.figure.canvas

    if TaskColors is None:
        TaskColors = _randomTaskColors(Schedule)

    if NodesLocations is None:
        NodesLocations = _nodesLocations(Schedule)

    # Plot the nodes
    TaskNames = {}
    for node in NodesLocations:
        plt.plot(NodesLocations[node]['x'], NodesLocations[node]['y'], '.')
        TaskNames[node] = []
    # For each node, build the string of task names
    for task in Schedule:
        agent = task["params"]["agent"]
        taskType = task['name']
        if taskType != "transfer_data":
            TaskNames[agent] += [taskType]
    for node in NodesLocations:
        # plt.text(NodesLocations[node]['x'],NodesLocations[node]['y'],node+': \n'+';\n'.join(TaskNames[node]))
        mytext = ax.text(NodesLocations[node]
                         ['x'], NodesLocations[node]['y'], node)
        mytext.draw(canvas.get_renderer())
        ex = mytext.get_window_extent()
        t = transforms.offset_copy(
            mytext.get_transform(), y=-2*ex.height, units='dots')
        for task in TaskNames[node]:
            if TaskShortNames is None:
                task_short_name = task
            else:
                task_short_name = TaskShortNames.get(task, task)
            mytext = ax.text(NodesLocations[node]['x'], NodesLocations[node]['y'], task_short_name, color=TaskColors.get(
                task, [.1, .1, .1]), transform=t)
            mytext.draw(canvas.get_renderer())
            ex = mytext.get_window_extent()
            t = transforms.offset_copy(
                mytext.get_transform(), y=-2*ex.height, units='dots')

    # Plot communication links
    CommScheduleByOD = {}
    for task in Schedule:
        agent = task["params"]["agent"]
        taskType = task['name']
        if taskType == "transfer_data":
            sender = task["params"]["transmitter"]
            receiver = task["params"]["receiver"]
            taskName = task["params"]["data_type"]
            bandwidth = task["params"]["bandwidth"]
            od = (sender, receiver)
            if od not in CommScheduleByOD:
                CommScheduleByOD[od] = []
            CommScheduleByOD[od] += [(taskName, bandwidth)]

    for od, content in CommScheduleByOD.items():
        sender, receiver = od
        vector = np.array([NodesLocations[receiver]['x']-NodesLocations[sender]
                           ['x'], NodesLocations[receiver]['y']-NodesLocations[sender]['y']])
        offsetDir = np.array([-vector[1], vector[0]])  # Offset direction
        offsetMag = 0.
        for msg in content:
            task, bandwidth = msg
            offsetMag += 1.2*float(bandwidth)*BandwidthScale + OffsetGap
            offset = -(offsetDir/np.linalg.norm(offsetDir)) * \
                offsetMag  # Magnitude
            t = transforms.offset_copy(
                ax.transData, x=offset[0], y=offset[1], fig=fig, units='points')
            plt.arrow(NodesLocations[sender]['x'], NodesLocations[sender]['y'],
                      NodesLocations[receiver]['x'] -
                      NodesLocations[sender]['x'], NodesLocations[receiver]['y'] -
                      NodesLocations[sender]['y'],
                      linewidth=float(bandwidth) *
                      BandwidthScale,  # Width in points
                      color=TaskColors.get(task, [.1, .1, .1]),
                      transform=t
                      )
    plt.axis('off')
    if save is True:
        plt.savefig(save_path)
    if show is True:
        plt.show()


def TaskAllocationPlotter_PGV(
    timeline,
    TaskColors: dict=None,
    NodeColors: dict=None,
    NodesLocations: dict=None,
    TaskShortNames: dict=None,
    BandwidthScale=1.,
    PositionScale=1.,
    save_path="PGVallocation.pdf",
    node_filled=True,
):  
    """ A function to plot the allocation of tasks to agents with GraphViz

    :param timeline: a schedule produced by a MOSAIC scheduler (described in the :doc:`API`)
    :type timeline: dict
    :param TaskColors: a dictionary with a RGB color (i.e. a list of three floats in [0,1])
                       for each task in the schedule. If None (default), random colors are used
    :type TaskColors: dict, optional
    :param NodeColors: a dictionary with a RGB color (i.e. a list of three floats in [0,1])
                       for each agent in the schedule. If None (default), random colors are used
    :type NodeColors: dict, optional
    :param NodesLocations: A dictionary with node names as keys. For each node, the value is a
             dictionary with two keys, 'x' and 'y', with float values. If None (default), nodes
            are set in a circle.
    :type NodesLocations: dict, optional
    :param TaskShortNames: NOT USED. Kept for compatibility with TaskAllocationPlotter
    :type TaskShortNames: dict, optional
    :param BandwidthScale: a multiplier to apply to the width of the links denoting
                           bandwidth between the agents, defaults to 1.
    :type BandwidthScale: float, optional
    :param PositionScale: A scaling factor applied to the nodes' locations. Defaults to 1.
    :type PositionScale: float, optional
    :param save_path: the file name and path where to save the plot if `save` is True.
                      Defaults to "schedule_plot.png"
    :type save_path: str, optional
    :param node_filled: whether the nodes should be filled in the plot. Defaults to True
    :type node_filled: bool, optional
    """
    import pygraphviz as pgv

    if type(timeline) == str:
        timeline = json.loads(timeline)
    if list(timeline.keys()) == ['tasks']:
        timeline = timeline['tasks']

    # If the nodes locations are not specified, place in a circle
    # TODO arrange according to bandwidth
    if NodesLocations is None:
        NodesLocations = _nodesLocations(timeline)

    # Get list of tasks

    nodes = NodesLocations.keys()

    if NodeColors is None:
        import random
        NodeColors = {}
        for node in nodes:
            NodeColors[node] = [random.random(), random.random(),
                                random.random()]

    if TaskColors is None:
        TaskColors = _randomTaskColors(timeline)

    G = pgv.AGraph(strict=False, directed=True)
    for node in nodes:
        G.add_node(node, color=plt_colors.to_hex(NodeColors[node]), pos="{},{}!".format(
            NodesLocations[node]['x']*PositionScale, NodesLocations[node]['y']*PositionScale))

    for task in timeline:
        agent = task["params"]["agent"]
        taskType = task['name']
        if taskType == "transfer_data":
            sender = task["params"]["transmitter"]
            receiver = task["params"]["receiver"]
            taskName = task["params"]["data_type"]
            bandwidth = task["params"]["bandwidth"]
            if bandwidth > 0:
                G.add_edge(
                    sender,
                    receiver,
                    color=plt_colors.to_hex(TaskColors[taskName]),
                    penwidth=bandwidth*BandwidthScale
                )

    if node_filled:
        G.node_attr['style'] = 'filled'
    G.layout('neato')
    G.draw(save_path)
