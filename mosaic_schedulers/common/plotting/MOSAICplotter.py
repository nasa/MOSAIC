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

import json
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.colors as plt_colors
from matplotlib import transforms
import random


def _randomTaskColors(Schedule):
    '''
    Generate colors for a random list of tasks
    '''
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


def SchedulePlotter(JSONSchedule, TaskColors=None):
    ''' A function to plot the MOSAIC schedule '''
    Schedule = JSONSchedule
    if type(JSONSchedule) == str:
        Schedule = json.loads(JSONSchedule)
    Schedule = Schedule['tasks']

    # Order the agents for plotting
    AgentsList = []
    for task in Schedule:
        AgentsList.append(task["params"]["agent"])
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
            Rect = patches.Rectangle((StartTime, float(AgentID)), Duration, 1., linewidth=1,
                                     edgecolor='k', facecolor=TaskColors.get(TaskType, [.1, .1, .1]))
            myax.add_patch(Rect)
            RectCx, RectCy = StartTime + \
                float(Duration) / 2., float(AgentID) + .5
            myax.annotate(TaskType, (RectCx, RectCy), color=BestTextColor(TaskColors.get(
                TaskType, [.1, .1, .1])), weight='bold', fontsize=6, ha='center', va='center')
        else:
            # Plot two rectangles connected by a line for communication
            Transmitter = AgentsList.index(task["params"]["transmitter"])
            Receiver = AgentsList.index(task["params"]["receiver"])
            Product = task["params"]["data_type"]
            Rect1 = patches.Rectangle((StartTime, float(Transmitter)), Duration, 1.,
                                      linewidth=1, edgecolor='k', facecolor=TaskColors.get(Product, [.1, .1, .1]))
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

    fig = plt.figure()
    ax = fig.add_subplot(111)  # , aspect='equal')

    min_start_time = float("inf")
    max_end_time = -float("inf")

    for task in Schedule:
        PlotScheduledTask(task, ax)
        min_start_time = min(min_start_time, task["start_time"])
        max_end_time = max(max_end_time, task["start_time"]+task["duration"])

    ax.set_xlim(min_start_time, max_end_time)
    ax.set_ylim(0, len(AgentsList))
    ax.set_xlabel('Time [s]')
    ax.set_yticks(np.array(range(len(AgentsList))) + .5)
    ax.set_yticklabels(AgentsList)
    plt.show()


def TaskAllocationPlotter(JSONSchedule, NodesLocations=None, TaskColors=None, BandwidthScale=1., OffsetGap=2):
    ''' A function to plot the allocation of tasks to agents '''

    Schedule = json.loads(JSONSchedule)
    Schedule = Schedule['tasks']

    #TaskAssignment, CommSchedule = SchedulerOutput
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
        #plt.text(NodesLocations[node]['x'],NodesLocations[node]['y'],node+': \n'+';\n'.join(TaskNames[node]))
        mytext = ax.text(NodesLocations[node]
                         ['x'], NodesLocations[node]['y'], node)
        mytext.draw(canvas.get_renderer())
        ex = mytext.get_window_extent()
        t = transforms.offset_copy(
            mytext.get_transform(), y=-2*ex.height, units='dots')
        for task in TaskNames[node]:
            mytext = ax.text(NodesLocations[node]['x'], NodesLocations[node]['y'], task, color=TaskColors.get(
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

    for od, content in CommScheduleByOD.iteritems():
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
    plt.show()
