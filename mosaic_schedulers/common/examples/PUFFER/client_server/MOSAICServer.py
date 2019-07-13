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

from mosaic_schedulers.common.examples.PUFFER import MOSAICProblem
import json
from flask import Flask, request
import sqlite3
import time
import argparse
import sys
import os
import copy

app = Flask(__name__)


def setUp(scaleTaskTime=.5):
    conn = sqlite3.connect('MOSAICSolutions.db')
    conn.isolation_level = 'EXCLUSIVE'
    conn.execute('BEGIN EXCLUSIVE')
    c = conn.cursor()
    # Check if table already exists
    c.execute(
        '''SELECT name FROM sqlite_master WHERE type='table' AND name='Solutions' ''')
    Table = c.fetchone()
    c.execute(
        '''SELECT name FROM sqlite_master WHERE type='table' AND name='Parameters' ''')
    Parameters = c.fetchone()
    if Table is None:
        c.execute('''CREATE TABLE Solutions
					(problem text, component text, solved int, solution text, mipstart text, solving int, taskcolors text)''')
    if Parameters is None:
        c.execute('''CREATE TABLE Parameters
					(scaleTaskTime real)''')
        c.execute('INSERT INTO Parameters (scaleTaskTime) VALUES (?)', [
                  scaleTaskTime])
    conn.commit()
    conn.close()


def resetDB(reset_solutions=True, reset_params=False):
    conn = sqlite3.connect('MOSAICSolutions.db')
    c = conn.cursor()
    if reset_solutions:
        c.execute('''DROP TABLE Solutions''')
        conn.commit()
    if reset_params:
        c.execute('''DROP TABLE Parameters''')
        conn.commit()
    conn.close()


def buildMinimalProblem(JSONproblemState):
    MinimalProblemState = {}
    # MinimalProblemState['agent'] = JSONproblemState['agent']
    MinimalProblemState['time_horizon'] = JSONproblemState['time_horizon']
    MinimalProblemState['link_state'] = {}
    for link, vals in JSONproblemState['link_state'].iteritems():
        MinimalProblemState['link_state'][link] = {
            'bandwidth': JSONproblemState['link_state'][link]['bandwidth']}
    MinimalProblemState['nodes'] = {}
    for nd, vals in JSONproblemState['nodes'].iteritems():
        MinimalProblemState['nodes'][nd] = {
            'in_science_zone': JSONproblemState['nodes'][nd]['in_science_zone']}
    return MinimalProblemState


def stripInactiveNodes(problemState):
    newProblemState = copy.deepcopy(problemState)
    deadNodes = []
    for node in problemState['nodes']:
        # We default to alive if there is no 'active' key
        if ('active' in problemState['nodes'][node]) and (problemState['nodes'][node]['active'] is False):
            newProblemState['nodes'].pop(node)
            print('Removing inactive node {}'.format(node))
            deadNodes.append(node)
    for link in problemState['link_state']:
        linkNodes = link.split(':')
        if (linkNodes[0] in deadNodes) or (linkNodes[1] in deadNodes):
            newProblemState['link_state'].pop(link)
    return newProblemState


@app.route("/", methods=['GET', 'POST'])
def MOSAICServer():
    if request.method == 'GET':
        return 'Welcome to MOSAIC! This system replies to POST requests.'
    else:
        RequestFromJSON = request.get_json()
        problemStateFromJSON = RequestFromJSON['State']
        SettingsFromJSON = RequestFromJSON['Settings']
        # Deserialize the problem instance: done server-side.
        problemStateFromJSON = stripInactiveNodes(problemStateFromJSON)
        problemState = MOSAICProblem.destringifyState(problemStateFromJSON)
        # Build minimal problem state (strip all info that makes no difference to the scheduler). Only keep 'agent', 'link_state'->'bandwidth', 'nodes'->'in_science_zone'
        MinimalProblemState = buildMinimalProblem(problemStateFromJSON)

        # Read solver parameters from request
        # Disconnect agents if latency is higher than this value and partition mode is 'Latency'
        maxLatency = SettingsFromJSON['maxLatency']
        # Disconnect agents if bandwidth is lower than this value and partition mode is 'Bandwidth'
        minBandwidth = SettingsFromJSON['minBandwidth']
        maxHops = SettingsFromJSON['maxHops']
        # Emergency stop: stop optimization if it takes longer than this many seconds (non-deterministic)
        solverClockLimit = SettingsFromJSON['solverClockLimit']
        # Deterministic time limit: Stop optimization if it takes longer than this many ticks
        solverDetTicksLimit = SettingsFromJSON['solverDetTicksLimit']
        partitionMode = SettingsFromJSON['partitionMode']
        solverParameters = MOSAICProblem.destringifyJSONDict(
            SettingsFromJSON['solverParameters'])  # Parameters for the MIP solver

        conn = sqlite3.connect('MOSAICSolutions.db')
        c = conn.cursor()
        c.execute('SELECT scaleTaskTime FROM Parameters')
        scaleTaskTime = c.fetchone()
        scaleTaskTime = scaleTaskTime[0]

        # Create a problem instance
        MProblem = MOSAICProblem.MOSAICProblem(
            problemState,
            # Disconnect agents if latency is higher than this value and partition mode is 'Latency'
            maxLatency=maxLatency,
            # Disconnect agents if bandwidth is lower than this value and partition mode is 'Bandwidth'
            minBandwidth=minBandwidth,
            maxHops=maxHops,
            # Emergency stop: stop optimization if it takes longer than this many seconds (non-deterministic)
            solverClockLimit=solverClockLimit,
            # Deterministic time limit: Stop optimization if it takes longer than this many ticks
            solverDetTicksLimit=solverDetTicksLimit,
            solverParameters=solverParameters,  # Parameters for the MIP solver
            scaleTaskTime=scaleTaskTime,
        )

        # Partition the network
        ConnectedComponents = MProblem.partition(
            mode=partitionMode)  # Partition the network
        # Figure out which partition we belong to.
        agent = problemState['agent']
        if agent not in problemState['nodes']:
            print(
                agent + ": ERROR: agent is not in list of nodes (perhaps an inactive agent?)")
            response = app.response_class(
                response=None,
                status=204,
                mimetype='application/json'
            )
            return response

        for component in ConnectedComponents:
            if agent in component:
                MyComponent = component
                break
        # Check if (partition, minimal_problem_state) is already a key in the DB
        JSONMinimalProblem = json.dumps(MinimalProblemState, sort_keys=True)
        JSONComponent = json.dumps(MyComponent, sort_keys=True)

        # Lock the DB - so we know a new write will not sneak in.
        conn.isolation_level = 'EXCLUSIVE'
        conn.execute('BEGIN EXCLUSIVE')

        c.execute('SELECT problem, component, solved, solution, mipstart, solving FROM Solutions WHERE problem=? and component=?', [
                  JSONMinimalProblem, JSONComponent])
        ExistingSol = c.fetchmany(2)
        assert (len(ExistingSol) <= 1), 'ERROR: duplicate entries in DB!'

        if len(ExistingSol) == 0:
            ExistingSol = None
        else:  # For compatibility with fetchone()
            ExistingSol = ExistingSol[0]

        # 	Do we have a solution? If we do, return it.
        if ExistingSol is not None:
            print(agent + ": Record exists - we have already tried to solve this...")
            # SQLite does not have a native Bool
            isSolved = (ExistingSol[2] > 0)
            isSolving = (ExistingSol[5] > 0)
            # Unlock the DB
            conn.commit()

            if isSolved:
                print("...and we have a solution stored. That was easy.")
                Schedule = ExistingSol[3]
            # 	If we do not, someone is computing it - wait until a timeout.
            elif isSolving:
                print(
                    "...but someone else is still working on the solution.\nWait for it...")
                retryTime = .5  # Retry the DB with this frequency
                bufferTime = 3  # We give up and return 503 after solverClockLimit + bufferTime
                waitTime = MProblem.solverClockLimit + bufferTime

                foundSol = False
                startWait = time.time()
                while time.time() - startWait < waitTime:
                    c.execute('SELECT problem, component, solved, solution, mipstart, solving FROM Solutions WHERE problem=? and component=?', [
                              JSONMinimalProblem, JSONComponent])
                    ExistingSol = c.fetchmany(2)
                    assert (len(ExistingSol) ==
                            1), 'ERROR: duplicate entries in DB!'
                    ExistingSol = ExistingSol[0]
                    inner_isSolving = (ExistingSol[5] > 0)
                    if (not inner_isSolving):  # finally the other process released a new solution
                        print(agent + ": Solution appeared in the DB!")
                        Schedule = ExistingSol[3]
                        foundSol = True
                        break
                    time.sleep(retryTime)
                # If we get here, something went horribly wrong: we did not get the solution in time as we'd expected.
                if foundSol is False:
                    response = app.response_class(
                        response='Timed out waiting on someone else to finish to compute the same problem! Please try again later',
                        status=503,
                    )
                    return response
            else:  # Nobody is solving this, but it is not optimal yet - fetch it and retry
                print("...but they did not get to optimality.\nLet's try some more.")
                c.execute('UPDATE Solutions SET solving = ? WHERE problem = ? AND component = ?', [
                          True, JSONMinimalProblem, JSONComponent])
                conn.commit()
                try:
                    # Build the scheduler
                    MProblem.buildScheduler()
                    try:
                        MIPStart = json.loads(ExistingSol[4])
                        # Get an object to terminate the optimization asynchronously
                        MProblem.scheduler.setUp()  # We need to call this manually so we can set a MIP start
                        MProblem.scheduler.setMIPStart(MIPStart)
                    except ValueError:
                        print(
                            agent + ": MIP start in the DB seems invalid/corrupted. Ignoring...")
                        pass
                    OptimizationTerminator = MProblem.getOptimizationTerminator()
                    # Call the scheduler, get back a JSON solution
                    Schedule = MProblem.schedule()
                    # Store the solution in the DB
                    MIPStart = MProblem.scheduler.getMIPStart()
                    JSONStart = json.dumps(MIPStart, sort_keys=True)
                    # RecoveredStart = json.loads(JSONStart)
                    # print type(MIPStart), type(MIPStart[0]), RecoveredStart == MIPStart
                    solverState = MProblem.scheduler.getSolverState()
                    if ((solverState == 101) or (solverState == 102)):
                        print(
                            agent + ": Optimal (within tolerance), exit code {}".format(solverState))
                        isOptimal = True
                    else:
                        print(
                            agent + ": Solver stopped: exit code {}".format(solverState))
                        isOptimal = False
                    # Get task colors and serialize them
                    _taskColors = MProblem.getTaskColors()
                    _taskColorsList = {}
                    for key, val in _taskColors.iteritems():
                        _taskColorsList[key] = val.tolist()
                    _taskColorsJSON = json.dumps(_taskColorsList)
                    # print("Solution found! Updating DB with the solution")
                    c.execute('UPDATE Solutions SET solved = ?, solution = ?, mipstart = ?, solving = ?, taskcolors = ? WHERE problem = ? AND component = ?', [
                              isOptimal, Schedule, JSONStart, False, _taskColorsJSON, JSONMinimalProblem, JSONComponent])
                    conn.commit()
                except:  # If things go south, at least unlock the solution so we don't leave the DB in an inconsistent state
                    c.execute('UPDATE Solutions SET solving = ? WHERE problem = ? AND component = ?', [
                              False, JSONMinimalProblem, JSONComponent])
                    conn.commit()
                    raise

        # If we do not have a solution, Add a key to the DB, solve, then store the solution.
        else:
            # Create a DB record
            print(
                agent + ": We never solved this problem before! Adding a record to the DB.")
            c.execute('INSERT INTO Solutions (problem, component, solved, solution, mipstart, solving, taskcolors) VALUES (?, ?, ?, ?, ?, ?, ?)', [
                      JSONMinimalProblem, JSONComponent, False, '', '', True, ''])
            # Unlock the DB
            conn.commit()
            try:
                # Build the scheduler
                MProblem.buildScheduler()
                # Get an object to terminate the optimization asynchronously
                OptimizationTerminator = MProblem.getOptimizationTerminator()
                # Call the scheduler, get back a JSON solution
                Schedule = MProblem.schedule()
                # Store the solution in the DB
                MIPStart = MProblem.scheduler.getMIPStart()
                JSONStart = json.dumps(MIPStart, sort_keys=True)
                solverState = MProblem.scheduler.getSolverState()
                if ((solverState == 101) or (solverState == 102)):
                    print(
                        agent + ":Optimal (within tolerance), exit code {}".format(solverState))
                    isOptimal = True
                elif (solverState == 103 | (Schedule is None)):
                    print(agent + ": Infeasible, exit code {}".format(solverState))
                    isOptimal = True
                else:
                    print(agent + ":Solver stopped: exit code {}".format(solverState))
                    isOptimal = False
                # Get task colors and serialize them
                _taskColors = MProblem.getTaskColors()
                _taskColorsList = {}
                for key, val in _taskColors.iteritems():
                    _taskColorsList[key] = val.tolist()
                _taskColorsJSON = json.dumps(_taskColorsList)
                #print("Solution found! Updating DB with the solution")
                c.execute('UPDATE Solutions SET solved = ?, solution = ?, mipstart = ?, solving = ?, taskcolors = ? WHERE problem = ? AND component = ?', [
                          isOptimal, Schedule, JSONStart, False, _taskColorsJSON, JSONMinimalProblem, JSONComponent])
                conn.commit()
            except:
                c.execute('UPDATE Solutions SET solving = ? WHERE problem = ? AND component = ?', [
                          False, JSONMinimalProblem, JSONComponent])
                conn.commit()
                raise

        if Schedule is not None:  # If the problem is feasible
            replyStatus = 200
        else:
            replyStatus = 204
        response = app.response_class(
            response=Schedule,
            status=replyStatus,
            mimetype='application/json'
        )
        conn.close()
        return response


@app.route("/getColors/", methods=['GET', 'POST'])
def MOSAICColorServer():
    if request.method == 'GET':
        return 'Welcome to the MOSAIC color server! This system replies to POST requests.'
    else:
        RequestFromJSON = request.get_json()
        problemStateFromJSON = RequestFromJSON['State']
        SettingsFromJSON = RequestFromJSON['Settings']
        # Deserialize the problem instance: done server-side.
        problemStateFromJSON = stripInactiveNodes(problemStateFromJSON)
        problemState = MOSAICProblem.destringifyState(problemStateFromJSON)
        # Build minimal problem state (strip all info that makes no difference to the scheduler). Only keep 'agent', 'link_state'->'bandwidth', 'nodes'->'in_science_zone'
        MinimalProblemState = buildMinimalProblem(problemStateFromJSON)

        # Read solver parameters from request
        # Disconnect agents if latency is higher than this value and partition mode is 'Latency'
        maxLatency = SettingsFromJSON['maxLatency']
        # Disconnect agents if bandwidth is lower than this value and partition mode is 'Bandwidth'
        minBandwidth = SettingsFromJSON['minBandwidth']
        maxHops = SettingsFromJSON['maxHops']
        # Emergency stop: stop optimization if it takes longer than this many seconds (non-deterministic)
        solverClockLimit = SettingsFromJSON['solverClockLimit']
        # Deterministic time limit: Stop optimization if it takes longer than this many ticks
        solverDetTicksLimit = SettingsFromJSON['solverDetTicksLimit']
        partitionMode = SettingsFromJSON['partitionMode']
        solverParameters = MOSAICProblem.destringifyJSONDict(
            SettingsFromJSON['solverParameters'])  # Parameters for the MIP solver
        conn = sqlite3.connect('MOSAICSolutions.db')
        c = conn.cursor()
        c.execute('SELECT scaleTaskTime FROM Parameters')
        scaleTaskTime = c.fetchone()
        scaleTaskTime = scaleTaskTime[0]

        # Create a problem instance
        MProblem = MOSAICProblem.MOSAICProblem(
            problemState,
            # Disconnect agents if latency is higher than this value and partition mode is 'Latency'
            maxLatency=maxLatency,
            # Disconnect agents if bandwidth is lower than this value and partition mode is 'Bandwidth'
            minBandwidth=minBandwidth,
            maxHops=maxHops,
            # Emergency stop: stop optimization if it takes longer than this many seconds (non-deterministic)
            solverClockLimit=solverClockLimit,
            # Deterministic time limit: Stop optimization if it takes longer than this many ticks
            solverDetTicksLimit=solverDetTicksLimit,
            solverParameters=solverParameters,  # Parameters for the MIP solver
            scaleTaskTime=scaleTaskTime
        )

        # Partition the network
        ConnectedComponents = MProblem.partition(
            mode=partitionMode)  # Partition the network
        # Figure out which partition we belong to.
        agent = problemState['agent']
        for component in ConnectedComponents:
            if agent in component:
                MyComponent = component
                break
        # Check if (partition, minimal_problem_state) is already a key in the DB
        JSONMinimalProblem = json.dumps(MinimalProblemState, sort_keys=True)
        JSONComponent = json.dumps(MyComponent, sort_keys=True)

        c.execute('SELECT solution, taskcolors FROM Solutions WHERE problem=? and component=?', [
                  JSONMinimalProblem, JSONComponent])
        ExistingSol = c.fetchmany(2)
        assert (len(ExistingSol) <= 1), 'ERROR: duplicate entries in DB!'
        if (len(ExistingSol) != 1):
            print(agent + 'ERROR: no color entry for this problem!')
            response = app.response_class(
                response=agent + ': ERROR: no color entry for this problem! Have you solved it yet?',
                status=404,
            )
            return response
        ExistingSol = ExistingSol[0]
        TaskColors = ExistingSol[1]
        # Reply
        print(agent + ' requesting colors for its scheduling problem.')
        response = app.response_class(
            response=TaskColors,
            status=200,
            mimetype='application/json'
        )
        conn.close()
        return response


@app.route("/parameters/", methods=['GET', 'POST'])
def MOSAICParameterServer():
    if request.method == 'GET':
        return 'Welcome to the MOSAIC parameter server! This system replies to POST requests.'
    else:
        # These parameters require us to reset the solutions table
        params_reset_solutions = ['scaleTaskTime']

        RequestFromJSON = request.get_json()
        conn = sqlite3.connect('MOSAICSolutions.db')
        c = conn.cursor()
        available_params = []
        for row in conn.execute("pragma table_info('Parameters')").fetchall():
            available_params.append(row[1])
        reset_DB = False
        for key, val in RequestFromJSON.iteritems():
            if key in available_params:
                print("Setting parameter {}={}".format(key, val))
                # Check the current value of the parameter
                c.execute('SELECT {} FROM Parameters'.format(key))
                old_val = c.fetchone()
                old_val = old_val[0]
                if val != old_val:
                    # Note that key is pre-vetted to be in available_params
                    c.execute(
                        'UPDATE Parameters SET {} = ?'.format(key), [val])
                    conn.commit()
                    if key in params_reset_solutions:
                        reset_DB = True
        if reset_DB:
            print("Resetting SQLite solution DB")
            resetDB(reset_solutions=True, reset_params=False)
            setUp()

        # Read back the parameters we set
        response_parameters = {}
        for key, val in RequestFromJSON.iteritems():
            if key in available_params:
                c.execute('SELECT {} FROM Parameters'.format(key))
                val = c.fetchone()
                val = val[0]
                response_parameters[key] = val

        response = app.response_class(
            response=json.dumps(response_parameters),
            status=201,
            mimetype='application/json'
        )
        return response


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("ERROR: no command-line arguments specified. For help, execute `MOSAICServer --help`")
        sys.exit(1)
    parser = argparse.ArgumentParser(description='Provide a server interface for the MOSAIC solver.',
                                     epilog='For full access to gunicorn options, run gunicorn [options] MOSAICServer:app from the command line.')
    parser.add_argument('--serve', '-s', action='store_true',
                        help='Starts a MOSAIC server. ')
    parser.add_argument('--bind', '-b', default='127.0.0.1:4000',
                        help='The socket to bind (passed to gunicorn).')
    parser.add_argument('--workers', '-w', default=20,
                        help='Number of workers for server (passed to gunicorn).')

    parser.add_argument('--setup', '-se', action='store_true',
                        help='Set up the SQLite database')
    parser.add_argument('--resetDB', '-r', action='store_true',
                        help='Reset the SQLite database')
    args = parser.parse_args()
    if args.setup + args.resetDB + args.serve > 1:
        print("ERROR: too many command-line arguments specified. For help, execute `MOSAICServer --help`")
        sys.exit(1)
    if args.setup:
        setUp()
        print('Setting up SQLite DB')
    if args.resetDB:
        print('Resetting SQLite DB')
        resetDB()
    if args.serve:
        setUp()
        os.system(
            "gunicorn -w {} -b {} -t 120 MOSAICServer:app".format(args.workers, args.bind))
    sys.exit(0)
