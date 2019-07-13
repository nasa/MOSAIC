import json
import urllib2
from mosaic_schedulers.common.examples.PUFFER import MOSAICProblem
import sys


class MOSAICClient:
	def __init__(self, ServerURL='http://127.0.0.1:5000'):
		self.ServerURL = ServerURL

	def solve(self, problemState, problemSettings=None):
		if problemSettings is None:
			problemSettings = self.getDefaultSettings()

		req = urllib2.Request(self.ServerURL)
		req.add_header('Content-Type', 'application/json')

		problemStateStr = MOSAICProblem.stringifyState(problemState)
		RequestDict = {
			'State': problemStateStr,
			'Settings': problemSettings,
			'Version': 2
		}

		RequestJSON = json.dumps(RequestDict, sort_keys=True)
		try:
			response = urllib2.urlopen(req, RequestJSON)
		except urllib2.URLError, e:
			print "URLError = {}".format(str(e.reason))
			return None

		if response.code != 200:
			if response.code == 204:
				print "WARNING: Problem infeasible! (reply code {})".format(response.code)
			else:
				print "ERROR: bad HTTP reply (reply code {})".format(response.code)
			return None
		return response.read()

	def getColors(self, problemState, problemSettings=None):
		if problemSettings is None:
			problemSettings = self.getDefaultSettings()

		req = urllib2.Request(self.ServerURL + '/getColors/')
		req.add_header('Content-Type', 'application/json')

		problemStateStr = MOSAICProblem.stringifyState(problemState)
		RequestDict = {
			'State': problemStateStr,
			'Settings': problemSettings,
			'Version': 2
		}

		RequestJSON = json.dumps(RequestDict, sort_keys=True)
		try:
			response = urllib2.urlopen(req, RequestJSON)
		except urllib2.URLError, e:
			print "URLError = {}".format(str(e.reason))
			return None

		if response.code != 200:
			print "ERROR: bad HTTP reply (reply code {})".format(response.code)
			return None
		return response.read()

	def getDefaultSettings(self):
		solverParameters = {
			('read', 'datacheck'): 1,
			('mip', 'strategy', 'variableselect'): 4,
			('mip', 'strategy', 'heuristicfreq'): 50,
			('mip', 'limits', 'cutpasses'): 1
		}
		ParameterDictionary = MOSAICProblem.stringifyTupleDict(solverParameters)

		SettingsDictionary = {
			'maxLatency': 1e-7,
			'minBandwidth': 0.2,
			'solverClockLimit': 20,
			'maxHops': sys.maxint,
			'solverDetTicksLimit': 10000,
			'solverParameters': ParameterDictionary,
			'partitionMode': 'Bandwidth'
		}
		return SettingsDictionary

	def setSolverParameters(self, parameter_dictionary):
		req = urllib2.Request(self.ServerURL + '/parameters/')
		req.add_header('Content-Type', 'application/json')

		JSON_params = json.dumps(parameter_dictionary, sort_keys=True)
		try:
			response = urllib2.urlopen(req, JSON_params)
		except urllib2.URLError, e:
			print "URLError = {}".format(str(e.reason))
			return None

		if response.code != 201:
			print "ERROR: bad HTTP reply (reply code {})".format(response.code)
			return None
		else:
			JSONreply = response.read()
			SetParams = json.loads(JSONreply)
			for key, val in SetParams.iteritems():
				print("Successfully set {}: {}".format(key, val))

	def setScaleTaskTime(self, scaleTaskTime):
		stdict = {'scaleTaskTime': scaleTaskTime}
		self.setSolverParameters(stdict)


if __name__ == "__main__":
	problemState = {
		'time': 1002.0,
		'agent': 'puffer3',
		'time_horizon': 60,
		'link_state': {
			('puffer2', 'puffer1'): {'bandwidth': 11.0, 'time': 832.9},
			('puffer1', 'puffer3'): {'bandwidth': 0.0, 'time': 831.2},
			('puffer2', 'puffer3'): {'bandwidth': 11.0, 'time': 832.9},
			('puffer1', 'base_station'): {'bandwidth': 0.0, 'time': 827.5},
			('puffer3', 'base_station'): {'bandwidth': 11.0, 'time': 831.2},
			('puffer3', 'puffer1'): {'bandwidth': 0.0, 'time': 831.2},
			('puffer3', 'puffer2'): {'bandwidth': 11.0, 'time': 832.9},
			('puffer2', 'base_station'): {'bandwidth': 0.0, 'time': 832.9},
			('puffer1', 'puffer2'): {'bandwidth': 11.0, 'time': 832.9},
			('base_station', 'puffer1'): {'bandwidth': 0.0, 'time': 831.2},
			('base_station', 'puffer2'): {'bandwidth': 0.0, 'time': 831.2},
			('base_station', 'puffer3'): {'bandwidth': 11.0, 'time': 831.2},
		},
		'nodes': {
			'puffer3': {'in_science_zone': False},
			'puffer2': {'in_science_zone': False},
			'puffer1': {'in_science_zone': True},
		},
		'contact_plan': {
			('puffer1', 'puffer3'): [{'bandwidth': 0.0, 'time_start': 831.2, 'time_end': 1.7976931348623157e+308}],
			('puffer1', 'base_station'): [{'bandwidth': 11.0, 'time_start': 827.5, 'time_end': 1.7976931348623157e+308}],
			('puffer1', 'puffer2'): [{'bandwidth': 0.0, 'time_start': 832.9, 'time_end': 1.7976931348623157e+308}],
			('puffer2', 'base_station'): [{'bandwidth': 0.0, 'time_start': 827.5, 'time_end': 1.7976931348623157e+308}],
			('puffer3', 'base_station'): [{'bandwidth': 0.0, 'time_start': 831.2, 'time_end': 1.7976931348623157e+308}],
			('puffer3', 'puffer1'): [{'bandwidth': 0.0, 'time_start': 831.2, 'time_end': 1.7976931348623157e+308}],
			('puffer3', 'puffer2'): [{'bandwidth': 11.0, 'time_start': 832.9, 'time_end': 1.7976931348623157e+308}],
		}
	}

	SURL = 'http://127.0.0.1:4000'

	Client = MOSAICClient(SURL)
	# Change the time scaling of the tasks on the server - this also resets
	#  the solutions DB
	Client.setScaleTaskTime(0.5)
	# Get the default settings
	SettingsDictionary = Client.getDefaultSettings()
	# Change the time allotted to the solver
	SettingsDictionary['solverClockLimit'] = 2
	response = Client.solve(problemState, SettingsDictionary)
	if response is not None:
		print response
