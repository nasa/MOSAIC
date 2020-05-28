# PUFFER-related examples for the MOSAIC scheduler

Examples in this repository encode the PUFFER FY18 problem in MOSAIC format. The software network for the problem is reported in the ICAPS 2019 paper and in the figure below.

![The PUFFER FY18 demo software network](docs/images/scenario_task_network.png)

## PUFFER input format

All scheduler examples in this folder accept a common input format that describes salient problem parameters. An example of the input format is reported below.

```json
{
  "Agent": "puffer1",
  "Agents": ["puffer1", "puffer2", "base_station"],
  "AgentStates": {
    "base_station": { },
    "puffer2": {"in_science_zone": false, "samples": 0},
    "puffer1": {"in_science_zone": true, "samples": 3}
},
	"AgentCapabilities": {
	  	"ComputationTime": {
        "puffer1": {
          "short_range_image": 3,
          "vo_localization": 10,
          "plan_path": 10,
          "send_drive_cmd": 0.1,
          "take_sample": 5,
          "analyze_sample": 10
        }
	   	},
	  	"EnergyCost": {
        "puffer1": {
          "short_range_image": 0.13333333,
          "vo_localization": 0.3333333333333333,
          "plan_path": 0.3333333333333333,
          "send_drive_cmd": 0.06666666666666667,
          "take_sample": 0.2,
          "analyze_sample": 0.3333333333333333
        }
	    }
	},
	"CommunicationNetwork": [
		{"bandwidth": 11.0,
	     "destination": "puffer1",
	     "energy_cost": 0.0,
	     "latency": 0.001,
	     "origin": "puffer1",
	     "time_end": 1.7976931348623157e+308,
	     "time_start": 831.2}
	   ],
  "Options": {}, 
	"Tasks": {
	    "ProductsSize": { 
        "short_range_image": 8.0,
        "vo_localization": 0.1,
        "plan_path": 0.1,
        "send_drive_cmd": 0.1,
        "take_sample": 15,
        "analyze_sample": 1,
        "store_sample": 0.1
	    },
	    "TaskReward": {
        "short_range_image": 0,
        "vo_localization": 0,
        "plan_path": 0,
        "send_drive_cmd": 0,
        "take_sample": 0,
        "analyze_sample": 10,
        "store_sample": 20
      },
      "DomainSpecific": {}
	},
	"Time": {
    "CurrentTime": 1001,
		"TimeHorizon": 30
	},
  "CostFunction": {
    "energy": 0,
    "total_task_reward": 1.0,
    "total_time": 0.5
  }
}
```

## ScenarioGenerator

The `ScenarioGenerator.py` software generates an instance of the PUFFER input format described above.
```
usage: ScenarioGenerator.py [-h] [--num_puffers NUM_PUFFERS]
                            [--base_station_present]
                            [--science_zone_probability SCIENCE_ZONE_PROBABILITY]
                            [--random_seed RANDOM_SEED] [--Plot]

Generates a scenario parameters file for the PUFFER FY18 demo MOSAIC scenario.

optional arguments:
  -h, --help            show this help message and exit
  --num_puffers NUM_PUFFERS
                        The number of PUFFERs in the scenario
  --base_station_present
                        Whether a base station is present
  --science_zone_probability SCIENCE_ZONE_PROBABILITY
                        The likelihood that a PUFFER is in a science zone
  --random_seed RANDOM_SEED
                        The random seed for the scenario
  --Plot                Plot the scenario and the schedule
  ```

## MILPProblemGenerator

The `MILPProblemGenerator.py` software takes as input an instance of the PUFFER input format and creates an instance of the time-varying MILP MOSAIC scheduler.

## MOSAICProblem

`MOSAICProblem.py` creates an instance of the time-varying MILP scheduler for the PUFFER problem with hard-coded task rewards and agent capabilities. It takes as input a description of the problem state containing the network state and a vector of boolean variables describing whether the agents are in a "science zone". 


## MOSAICTIProblem

`MOSAICTIProblem.py` creates an instance of the time-invariant MILP scheduler for the PUFFER problem with hard-coded task rewards and agent capabilities. Its inputs are identical to `MOSAICProblem`.