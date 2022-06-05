#!/usr/bin/env python3

class SystemValues:

	teleop_input_name = "teleop_input"
	current_state = "current_state"

	emergency_code = 111

	StateControllerManager = None

	mow_command_service_name = "mow_command"
	setup_command_service_name = "setup_command"

	dataPath = "/home/alex20004044/Diploma/Lawnmower-Robot-Control-System/Data/"
	dataZoneName = "DataZone.json"
	basePointName = "BasePoint.json"

	greenZoneMapName = "GreenZoneMap.pgm"
	yellowZoneMapName = "YellowZoneMap.pgm"
	redZoneMapName = "RedZoneMap.pgm"

	mowZoneMapName = "MowZoneMap.pgm"
	roadZoneMapName = "RoadZoneMap.pgm"

	roadMapTopicName = "map"
	mowMapTopicName = "mow_map"

	clickedPointsTopicName = "/clicked_point"

	map_width = 5000
	map_height = 5000
	map_resolution = 0.05
	#map_origin = (-125,-125)
	map_origin = (-100, -100)