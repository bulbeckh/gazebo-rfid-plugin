#!/bin/bash

gz service -s /rfidtagcreate --reqtype gz.custom_msgs.RFIDCreateRequest --reptype gz.custom_msgs.RFIDCreateResponse\
	-r 'tags:
		[{
			category: "cat 1",
			pose: {position: {x: 0, y: 0, z: 0 }}
		},
		{
			category: "cat 2",
			pose: {position: {x: 2, y: 3, z: -4 }}
		}]'

