#!/bin/bash

gz service -s /rfid_tag_create --reqtype gz.custom_msgs.RFIDTagList --reptype gz.msgs.Boolean\
	-r 'tags:
		[{
			uid: "x22434",
			data: "some tag data",
			pose: {position: {x: 0, y: 0, z: 0 }}
		},
		{
			uid: "x21989",
			pose: {position: {x: 2, y: 3, z: -4 }}
		}]'

