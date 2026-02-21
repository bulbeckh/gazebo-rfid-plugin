#!/bin/bash

gz service -s /rfid_tag_create --reqtype gz.custom_msgs.RFIDTagList --reptype gz.msgs.Boolean\
	-r 'tags:
		[{
			uid: "x22434",
			data: "some tag data",
			pose: {position: {x: 2, y: 1, z: 1 }}
		},
		{
			uid: "asd22",
			data: "nodata",
			pose: {position: {x: 3, y: 1, z: 1 }}

		},
		{
			uid: "oiwuer980w98e",
			data: "some other tag data to be added",
			pose: {position: {x: 1, y: 1, z: 1}}
		}]'
