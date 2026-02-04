#!/bin/bash

gz service -s /rfid_tag_create --reqtype gz.custom_msgs.RFIDTagList --reptype gz.msgs.Boolean\
	-r 'tags:
		[{
			uid: "x22434",
			data: "some tag data",
			pose: {position: {x: 5, y: 2, z: 3 }}
		}]'
