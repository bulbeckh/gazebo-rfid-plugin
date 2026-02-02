#!/bin/bash

gz service -s /rfid_tag_remove --reqtype gz.custom_msgs.RFIDTagList --reptype gz.msgs.Boolean\
	-r 'tags:
		[{
			uid: "x22434",
		}]'
