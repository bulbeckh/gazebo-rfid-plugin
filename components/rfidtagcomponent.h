#pragma once

#include <gz/sim/components/Component.hh>

/* This component holds our information about each RFID tag is and is added to each tag model entity during
 * the RFIDTagManager PreUpdate method */

struct RFIDTagData
{
	public:
		std::string uid;
		std::string data;

	public:
		bool operator==(const RFIDTagData& other) const {
			return uid == other.uid && data == other.data;
		}

		bool operator!=(const RFIDTagData& other) const {
			return !(*this == other);
		}
};

using RFIDTag = gz::sim::components::Component<RFIDTagData, class RFIDTagTag>;

