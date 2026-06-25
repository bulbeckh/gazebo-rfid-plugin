#pragma once

#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>

/* This component holds our information about each RFID tag is and is added to each tag model entity during
 * the RFIDTagManager PreUpdate method */

namespace gz
{
namespace sim
{
namespace components
{

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

	using RFIDTag = Component<RFIDTagData, class RFIDTagTag>;

	GZ_SIM_REGISTER_COMPONENT("gz_sim_components.RFIDTag", RFIDTag)
}

}

}
