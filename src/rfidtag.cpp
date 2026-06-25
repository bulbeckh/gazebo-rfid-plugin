
#include "rfidtag.h"

#include "../components/rfidtagcomponent.h"

#include <gz/plugin/Register.hh>

RFIDTag::RFIDTag()
{
}

void RFIDTag::Configure(const gz::sim::Entity &_entity,
					const std::shared_ptr<const sdf::Element> &_sdf,
					gz::sim::EntityComponentManager &_ecm,
					gz::sim::EventManager &_eventMgr)
{
	/* We assume that our RFID Tag plugin SDF element has tags directly under the <plugin> tag.
	 *
	 * <plugin name=... filename=...>
	 * 	<uid>epc-343f8e</uid>
	 * 	<data>arbitrary tag data</data>
	 * </plugin>
	 *
	 * At minimum, we must specify the <uid>. The <data> field is optional, but it must be less than 512 bytes.
	 */

	std::string tag_uid;
	std::string tag_data;

	if (_sdf->HasElement("uid"))
	{
		tag_uid = _sdf->Get<std::string>("uid");
	} else {
		gzwarn << "Failed to create tag. Tag does not specify a <uid> field.\n";
		return;
	}

	if (_sdf->HasElement("data"))
	{
		tag_data = _sdf->Get<std::string>("data");
	}

	if (tag_data.length() > 512)
	{
		gzwarn << "Failed to create Tag(uid=" << tag_uid << "). Data field exceeds 512 bytes.\n";
		return;
	}
	
	// Create component for this tag
	auto* componentptr = _ecm.CreateComponent(_entity, gz::sim::components::RFIDTag({tag_uid, tag_data}));

	if (componentptr == nullptr) {
		gzwarn << "Failed to create component for Tag(" << tag_uid << ", " << tag_data << ")\n";
	}

	return;
}

GZ_ADD_PLUGIN(RFIDTag,
		gz::sim::System,
		RFIDTag::ISystemConfigure
)

