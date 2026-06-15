
#include "rfidtag.h"

#include "components/rfidtagcomponent.h"
#include <gz/plugin/Register.hh>

GZ_ADD_PLUGIN(RFIDTagPlugin,
		gz::sim::System,
		RFIDTagPlugin::ISystemConfigure,
		RFIDTagPlugin::ISystemPreUpdate
)

RFIDTagPlugin::RFIDTagPlugin()
{
}

void RFIDTagPlugin::Configure(const gz::sim::Entity &_entity,
					const std::shared_ptr<const sdf::Element> &_sdf,
					gz::sim::EntityComponentManager &_ecm,
					gz::sim::EventManager &_eventMgr)
{
	if (_sdf->HasElement("uid")) tag_uid = _sdf->Get<std::string>("uid");
	if (_sdf->HasElement("data")) tag_data = _sdf->Get<std::string>("data");

	gzmsg << "Found SDF Tag: Tag(" << tag_uid << ", " << tag_data << ")\n";
	
	// Create component for this tag
	auto* componentptr = _ecm.CreateComponent(_entity, RFIDTag({tag_uid, tag_data}));

	// If we successfully created, then remove from list
	if (componentptr == nullptr) {
		gzwarn << "Failed to create Tag(" << tag_uid << ", " << tag_data << ")\n";
	}
}

void RFIDTagPlugin::PreUpdate(const gz::sim::UpdateInfo &_info,
		gz::sim::EntityComponentManager &_ecm)
{
	// TODO
}


