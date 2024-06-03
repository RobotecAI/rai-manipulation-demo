
#include <AzCore/Serialization/SerializeContext.h>

#include "RAIManipulationDemoSystemComponent.h"

#include <RAIManipulationDemo/RAIManipulationDemoTypeIds.h>

namespace RAIManipulationDemo
{
    AZ_COMPONENT_IMPL(RAIManipulationDemoSystemComponent, "RAIManipulationDemoSystemComponent",
        RAIManipulationDemoSystemComponentTypeId);

    void RAIManipulationDemoSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<RAIManipulationDemoSystemComponent, AZ::Component>()
                ->Version(0)
                ;
        }
    }

    void RAIManipulationDemoSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("RAIManipulationDemoService"));
    }

    void RAIManipulationDemoSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("RAIManipulationDemoService"));
    }

    void RAIManipulationDemoSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void RAIManipulationDemoSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    RAIManipulationDemoSystemComponent::RAIManipulationDemoSystemComponent()
    {
        if (RAIManipulationDemoInterface::Get() == nullptr)
        {
            RAIManipulationDemoInterface::Register(this);
        }
    }

    RAIManipulationDemoSystemComponent::~RAIManipulationDemoSystemComponent()
    {
        if (RAIManipulationDemoInterface::Get() == this)
        {
            RAIManipulationDemoInterface::Unregister(this);
        }
    }

    void RAIManipulationDemoSystemComponent::Init()
    {
    }

    void RAIManipulationDemoSystemComponent::Activate()
    {
        RAIManipulationDemoRequestBus::Handler::BusConnect();
    }

    void RAIManipulationDemoSystemComponent::Deactivate()
    {
        RAIManipulationDemoRequestBus::Handler::BusDisconnect();
    }
}
